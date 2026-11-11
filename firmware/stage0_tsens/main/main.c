/* =============================================================================
 * ESP32-S3 环境监测终端 · Stage 0
 * -----------------------------------------------------------------------------
 * 完整数据链：
 *   BOOT 键(GPIO0) --中断--> 队列 --> btn_task --> 翻转加热开关 s_heat_on
 *   heat_task(绑核1) 满载烧 CPU  -->  芯片升温
 *   TSENS 采温度 --> 滑动平均 --> ① 串口 CSV  ② WS2812 按阈值变色
 *
 * 串口输出两行（各司其职）：
 *   给人看：加热[开] 原始 33.80  滤波 33.92 °C
 *   给机器：DATA,毫秒,原始值,滤波值,加热状态      ← Python 只抓 DATA 行
 *
 * 相比"按键换色"版的改动：
 *   1. 按键不再直接换颜色，改为控制加热开关；颜色交给温度决定
 *   2. 新增温度传感器采集 + 滑动平均滤波
 *   3. 新增 CSV 输出（Stage 0 的核心）
 *   4. heat_task 加了 vTaskDelay，修掉看门狗告警
 *
 * 硬件（ESP32-S3-DevKitC-1 N16R8）：
 *   GPIO48 = WS2812 RGB 灯    GPIO0 = BOOT 键（内部上拉，下降沿触发）
 * ============================================================================= */

#include <stdio.h>

#include "driver/gpio.h"
#include "driver/temperature_sensor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "led_strip.h"

/* ---------------- 配置区：想改参数就改这里 ---------------- */
#define LED_GPIO     48        /* 板载 WS2812 */
#define BTN_GPIO     0         /* BOOT 键 */
#define SAMPLE_MS    500       /* 采样周期（毫秒） */
#define FILTER_N     8         /* 滑动平均窗口（滞后 = N × SAMPLE_MS / 2） */
#define T_WARN       36.0f     /* 蓝 → 绿 阈值 */
#define T_ALARM      42.0f     /* 绿 → 红 阈值 */
#define DEBOUNCE_US  (50 * 1000)   /* 按键消抖 50ms */

static const char *TAG = "env";

/* ---------------- 全局句柄 ---------------- */
static led_strip_handle_t s_strip = NULL;
static QueueHandle_t      s_queue = NULL;
static volatile bool      s_heat_on = false;   /* ISR → 任务 → 主循环共享，必须 volatile */

/* =============================================================================
 * 滑动平均滤波器
 * 作用：把 1°C 分辨率的阶梯，变成 0.1°C 级别的平滑曲线
 * 代价：滞后 N/2 个采样周期（N=8、500ms → 4 秒）
 * ============================================================================= */
static float s_buf[FILTER_N];
static int   s_idx = 0;
static bool  s_filled = false;

static float filter_push(float v)
{
    s_buf[s_idx] = v;
    s_idx = (s_idx + 1) % FILTER_N;
    if (s_idx == 0) {
        s_filled = true;
    }
    int n = s_filled ? FILTER_N : s_idx;

    float sum = 0;
    for (int i = 0; i < n; i++) {
        sum += s_buf[i];
    }
    return sum / n;
}

/* =============================================================================
 * 按键 ISR —— 只做"记一笔"，快进快出
 * IRAM_ATTR：把函数放进内部 RAM，Flash 被擦写时中断也能正常响应
 * 这里的 s_last_us 只在 ISR 内部使用，所以不需要 volatile
 * ============================================================================= */
static void IRAM_ATTR btn_isr_handler(void *arg)
{
    static int64_t s_last_us = 0;
    int64_t now = esp_timer_get_time();

    if (now - s_last_us < DEBOUNCE_US) {
        return;                       /* 50ms 内的重复触发 = 机械抖动，丢弃 */
    }
    s_last_us = now;

    uint32_t tick = (uint32_t)(now / 1000);
    BaseType_t higher_woken = pdFALSE;
    xQueueSendFromISR(s_queue, &tick, &higher_woken);   /* ISR 里必须用 FromISR 版 API */
    if (higher_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/* =============================================================================
 * 按键任务 —— ISR 之外的活都在这里干（打印、改状态）
 * ============================================================================= */
static void btn_task(void *arg)
{
    uint32_t tick;
    while (1) {
        if (xQueueReceive(s_queue, &tick, portMAX_DELAY) == pdTRUE) {
            s_heat_on = !s_heat_on;
            ESP_LOGI(TAG, "按键 @ %u ms → 加热 %s", (unsigned)tick, s_heat_on ? "开" : "关");
        }
    }
}

/* =============================================================================
 * 加热任务 —— 满载烧 CPU，绑在核 1（核 0 留给主逻辑和将来的 WiFi）
 * ⚠️ 关键：忙 90ms 后必须 vTaskDelay(10ms) 让出 CPU，
 *    否则核 1 的 IDLE 任务永远排不上队 → 喂不了看门狗 → 每 5 秒刷 task_wdt 告警
 * ============================================================================= */
static void heat_task(void *arg)
{
    while (1) {
        if (s_heat_on) {
            uint32_t t0 = xTaskGetTickCount();
            while (xTaskGetTickCount() - t0 < pdMS_TO_TICKS(90)) {
                for (volatile int i = 0; i < 10000; i++) {
                    /* 空转消耗 CPU；volatile 防止被编译器优化掉 */
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));      /* 让出 CPU，占空比仍有 90% */
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/* =============================================================================
 * 主流程
 * ============================================================================= */
void app_main(void)
{
    /* ---------- 1. WS2812（RMT 驱动，普通 gpio_set_level 点不亮） ---------- */
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,                /* 板载只有 1 颗 */
        /* 其余字段用默认值：LED_MODEL_WS2812 + GRB 顺序 */
    };
    led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,   /* 10MHz → 0.1us 分辨率，够表达 400/800ns 脉冲 */
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip));
    ESP_ERROR_CHECK(led_strip_clear(s_strip));

    /* ---------- 2. 内置温度传感器 ----------
     * 量程 10~60 是有讲究的：官方预置档位里 "-10~80" 这一档误差 <1°C，
     * 而我们之前用 (-10, 110) 跨了档位边界，直接安装失败。
     * 档位对照：50~125(<3°C) / 20~100(<2°C) / -10~80(<1°C) / -30~50(<2°C) / -40~20(<3°C)
     */
    temperature_sensor_handle_t tsens = NULL;
    temperature_sensor_config_t tsens_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 60);
    ESP_ERROR_CHECK(temperature_sensor_install(&tsens_cfg, &tsens));
    ESP_ERROR_CHECK(temperature_sensor_enable(tsens));

    /* ---------- 3. 按键 GPIO + 中断 ---------- */
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,       /* 按下 = 拉低 = 下降沿 */
    };
    ESP_ERROR_CHECK(gpio_config(&io_cfg));

    s_queue = xQueueCreate(8, sizeof(uint32_t));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_GPIO, btn_isr_handler, NULL));

    /* ---------- 4. 启动任务 ---------- */
    xTaskCreate(btn_task, "btn", 4096, NULL, 5, NULL);
    xTaskCreatePinnedToCore(heat_task, "heat", 4096, NULL, 5, NULL, 1);   /* 核 1 */

    /* ---------- 5. 主循环：采样 → 滤波 → 输出 → 显示 ---------- */
    ESP_LOGI(TAG, "就绪。按 BOOT 键开/关加热，建议观察 90~120 秒");
    printf("DATA_HEAD,t_ms,raw,filtered,heat\n");   /* 表头，方便确认格式 */

    while (1) {
        float raw = 0;
        ESP_ERROR_CHECK(temperature_sensor_get_celsius(tsens, &raw));
        float filt = filter_push(raw);

        /* 给人看 */
        printf("加热[%s] 原始 %.2f  滤波 %.2f °C\n",
               s_heat_on ? "开" : "关", raw, filt);

        /* 给机器看（Python 只抓 DATA 开头的行） */
        printf("DATA,%lld,%.2f,%.2f,%d\n",
               (long long)(esp_timer_get_time() / 1000), raw, filt, s_heat_on ? 1 : 0);

        /* 按温度阈值变色：<36 蓝 / <42 绿 / ≥42 红
         * TODO: 后面加滞回（超 42 报警、降到 40 才解除），避免临界点疯狂闪烁
         */
        if (filt < T_WARN) {
            ESP_ERROR_CHECK(led_strip_set_pixel(s_strip, 0, 0, 0, 40));    /* 蓝 */
        } else if (filt < T_ALARM) {
            ESP_ERROR_CHECK(led_strip_set_pixel(s_strip, 0, 0, 40, 0));    /* 绿 */
        } else {
            ESP_ERROR_CHECK(led_strip_set_pixel(s_strip, 0, 40, 0, 0));    /* 红 */
        }
        ESP_ERROR_CHECK(led_strip_refresh(s_strip));

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
    }
}
