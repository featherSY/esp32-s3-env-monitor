/* =============================================================================
 * ESP32-S3 Stage 1：ADC 标定与有效位数测量
 * -----------------------------------------------------------------------------
 * 接线：电位器两端接 3.3V / GND，中间抽头 → GPIO4 (ADC1_CH3)
 *
 * 交互流程（按 BOOT 键推进每一步，等待时实时打印当前读数）：
 *   step0  旋到最小      → 记零点 min_raw
 *   step1  旋到最大      → 记满度 max_raw（自动检测是否饱和）
 *   step2  旋到 1/4 圈   → 线性度点 1
 *   step3  旋到 1/2 圈   → 线性度点 2
 *   step4  旋到 3/4 圈   → 线性度点 3
 *   step5  保持不动      → 连采 100 次 → 标准差 σ → 有效位数 ENOB
 *   step6  打印完整标定报告
 *
 * ⚠️ 注意 POT_TURNS：3296W 常见是 25 圈，拿到手先自己数一遍，
 *    从最小旋到最大，数出来的总圈数填到下面的宏里（两端空转不算）
 * ============================================================================= */

#include <math.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/* ---------------- 配置 ---------------- */
#define POT_GPIO   4            /* GPIO4 = ADC1_CH3（必须用 ADC1，WiFi 会抢 ADC2） */
#define BTN_GPIO   0            /* BOOT 键 */
#define ADC_CH     ADC_CHANNEL_3
#define POT_TURNS  25           /* ← 改成你数出来的总圈数 */
#define VREF_MV    3300
#define N_AVG      32           /* 单次读取的硬件平均次数 */
#define N_REPEAT   100          /* 重复性测试的采样数 */

static const char *TAG = "calib";

static QueueHandle_t             s_queue = NULL;
static adc_oneshot_unit_handle_t s_adc   = NULL;

/* 标定结果 */
static int   s_min_raw = 0, s_max_raw = 4095;
static int   s_lin_actual[3], s_lin_theory[3];
static float s_sigma = 0;

/* =============================================================================
 * 读一次 ADC（内部做 N_AVG 次平均，抑制随机噪声）
 * ============================================================================= */
static int read_raw_avg(int n)
{
    long sum = 0;
    for (int i = 0; i < n; i++) {
        int raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(s_adc, ADC_CH, &raw));
        sum += raw;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return (int)(sum / n);
}

/* =============================================================================
 * 按键 ISR：50ms 消抖 + 队列
 * ============================================================================= */
static void IRAM_ATTR btn_isr_handler(void *arg)
{
    static int64_t s_last_us = 0;
    int64_t now = esp_timer_get_time();
    if (now - s_last_us < 50 * 1000) {
        return;
    }
    s_last_us = now;

    uint32_t tick = (uint32_t)(now / 1000);
    BaseType_t hpw = pdFALSE;
    xQueueSendFromISR(s_queue, &tick, &hpw);
    if (hpw == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/* =============================================================================
 * 主流程：状态机
 * ============================================================================= */
void app_main(void)
{
    /* ---------- ADC 初始化 ---------- */
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_11,      /* 量程约 0.15 ~ 3.1 V */
        .bitwidth = ADC_BITWIDTH_DEFAULT, /* 12 位 → 0~4095 */
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CH, &chan_cfg));

    /* ---------- 按键 ---------- */
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << BTN_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_cfg));
    s_queue = xQueueCreate(4, sizeof(uint32_t));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_GPIO, btn_isr_handler, NULL));

    ESP_LOGI(TAG, "ADC 标定程序就绪，当前电位器总圈数设为 %d", POT_TURNS);

    /* ---------- 状态机 ---------- */
    const char *prompts[] = {
        "① 把电位器【逆时针旋到底】(最小)，然后按 BOOT",
        "② 把电位器【顺时针旋到底】(最大)，然后按 BOOT",
        "③ 从最小往回旋 %d 圈(约 1/4)，然后按 BOOT",
        "④ 从最小往回旋 %d 圈(约 1/2)，然后按 BOOT",
        "⑤ 从最小往回旋 %d 圈(约 3/4)，然后按 BOOT",
        "⑥ 【保持电位器不要动】，按 BOOT 开始重复性测试(100 次)",
        "⑦ 按 BOOT 生成【标定报告】",
    };

    int  step = 0;
    bool done = false;

    printf("\n  >>> %s\n\n", prompts[0]);

    while (!done) {
        /* 等待期间实时打印当前读数，方便你一边旋一边看 */
        int raw = read_raw_avg(8);
        printf("  step%d | raw=%4d  ≈ %4d mV\r\n", step, raw,
               (int)((float)raw * VREF_MV / 4095.0f));
        fflush(stdout);

        /* 非阻塞检查按键（超时 0，不耽误打印） */
        uint32_t tick;
        if (xQueueReceive(s_queue, &tick, 0) != pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }

        printf("\n");
        switch (step) {
        case 0:
            s_min_raw = read_raw_avg(N_AVG);
            printf("  → 零点记录 raw = %d\n", s_min_raw);
            break;

        case 1:
            s_max_raw = read_raw_avg(N_AVG);
            printf("  → 满度记录 raw = %d", s_max_raw);
            if (s_max_raw >= 4090) {
                printf("  ⚠️ 已接近饱和(4095)！11dB 衰减量程约 3.1V，"
                       "接 3.3V 会削顶，建议串电阻分压或改用 0~3.1V 区间\n");
            } else {
                printf("\n");
            }
            break;

        case 2:
        case 3:
        case 4: {
            int idx   = step - 2;
            int turns = (idx == 0) ? POT_TURNS / 4
                      : (idx == 1) ? POT_TURNS / 2
                      : POT_TURNS * 3 / 4;
            s_lin_actual[idx] = read_raw_avg(N_AVG);
            /* 理论值：按线性电位器，圈数比例 = 电压比例 */
            float ratio = (float)turns / (float)POT_TURNS;
            s_lin_theory[idx] = s_min_raw + (int)((s_max_raw - s_min_raw) * ratio);
            printf("  → 第 %d 圈实测 raw=%d，理论 raw=%d，偏差 %d LSB\n",
                   turns, s_lin_actual[idx], s_lin_theory[idx],
                   s_lin_actual[idx] - s_lin_theory[idx]);
            break;
        }

        case 5: {
            printf("  采样中，请勿触碰电位器...\n");
            int vals[N_REPEAT];
            long sum = 0;
            for (int i = 0; i < N_REPEAT; i++) {
                vals[i] = read_raw_avg(4);
                sum += vals[i];
            }
            float mean = (float)sum / N_REPEAT;
            double var = 0;
            for (int i = 0; i < N_REPEAT; i++) {
                double d = vals[i] - mean;
                var += d * d;
            }
            s_sigma = (float)sqrt(var / N_REPEAT);
            printf("  → 均值 %.1f LSB，标准差 σ = %.2f LSB\n", mean, s_sigma);
            break;
        }

        case 6:
            done = true;
            break;
        }

        if (!done) {
            step++;
            char buf[128];
            if (step >= 2 && step <= 4) {
                int turns = (step == 2) ? POT_TURNS / 4
                          : (step == 3) ? POT_TURNS / 2
                          : POT_TURNS * 3 / 4;
                snprintf(buf, sizeof(buf), prompts[step], turns);
            } else {
                snprintf(buf, sizeof(buf), "%s", prompts[step]);
            }
            printf("\n  >>> %s\n\n", buf);
        }
    }

    /* ================= 标定报告 ================= */
    printf("\n");
    printf("══════════════ ADC 标定报告 ══════════════\n");
    printf("  量程(12bit)   : 0 ~ 4095 LSB\n");
    printf("  零点 raw      : %d\n", s_min_raw);
    printf("  满度 raw      : %d\n", s_max_raw);
    printf("  有效跨度      : %d LSB（占满量程 %.1f%%）\n",
           s_max_raw - s_min_raw, (s_max_raw - s_min_raw) / 4095.0f * 100);
    printf("  标定系数      : mV = (raw - %d) × %.4f\n",
           s_min_raw, VREF_MV / (float)(s_max_raw - s_min_raw));

    printf("\n  【线性度 INL】\n");
    for (int i = 0; i < 3; i++) {
        int err = s_lin_actual[i] - s_lin_theory[i];
        printf("    点%d  实测 %4d  理论 %4d  偏差 %+4d LSB  (%.2f%% FS)\n",
               i + 1, s_lin_actual[i], s_lin_theory[i], err,
               (float)err / 4095.0f * 100);
    }

    printf("\n  【重复性与有效位数】\n");
    printf("    噪声标准差 σ : %.2f LSB\n", s_sigma);
    if (s_sigma > 0) {
        float enob = 12.0f - log2f(s_sigma);
        printf("    有效位数 ENOB: %.1f 位（理论 12 位）\n", enob);
        printf("    有效分辨率   : %.1f mV（理论 0.8 mV）\n",
               VREF_MV / powf(2.0f, enob));
    }
    printf("═════════════════════════════════════════\n");
    printf("\n  注： Espressif 官方给了基于 eFuse 的校准方案 (esp_adc/adc_cali.h)，\n");
    printf("       下一步可以拿它跟这次手工标定做对比。\n\n");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
