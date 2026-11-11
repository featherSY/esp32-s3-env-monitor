/*
 * Stage 3：WiFi STA + HTTP server + 实时温度曲线
 * -----------------------------------------------------------------------------
 * 三个里程碑一次到位：
 *   ① WiFi 连上并打印 IP（断线自动重连，带退避）
 *   ② GET /data 返回 JSON：{"t":ms,"temp":滤波值,"raw":原始值}
 *   ③ GET /     返回单页 HTML，手机浏览器里看实时曲线
 *
 * 设计要点（面试可讲）：
 *   - 采样在独立 FreeRTOS 任务里跑，HTTP handler 只读共享变量，绝不做耗时操作
 *   - 共享结构体加互斥量（float 单读单写原子，但结构体整体读不是）
 *   - 滤波窗口未填满时按实际样本数求平均，避免开机前 8 个点被 0 拉低
 *   - WiFi 凭据放 wifi_creds.h，该文件不进仓库（.gitignore 已排除）
 *
 * 硬件：ESP32-S3-WROOM-1-N16R8，ESP-IDF v5.4.4
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/temperature_sensor.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "wifi_creds.h" /* WIFI_SSID / WIFI_PASS —— 不纳入仓库 */

#define SAMPLE_MS 500
#define FILTER_N  8

static const char *TAG = "env";

/* ---------------- 共享数据 ---------------- */

typedef struct {
    float   raw;    /* TSENS 原始读数（1°C 分辨率） */
    float   filt;   /* 滑动平均后的值 */
    int64_t t_ms;   /* 采样时间戳（ms） */
} sample_t;

static sample_t               s_sample;
static SemaphoreHandle_t      s_lock;
static temperature_sensor_handle_t s_tsens;

/* ---------------- 采样任务 ---------------- */

static void sensor_task(void *arg)
{
    float win[FILTER_N] = {0};
    int   idx = 0;
    int   filled = 0;

    while (1) {
        float raw = 0.0f;
        temperature_sensor_get_celsius(s_tsens, &raw);

        win[idx] = raw;
        idx = (idx + 1) % FILTER_N;
        if (filled < FILTER_N) {
            filled++;
        }

        /* 窗口未填满时只对有效样本求平均，避免开机时被 0 拉低 */
        float sum = 0.0f;
        for (int i = 0; i < filled; i++) {
            sum += win[i];
        }
        float filt = sum / (float)filled;

        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_sample.raw  = raw;
        s_sample.filt = filt;
        s_sample.t_ms = esp_timer_get_time() / 1000;
        xSemaphoreGive(s_lock);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
    }
}

/* ---------------- WiFi STA + 自动重连 ---------------- */

static int s_retry = 0;

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        /* 注意：本函数跑在事件任务上下文，不能长时间阻塞。
         * 这里直接重连（ESP-IDF 内部会自行退避）；要做指数退避应当交给 esp_timer。 */
        s_retry++;
        ESP_LOGW(TAG, "WiFi 断开，第 %d 次重连（reason=%d）", s_retry,
                 ((wifi_event_sta_disconnected_t *)data)->reason);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        s_retry = 0;
        ESP_LOGI(TAG, "got ip:" IPSTR "  ← 浏览器打开 http://" IPSTR, IP2STR(&e->ip_info.ip),
                 IP2STR(&e->ip_info.ip));
    }
}

static void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL));

    wifi_config_t wc = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi 启动，SSID=%s", WIFI_SSID);
}

/* ---------------- HTTP server ---------------- */

static esp_err_t index_get(httpd_req_t *req)
{
    extern const char *INDEX_HTML;
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t data_get(httpd_req_t *req)
{
    /* 只做「加锁拷一份 → 解锁 → 组包 → 发送」，handler 里没有任何耗时操作 */
    sample_t s;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s = s_sample;
    xSemaphoreGive(s_lock);

    char buf[128];
    int  n = snprintf(buf, sizeof(buf), "{\"t\":%lld,\"temp\":%.2f,\"raw\":%.2f}\n",
                      (long long)s.t_ms, s.filt, s.raw);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, buf, n);
}

static void http_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = 80;
    cfg.max_uri_handlers = 8;
    cfg.stack_size       = 8192;   /* handler 里用了 snprintf，留足栈 */
    cfg.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &cfg));

    httpd_uri_t uris[] = {
        {.uri = "/",     .method = HTTP_GET, .handler = index_get},
        {.uri = "/data", .method = HTTP_GET, .handler = data_get},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uris[i]));
    }
    ESP_LOGI(TAG, "HTTP server 已启动，端口 80");
}

/* ---------------- 前端页面（内嵌，免文件系统） ---------------- */

const char *INDEX_HTML =
    "<!doctype html><html lang=\"zh\"><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>ESP32-S3 环境监测</title><style>"
    "body{margin:0;background:#111;color:#eee;font-family:system-ui,sans-serif}"
    "#box{padding:16px;max-width:900px;margin:0 auto}"
    "#v{font-size:52px;font-weight:700;color:#e74c3c;line-height:1}"
    ".u{font-size:22px;color:#888}.m{color:#777;font-size:13px;margin:6px 0 14px}"
    "canvas{width:100%;height:300px;background:#181818;border-radius:10px;display:block}"
    "</style></head><body><div id=\"box\">"
    "<div><span id=\"v\">--</span><span class=\"u\"> °C</span></div>"
    "<div class=\"m\">芯片内置 TSENS · 滑动平均 N=8 · 每 0.5 s 更新</div>"
    "<canvas id=\"c\"></canvas></div><script>"
    "const N=240,buf=[];const c=document.getElementById('c'),x=c.getContext('2d');"
    "function rs(){c.width=c.clientWidth*devicePixelRatio;"
    "c.height=c.clientHeight*devicePixelRatio;x.lineWidth=2*devicePixelRatio;}"
    "rs();addEventListener('resize',function(){rs();draw();});"
    "function draw(){const w=c.width,h=c.height;x.clearRect(0,0,w,h);"
    "if(buf.length<2)return;"
    "let lo=Math.min.apply(null,buf),hi=Math.max.apply(null,buf);"
    "if(hi-lo<1){hi=lo+1;}lo-=0.3;hi+=0.3;"
    "x.strokeStyle='#e74c3c';x.beginPath();"
    "for(let i=0;i<buf.length;i++){const px=i/(N-1)*w,py=h-(buf[i]-lo)/(hi-lo)*h;"
    "if(i===0){x.moveTo(px,py);}else{x.lineTo(px,py);}}x.stroke();"
    "x.fillStyle='#555';x.font=(12*devicePixelRatio)+'px system-ui';"
    "x.fillText(hi.toFixed(1)+' °C',8,18*devicePixelRatio);"
    "x.fillText(lo.toFixed(1)+' °C',8,h-8);}"
    "async function tick(){try{const r=await fetch('/data',{cache:'no-store'});"
    "const d=await r.json();document.getElementById('v').textContent=d.temp.toFixed(2);"
    "buf.push(d.temp);if(buf.length>N)buf.shift();draw();}catch(e){}"
    "setTimeout(tick,500);}"
    "tick();</script></body></html>";

/* ---------------- 入口 ---------------- */

void app_main(void)
{
    /* NVS 是 WiFi 驱动存取校准数据的前提 */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    temperature_sensor_config_t ts_cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 60);
    ESP_ERROR_CHECK(temperature_sensor_install(&ts_cfg, &s_tsens));
    ESP_ERROR_CHECK(temperature_sensor_enable(s_tsens));

    s_lock = xSemaphoreCreateMutex();
    configASSERT(s_lock);

    xTaskCreate(sensor_task, "sensor", 3072, NULL, 5, NULL);

    wifi_start();
    http_start();

    ESP_LOGI(TAG, "就绪：浏览器打开 http://<板子IP>/  数据接口 /data");
}
