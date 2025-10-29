// test_esp32cam.c
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "esp_http_server.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"     // เช็ค SPIRAM จาก heap

#include "lwip/inet.h"         // inet/aton helpers
#include "lwip/ip4_addr.h"     // ip4addr_aton(), ip4_addr_get_u32()

// ====== USER CONFIG ======
// Wi-Fi
#define WIFI_SSID            "Adil"
#define WIFI_PASS            "0622198015"
#define WIFI_FIXED_CHANNEL   6          // ล็อกช่องที่ AP ใช้อยู่ (ถ้าจริงไม่ใช่ ch6 ให้แก้เลขนี้)

// Static IP (ช่วยลดโอกาส DHCP ช้า → reason 204)
#define USE_STATIC_IP        1          // 0=DHCP, 1=Static IP
#define STATIC_IP_ADDR       "172.20.10.10"
#define STATIC_NETMASK       "255.255.255.240"
#define STATIC_GW_ADDR       "172.20.10.1"

// Camera
#define CAM_FLASH_GPIO       4          // Flash LED (AI-Thinker = GPIO4)
#define CAM_XCLK_MHZ         20
#define CAM_FRAME_SIZE       FRAMESIZE_VGA   // เริ่มแบบนิ่ง ๆ ก่อน
#define CAM_JPEG_QUALITY     15
#define CAM_FB_COUNT         2               // ถ้าไม่มี PSRAM จะโดนลดเหลือ 1 ด้านล่าง
// ==========================

// ====== Pins for AI-Thinker ESP32-CAM (OV2640) ======
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
// ================================================

static const char *TAG = "ESP32-CAM";

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// ---------- helper: reason to string ----------
static const char* wifi_reason_str(uint8_t r)
{
    switch (r) {
        case 1:  return "UNSPECIFIED";
        case 2:  return "AUTH_EXPIRE";
        case 3:  return "AUTH_LEAVE";
        case 4:  return "ASSOC_EXPIRE";
        case 5:  return "ASSOC_TOOMANY";
        case 6:  return "NOT_AUTHED";
        case 7:  return "NOT_ASSOCED";
        case 8:  return "ASSOC_LEAVE";
        case 9:  return "ASSOC_NOT_AUTHED";
        case 15: return "4WAY_HANDSHAKE_TIMEOUT";
        case 17: return "GROUP_KEY_UPDATE_TIMEOUT";
        case 200:return "BEACON_TIMEOUT";
        case 201:return "NO_AP_FOUND";
        case 202:return "AUTH_FAIL";
        case 203:return "ASSOC_FAIL/HANDSHAKE";
        case 204:return "CONNECTION_FAIL";
        case 205:return "AP_NOT_FOUND";
        default: return "UNKNOWN";
    }
}

// helper: แปลง "x.x.x.x" → esp_ip4_addr_t (สำหรับ esp_netif_ip_info_t)
static inline void str_to_esp_ip4(const char *s, esp_ip4_addr_t *out)
{
    ip4_addr_t tmp;
    ip4addr_aton(s, &tmp);                 // เขียนค่าใส่ tmp (ชนิดของ lwIP)
    out->addr = ip4_addr_get_u32(&tmp);    // คัดลอกเป็น u32 ไปที่ esp_ip4_addr_t
}

// ---------- Wi-Fi ----------
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "STA started");
#if USE_STATIC_IP
        // ปิด power save เพื่อลดปัญหา handshake/DHCP ช้า
        esp_wifi_set_ps(WIFI_PS_NONE);
#endif
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *e = (const wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Wi-Fi disconnected, reason=%d (%s)", e ? e->reason : 0xFF,
                 e ? wifi_reason_str(e->reason) : "NO_DATA");
        vTaskDelay(pdMS_TO_TICKS(500)); // เว้นจังหวะกัน log วิ่งถี่
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* e = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_sta_init_start(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    // Country (ให้ตรงตามโซน/ช่อง)
    wifi_country_t cc = {
        .cc = "TH",
        .schan = 1,
        .nchan = 13,
        .policy = WIFI_COUNTRY_POLICY_AUTO
    };
    esp_wifi_set_country(&cc);

    // เริ่ม STA
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

#if USE_STATIC_IP
    // ตั้ง Static IP ก่อน start Wi-Fi
    esp_netif_dhcpc_stop(sta);
    esp_netif_ip_info_t ip;
    memset(&ip, 0, sizeof(ip));
    str_to_esp_ip4(STATIC_IP_ADDR, &ip.ip);
    str_to_esp_ip4(STATIC_NETMASK, &ip.netmask);
    str_to_esp_ip4(STATIC_GW_ADDR, &ip.gw);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(sta, &ip));
#endif

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi connecting to %s (fixed channel)", WIFI_SSID);

    wifi_config_t wifi_config = { 0 };
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg = (wifi_pmf_config_t){ .capable = true, .required = false };
    wifi_config.sta.channel = WIFI_FIXED_CHANNEL;      // ล็อกช่องเพื่อต่อเร็วขึ้น
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.listen_interval = 3;

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_wifi_connect();

    // รอจนได้ IP (Static IP ก็จะเข้า IP_EVENT_STA_GOT_IP เร็ว)
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

// ---------- Camera ----------
static esp_err_t camera_init(void)
{
    camera_config_t config = { 0 };
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    // ใช้ชื่อใหม่ตามไลบรารี (เลี่ยงชื่อ legacy)
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = CAM_XCLK_MHZ * 1000000;
    config.pixel_format = PIXFORMAT_JPEG;

    bool has_spiram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0;

    if (has_spiram) {
        config.frame_size   = CAM_FRAME_SIZE;
        config.jpeg_quality = CAM_JPEG_QUALITY;
        config.fb_count     = CAM_FB_COUNT;
        config.fb_location  = CAMERA_FB_IN_PSRAM;
    } else {
        config.frame_size   = CAM_FRAME_SIZE;    // VGA
        config.jpeg_quality = CAM_JPEG_QUALITY;  // 15
        config.fb_count     = 1;
        config.fb_location  = CAMERA_FB_IN_DRAM;
    }

    // ตั้งค่าแฟลช GPIO
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CAM_FLASH_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
    gpio_set_level(CAM_FLASH_GPIO, 0);

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with 0x%x", err);
    } else {
        ESP_LOGI(TAG, "Camera init ok");
    }
    return err;
}

// ---------- HTTP ----------
static const char *INDEX_HTML =
"<!DOCTYPE html><html><head><meta charset='utf-8'/>"
"<title>ESP32-CAM</title>"
"<style>body{font-family:sans-serif;margin:24px}a,button{padding:8px 14px;display:inline-block}</style>"
"</head><body>"
"<h2>ESP32-CAM Live</h2>"
"<p><img src='/stream' width='480' /></p>"
"<h3>Manual capture</h3>"
"<p><a href='/jpg'>Take Photo (/jpg)</a></p>"
"<p><a href='/jpg?flash=on'>/jpg?flash=on</a> | <a href='/jpg?flash=off'>/jpg?flash=off</a></p>"
"</body></html>";

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t jpg_get_handler(httpd_req_t *req)
{
    // parse query ?flash=on/off
    char buf[32];
    bool use_flash = false;
    size_t qlen = httpd_req_get_url_query_len(req) + 1;
    if (qlen > 1 && qlen < sizeof(buf)) {
        if (httpd_req_get_url_query_str(req, buf, qlen) == ESP_OK) {
            char val[8];
            if (httpd_query_key_value(buf, "flash", val, sizeof(val)) == ESP_OK) {
                if (!strcmp(val, "on")) use_flash = true;
            }
        }
    }

    if (use_flash) gpio_set_level(CAM_FLASH_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(use_flash ? 120 : 5)); // รอแสงนิ่งนิดนึง

    camera_fb_t *fb = esp_camera_fb_get();
    if (use_flash) gpio_set_level(CAM_FLASH_GPIO, 0);

    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

// ---- Live MJPEG stream (/stream) ----
static esp_err_t stream_handler(httpd_req_t *req)
{
    static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
    static const char* _STREAM_BOUNDARY     = "\r\n--frame\r\n";
    static const char* _STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

    httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);

    camera_fb_t *fb = NULL;
    char part_buf[64];
    esp_err_t res = ESP_OK;

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            res = ESP_FAIL;
            break;
        }

        if (httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY)) != ESP_OK) {
            esp_camera_fb_return(fb);
            res = ESP_FAIL;
            break;
        }

        int hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, fb->len);
        if (httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK) {
            esp_camera_fb_return(fb);
            res = ESP_FAIL;
            break;
        }

        if (httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len) != ESP_OK) {
            esp_camera_fb_return(fb);
            res = ESP_FAIL;
            break;
        }

        if (httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) {
            esp_camera_fb_return(fb);
            res = ESP_FAIL;
            break;
        }

        esp_camera_fb_return(fb);

        // จำกัดเฟรมเรต ~8–10 fps (ปรับได้)
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    httpd_resp_send_chunk(req, NULL, 0);
    return res;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_uri_t jpg = {
            .uri       = "/jpg",
            .method    = HTTP_GET,
            .handler   = jpg_get_handler,
            .user_ctx  = NULL
        };
        httpd_uri_t stream = {
            .uri       = "/stream",
            .method    = HTTP_GET,
            .handler   = stream_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &jpg);
        httpd_register_uri_handler(server, &stream);
        ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server");
    }
    return server;
}

void app_main(void)
{
    // NVS (จำเป็นสำหรับ Wi-Fi)
    ESP_ERROR_CHECK(nvs_flash_init());

    // กล้อง
    ESP_ERROR_CHECK(camera_init());

    // Wi-Fi
    wifi_sta_init_start();

    // HTTP
    start_webserver();

    ESP_LOGI(TAG, "Ready. Open http://<ESP32-CAM-IP>/  (Live at /stream)");
}
