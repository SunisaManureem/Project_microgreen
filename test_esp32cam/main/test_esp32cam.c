// ============================================================
// ESP32-CAM with Wi-Fi Provisioning (BLE)
// ============================================================
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"

// ====== Wi-Fi Provisioning Headers ======
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_ble.h"

static const char *TAG = "ESP32-CAM-PROV";

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

#define CAM_FLASH_GPIO     4
#define CAM_XCLK_MHZ      20

// ====== Provisioning Config ======
// ชื่อ Service ที่จะขึ้นในแอป และรหัสยืนยัน
#define PROV_SERVICE_NAME "ESP32CAM-Prov"
#define PROV_POP          "123456" 

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

/* ----------------------------------------------------------------
   Event Handler: จัดการ Wi-Fi, IP และ Provisioning Events
   ---------------------------------------------------------------- */
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    // --- Wi-Fi / IP Events ---
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Disconnected. Retrying connect...");
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Connected! Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    // --- Provisioning Events ---
    else if (event_base == WIFI_PROV_EVENT) {
        switch (event_id) {
            case WIFI_PROV_START:
                ESP_LOGI(TAG, "Provisioning started");
                break;
            case WIFI_PROV_CRED_RECV:
                ESP_LOGI(TAG, "Received Wi-Fi credentials");
                break;
            case WIFI_PROV_CRED_FAIL:
                ESP_LOGW(TAG, "Provisioning failed!");
                break;
            case WIFI_PROV_CRED_SUCCESS:
                ESP_LOGI(TAG, "Provisioning successful");
                break;
            case WIFI_PROV_END:
                ESP_LOGI(TAG, "Provisioning end");
                wifi_prov_mgr_deinit();
                break;
            default:
                break;
        }
    }
}

/* ----------------------------------------------------------------
   Wi-Fi Init with Provisioning Logic
   ---------------------------------------------------------------- */
static void wifi_init_prov(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register Event Handlers
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH)); // เก็บค่าใน NVS

    // Config Provisioning Manager
    wifi_prov_mgr_config_t prov_cfg = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM
    };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(prov_cfg));

    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (!provisioned) {
        ESP_LOGI(TAG, "Starting BLE provisioning (Service: %s, PoP: %s)", PROV_SERVICE_NAME, PROV_POP);
        
        // Start Provisioning
        wifi_prov_security_t security = WIFI_PROV_SECURITY_1;
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(security, PROV_POP, PROV_SERVICE_NAME, NULL));
    } else {
        ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi...");
        wifi_prov_mgr_deinit();
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_ERROR_CHECK(esp_wifi_connect());
    }

    // Wait for IP
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
}

// ---------- Camera Init (เหมือนเดิม) ----------
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
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = CAM_XCLK_MHZ * 1000000;
    config.pixel_format = PIXFORMAT_JPEG;

    if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0) {
        config.frame_size   = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count     = 2;
        config.fb_location  = CAMERA_FB_IN_PSRAM;
    } else {
        config.frame_size   = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count     = 1;
        config.fb_location  = CAMERA_FB_IN_DRAM;
    }

    // Flash Pin Setup
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CAM_FLASH_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
    gpio_set_level(CAM_FLASH_GPIO, 0);

    return esp_camera_init(&config);
}

// ---------- HTTP Server Handlers (เหมือนเดิม) ----------
static const char *INDEX_HTML =
"<!DOCTYPE html><html><head><meta charset='utf-8'/>"
"<title>ESP32-CAM</title>"
"<style>body{font-family:sans-serif;margin:24px} button{padding:10px;margin:5px} img{margin-top:10px;border:1px solid #ccc}</style>"
"</head><body>"
"<h2>ESP32-CAM Provisioned</h2>"
"<button onclick=\"takePhoto(false)\">📸 Take Photo</button>"
"<button onclick=\"takePhoto(true)\">💡 Flash Photo</button><br/>"
"<img id='photo' width='480'/>"
"<script>"
"function takePhoto(flash){"
"  let url = '/jpg';"
"  if(flash) url += '?flash=on';"
"  document.getElementById('photo').src = url + '&t=' + Date.now();"
"}"
"</script>"
"</body></html>";

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t jpg_get_handler(httpd_req_t *req)
{
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
    vTaskDelay(pdMS_TO_TICKS(use_flash ? 120 : 5));

    camera_fb_t *fb = esp_camera_fb_get();
    if (use_flash) gpio_set_level(CAM_FLASH_GPIO, 0);

    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=frame";
    static const char* _STREAM_BOUNDARY     = "\r\n--frame\r\n";
    static const char* _STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);

    camera_fb_t *fb = NULL;
    char part_buf[64];
    esp_err_t res = ESP_OK;

    while (true) {
        fb = esp_camera_fb_get();
        if (!fb) { res = ESP_FAIL; break; }

        if (httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY)) != ESP_OK) {
            esp_camera_fb_return(fb); res = ESP_FAIL; break;
        }

        int hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, fb->len);
        if (httpd_resp_send_chunk(req, part_buf, hlen) != ESP_OK) {
            esp_camera_fb_return(fb); res = ESP_FAIL; break;
        }

        if (httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len) != ESP_OK) {
            esp_camera_fb_return(fb); res = ESP_FAIL; break;
        }

        if (httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) {
            esp_camera_fb_return(fb); res = ESP_FAIL; break;
        }

        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(100)); // ~10 FPS
    }
    return res;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_uri_t jpg = { .uri = "/jpg", .method = HTTP_GET, .handler = jpg_get_handler };
        httpd_uri_t stream = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler };
        
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &jpg);
        httpd_register_uri_handler(server, &stream);
    }
    return server;
}

// Function ล้าง Wi-Fi (ถ้าต้องการ Reset เพื่อ Provision ใหม่)
// ESP32-CAM มักไม่มีปุ่มอื่นนอกจาก Reset และ GPIO0 (Boot)
// ถ้ากด GPIO0 ค้างไว้ 5 วินาทีหลังเปิดเครื่อง ให้ล้างค่า
void check_reset_provisioning() {
    // GPIO 0 มักเป็นปุ่ม Boot บนบอร์ด (ถ้าไม่มีต้องหาทาง jump ลง GND เอง)
    gpio_reset_pin(GPIO_NUM_0);
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_0, GPIO_PULLUP_ONLY);

    if (gpio_get_level(GPIO_NUM_0) == 0) {
        ESP_LOGW(TAG, "GPIO0 held low... Waiting to see if factory reset is requested");
        vTaskDelay(pdMS_TO_TICKS(3000)); // รอ 3 วินาที
        if (gpio_get_level(GPIO_NUM_0) == 0) {
             ESP_LOGW(TAG, "Factory Reset Triggered! Erasing NVS...");
             nvs_flash_erase();
             nvs_flash_init();
             esp_restart();
        }
    }
}

void app_main(void)
{
    // NVS Init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // เช็คว่าผู้ใช้กดปุ่ม Reset Wi-Fi ไหม
    check_reset_provisioning();

    // Camera Init
    if(camera_init() != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }

    // Wi-Fi Init (BLE Provisioning)
    // แอปจะเห็นชื่อ: ESP32CAM-Prov
    // PoP: 123456
    wifi_init_prov();

    // HTTP Server
    start_webserver();

    ESP_LOGI(TAG, "System Ready.");
}