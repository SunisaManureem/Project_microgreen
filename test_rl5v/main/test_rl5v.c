#include <stdio.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_sntp.h"

#include "driver/gpio.h"

/* ======================== USER CONFIG ========================
 * Wi-Fi
 */
#define WIFI_SSID           "Areena"
#define WIFI_PASS           "1234567890"

/* Grow Light (Active-Low) — ตามโค้ด SNTP/NVS เดิม */
#define GROW_RELAY_GPIO         GPIO_NUM_33
#define GROW_ACTIVE_LEVEL       0   // LOW = ON
#define GROW_INACTIVE_LEVEL     1   // HIGH = OFF

/* Relay 2CH (Active-High) — ตามโค้ดทดสอบ 2 ช่องเดิม */
#define RELAY_PUMP_GPIO         GPIO_NUM_26   // ปั๊ม
#define RELAY_FAN_GPIO          GPIO_NUM_27   // พัดลม

/* ปุ่มรีเซ็ตรอบปลูก (ออปชัน) */
#define RESET_BTN_PIN           GPIO_NUM_0
#define RESET_HOLD_MS           3000

/* โหมดทดสอบเป็น "นาที" สำหรับไฟปลูก */
#define TOTAL_MINUTES           7   // รวมรอบ 7 นาที
#define OFF_MINUTES             3   // นาที 1-3 OFF → นาที 4-7 ON

/* ============================================================ */

static const char *TAG_GROW = "GROW_TEST_MIN";
static const char *TAG_2CH  = "RELAY2CH_AH";

/* -------- Helpers: Grow Light (Active-Low) -------- */
static inline void grow_on(void)  { gpio_set_level(GROW_RELAY_GPIO, GROW_ACTIVE_LEVEL); }
static inline void grow_off(void) { gpio_set_level(GROW_RELAY_GPIO, GROW_INACTIVE_LEVEL); }

/* -------- Helpers: 2CH (Active-High) -------- */
static inline void relay2ch_write(gpio_num_t pin, bool turn_on) {
    gpio_set_level(pin, turn_on ? 1 : 0);     // Active-High
}

/* ================= Wi-Fi ================= */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGW(TAG_GROW, "WiFi disconnected, retry...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG_GROW, "Got IP");
    }
}

static void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id, instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid)-1);
    strncpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password)-1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_GROW, "Connecting to Wi-Fi: %s ...", WIFI_SSID);
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG_GROW, "Wi-Fi connected");
}

/* ================= SNTP / Time ================= */
static void time_sync_wait(void) {
    if (!esp_sntp_enabled()) {
        esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, "pool.ntp.org");
        esp_sntp_init();
    }
    time_t now = 0;
    struct tm ti = {0};
    for (int i = 0; i < 30; i++) { // ~30s
        time(&now);
        localtime_r(&now, &ti);
        if (ti.tm_year >= (2016 - 1900)) {
            ESP_LOGI(TAG_GROW, "Time synced");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGW(TAG_GROW, "Time not synced (continue anyway)");
}

/* ================= NVS ================= */
static esp_err_t nvs_get_start_ts(time_t *out) {
    nvs_handle_t nvh;
    esp_err_t err = nvs_open("grow", NVS_READONLY, &nvh);
    if (err != ESP_OK) return err;
    int64_t ts64 = 0;
    err = nvs_get_i64(nvh, "start_ts", &ts64);
    nvs_close(nvh);
    if (err == ESP_OK) *out = (time_t)ts64;
    return err;
}

static esp_err_t nvs_set_start_ts(time_t ts) {
    nvs_handle_t nvh;
    ESP_ERROR_CHECK(nvs_open("grow", NVS_READWRITE, &nvh));
    esp_err_t err = nvs_set_i64(nvh, "start_ts", (int64_t)ts);
    if (err == ESP_OK) err = nvs_commit(nvh);
    nvs_close(nvh);
    return err;
}

static void nvs_clear_start_ts(void) {
    nvs_handle_t nvh;
    if (nvs_open("grow", NVS_READWRITE, &nvh) == ESP_OK) {
        nvs_erase_key(nvh, "start_ts");
        nvs_commit(nvh);
        nvs_close(nvh);
    }
}

/* ================= Button ================= */
static void button_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << RESET_BTN_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
}

static bool button_held_ms(uint32_t hold_ms) {
    const TickType_t step = pdMS_TO_TICKS(10);
    TickType_t ticks = 0;
    while (gpio_get_level(RESET_BTN_PIN) == 0) { // active-low button
        vTaskDelay(step);
        ticks += step;
        if (ticks >= pdMS_TO_TICKS(hold_ms)) return true;
    }
    return false;
}

/* ================= Grow-Light Minute Logic ================= */
static int minutes_since(time_t start_ts, time_t now) {
    if (now <= start_ts) return 0;
    double diff = difftime(now, start_ts);
    return (int)(diff / 60.0);
}

static void apply_light_policy_minute(int m) {
    if (m < OFF_MINUTES) {
        grow_off();
        ESP_LOGI(TAG_GROW, "Minute %d: LIGHT = OFF (phase 1–3 off)", m + 1);
    } else if (m < TOTAL_MINUTES) {
        grow_on();
        ESP_LOGI(TAG_GROW, "Minute %d: LIGHT = ON  (phase 4–7 on)", m + 1);
    } else {
        grow_off();
        ESP_LOGI(TAG_GROW, "Minute %d+: LIGHT = OFF (CYCLE DONE)", m + 1);
    }
}

/* ================= Tasks ================= */
// Task ควบคุมไฟปลูกตามนาที + วนรอบอัตโนมัติ
static void grow_cycle_task(void *arg) {
    // NVS init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // GPIO (grow)
    gpio_reset_pin(GROW_RELAY_GPIO);
    gpio_set_direction(GROW_RELAY_GPIO, GPIO_MODE_OUTPUT);
    grow_off();

    // Button
    button_init();

    // Wi-Fi + time
    wifi_init_sta();
    time_sync_wait();

    ESP_LOGI(TAG_GROW, "Hold RESET button %d ms to start a NEW 7-minute cycle.", RESET_HOLD_MS);
    vTaskDelay(pdMS_TO_TICKS(500));
    if (gpio_get_level(RESET_BTN_PIN) == 0 && button_held_ms(RESET_HOLD_MS)) {
        nvs_clear_start_ts();
        ESP_LOGW(TAG_GROW, "Reset requested: cleared previous start_ts");
    }

    time_t start_ts = 0;
    if (nvs_get_start_ts(&start_ts) != ESP_OK || start_ts == 0) {
        time(&start_ts);
        nvs_set_start_ts(start_ts);
        ESP_LOGI(TAG_GROW, "Set new start_ts = %ld", (long)start_ts);
    } else {
        ESP_LOGI(TAG_GROW, "Loaded start_ts  = %ld", (long)start_ts);
    }

    while (1) {
        time_t now;
        time(&now);
        int m = minutes_since(start_ts, now);
        apply_light_policy_minute(m);

        if (m >= TOTAL_MINUTES) {
            nvs_set_start_ts(now);
            start_ts = now;
            ESP_LOGI(TAG_GROW, "Auto-start NEW 7-minute cycle");
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // check ทุก 5 วินาที
    }
}

// Task ทดสอบรีเลย์ 2 ช่อง (Active-High) สลับทุก 5 วิ
static void relay2ch_test_task(void *arg) {
    // ตั้งขาเป็นเอาต์พุต
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<RELAY_PUMP_GPIO) | (1ULL<<RELAY_FAN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    // เริ่ม OFF ทั้งคู่
    relay2ch_write(RELAY_PUMP_GPIO, false);
    relay2ch_write(RELAY_FAN_GPIO,  false);
    ESP_LOGI(TAG_2CH, "Relay test ready (Active HIGH). Pump/Fan OFF.");

    while (1) {
        ESP_LOGI(TAG_2CH, "Pump ON, Fan ON");
        relay2ch_write(RELAY_PUMP_GPIO, true);   // HIGH = ON
        relay2ch_write(RELAY_FAN_GPIO,  true);
        vTaskDelay(pdMS_TO_TICKS(5000));

        ESP_LOGI(TAG_2CH, "Pump OFF, Fan OFF");
        relay2ch_write(RELAY_PUMP_GPIO, false);  // LOW = OFF
        relay2ch_write(RELAY_FAN_GPIO,  false);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* ================= app_main ================= */
void app_main(void)
{
    // สร้าง 2 งานให้ทำคู่กัน:
    // 1) grow_cycle_task  : ไฟปลูกตามนาที + Wi-Fi/SNTP/NVS
    // 2) relay2ch_test_task: ทดสอบรีเลย์ 2 ช่อง สลับทุก 5 วิ
    xTaskCreatePinnedToCore(grow_cycle_task,   "grow_cycle_task",   6144, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(relay2ch_test_task,"relay2ch_test_task",4096, NULL, 4, NULL, 1);
}
