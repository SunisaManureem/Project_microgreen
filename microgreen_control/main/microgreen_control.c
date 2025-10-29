// ===== microgreen_control.c (ready-to-drop) =====
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_sntp.h"

#include "driver/gpio.h"
#include "driver/adc.h"      // legacy ADC oneshot; OK for simple use
#include "esp_rom_sys.h"

/* ======================== USER CONFIG ======================== */
/* Wi-Fi */
#define WIFI_SSID           "Areena"
#define WIFI_PASS           "1234567890"

/* Grow Light (Active-Low) */
#define GROW_RELAY_GPIO         GPIO_NUM_33
#define GROW_ACTIVE_LEVEL       0   // LOW = ON
#define GROW_INACTIVE_LEVEL     1   // HIGH = OFF

/* Relay 2CH (Active-High) */
#define RELAY_PUMP_GPIO         GPIO_NUM_27   // ปั๊มน้ำ (Active-HIGH)
#define RELAY_FAN_GPIO          GPIO_NUM_26   // พัดลม (Active-HIGH)

/* ปุ่มรีเซ็ตรอบปลูก (กดค้างเพื่อล้าง start_ts) */
#define RESET_BTN_PIN           GPIO_NUM_0
#define RESET_HOLD_MS           3000

/* โหมดทดสอบเป็น "นาที" สำหรับไฟปลูก */
#define TOTAL_MINUTES           7   // รวมรอบ 7 นาที
#define OFF_MINUTES             3   // นาที 1-3 OFF → นาที 4-7 ON

/* ---------- Soil sensor (ADC) ---------- */
#define SOIL_ADC_CH             ADC1_CHANNEL_6   // GPIO34
#define SOIL_ADC_ATTEN          ADC_ATTEN_DB_12  // ลด warning deprecate
#define SOIL_SAMPLES_MEDIAN     15

// คาลิเบรตกับชุดจริง
static int rawDry = 3300;           // ดินแห้ง
static int rawWet = 1500;           // ดินเปียก

// ฮิสเทอรีซิส % ความชื้นดิน
static int SOIL_TH_LOW  = 35;       // ≤35% เปิดปั๊ม
static int SOIL_TH_HIGH = 60;       // ≥60% ปิดปั๊ม

// เวลาขั้นต่ำ/เซฟตี้ ปั๊มน้ำ
static const uint32_t PUMP_MIN_ON_MS      = 15*1000;
static const uint32_t PUMP_MIN_OFF_MS     = 10*1000;
static const uint32_t PUMP_MAX_CONT_ON_MS = 10*60*1000;

/* ---------- DHT22 → คุมพัดลม ---------- */
#define DHT_PIN                 GPIO_NUM_4
static const uint32_t DHT_INTERVAL_MS     = 2000;
static const float    FAN_ON_TEMP_C       = 30.0f;  // ≥30°C เปิด
static const float    FAN_OFF_TEMP_C      = 28.0f;  // ≤28°C ปิด
static const uint32_t FAN_MIN_ON_MS       = 10*1000;
static const uint32_t FAN_MIN_OFF_MS      = 10*1000;

/* ======================== TAGs ======================== */
static const char *TAG_GROW = "GROW";
static const char *TAG_NET  = "NET";
static const char *TAG_SOIL = "SOIL_PUMP";
static const char *TAG_DHT  = "DHT_FAN";

/* ================= Helpers: GPIO/Relay ================= */
static inline void grow_on(void)  { gpio_set_level(GROW_RELAY_GPIO, GROW_ACTIVE_LEVEL); }
static inline void grow_off(void) { gpio_set_level(GROW_RELAY_GPIO, GROW_INACTIVE_LEVEL); }
static inline void relay2ch_write(gpio_num_t pin, bool turn_on) { gpio_set_level(pin, turn_on ? 1 : 0); }

/* ================= Wi-Fi ================= */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t*)event_data;
        ESP_LOGW(TAG_NET, "WiFi disconnected, reason=%d", d ? d->reason : -1);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG_NET, "Got IP");
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

    // ปิด Power Save กันสัญญาณดรอปง่าย
    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_LOGI(TAG_NET, "Connecting to Wi-Fi: %s ...", WIFI_SSID);

    // รอ Wi-Fi สูงสุด 10 วินาที แล้วไปต่อแม้ยังไม่เชื่อม
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                                           false, true, pdMS_TO_TICKS(10000));
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_NET, "Wi-Fi connected");
    } else {
        ESP_LOGW(TAG_NET, "Wi-Fi not ready after 10s, continue offline");
    }
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
    for (int i = 0; i < 30; i++) {
        time(&now);
        localtime_r(&now, &ti);
        if (ti.tm_year >= (2016 - 1900)) {
            ESP_LOGI(TAG_NET, "Time synced");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGW(TAG_NET, "Time not synced (continue anyway)");
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
    while (gpio_get_level(RESET_BTN_PIN) == 0) {
        vTaskDelay(step);
        ticks += step;
        if (ticks >= pdMS_TO_TICKS(hold_ms)) return true;
    }
    return false;
}

/* ================= Grow-Light (minute policy) ================= */
static int minutes_since(time_t start_ts, time_t now) {
    if (now <= start_ts) return 0;
    double diff = difftime(now, start_ts);
    return (int)(diff / 60.0);
}

static void apply_light_policy_minute(int m) {
    if (m < OFF_MINUTES) {
        grow_off();
        ESP_LOGI(TAG_GROW, "Minute %d: LIGHT = OFF", m + 1);
    } else if (m < TOTAL_MINUTES) {
        grow_on();
        ESP_LOGI(TAG_GROW, "Minute %d: LIGHT = ON", m + 1);
    } else {
        grow_off();
        ESP_LOGI(TAG_GROW, "Minute %d+: LIGHT = OFF (CYCLE DONE)", m + 1);
    }
}

/* ================= Utilities ================= */
static inline uint32_t ms_now(void) { return (uint32_t)(esp_timer_get_time()/1000ULL); }

/* ================= Soil Sensor (ADC) ================= */
static int soil_read_raw_median(void)
{
    int buf[31]; int N = SOIL_SAMPLES_MEDIAN; if (N > 31) N = 31;
    for (int i = 0; i < N; ++i) {
        buf[i] = adc1_get_raw(SOIL_ADC_CH);
        esp_rom_delay_us(4000);
    }
    for (int i = 1; i < N; ++i) {
        int key = buf[i], j = i - 1;
        while (j >= 0 && buf[j] > key) { buf[j+1] = buf[j]; j--; }
        buf[j+1] = key;
    }
    return buf[N/2];
}

static int soil_raw_to_pct(int raw)
{
    long num = (long)(raw - rawDry) * 100L;
    long den = (long)(rawWet - rawDry);
    if (den == 0) return 0;

    long pct = num / den;
    if (pct < 0) {
        pct = 0;
    } else if (pct > 100) {
        pct = 100;
    }
    return (int)pct; // 0% แห้ง → 100% เปียก
}

/* ================= State Control (Pump/Fan) ================= */
static bool pumpOn = false;
static bool fanOn  = false;
static uint32_t pump_last_change_ms = 0, pump_on_started_ms = 0;
static uint32_t fan_last_change_ms  = 0;

static void set_pump(bool on)
{
    if (pumpOn == on) return;
    pumpOn = on;
    relay2ch_write(RELAY_PUMP_GPIO, on);  // Active-High
    pump_last_change_ms = ms_now();
    if (on) pump_on_started_ms = pump_last_change_ms;
}

static void set_fan(bool on)
{
    if (fanOn == on) return;
    fanOn = on;
    relay2ch_write(RELAY_FAN_GPIO, on);   // Active-High
    fan_last_change_ms = ms_now();
}

/* ================= DHT22 (robust bit-bang) ================= */
static int dht_wait_level(gpio_num_t pin, int level, int timeout_us)
{
    for (int t = 0; t <= timeout_us; ++t) {
        if (gpio_get_level(pin) == level) return t;
        esp_rom_delay_us(1);
    }
    return -1;
}

static bool dht22_try_once(float *out_tc, float *out_rh)
{
    uint8_t d[5] = {0};

    // Start signal
    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_PIN, 0);
    esp_rom_delay_us(1200);              // ≥1ms
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
    esp_rom_delay_us(30);                // bus settle

    // Response preamble (with relaxed timeouts)
    if (dht_wait_level(DHT_PIN, 0, 200) < 0) return false;
    if (dht_wait_level(DHT_PIN, 1, 220) < 0) return false;
    if (dht_wait_level(DHT_PIN, 0, 220) < 0) return false;

    // Read 40 bits
    for (int i = 0; i < 40; i++) {
        if (dht_wait_level(DHT_PIN, 1, 160) < 0) return false; // end of LOW
        int width = dht_wait_level(DHT_PIN, 0, 200);           // HIGH width
        if (width < 0) return false;
        d[i/8] <<= 1;
        d[i/8] |= (width > 45) ? 1 : 0;                        // tolerant threshold
    }

    uint8_t sum = (uint8_t)(d[0] + d[1] + d[2] + d[3]);
    if (sum != d[4]) return false;

    int16_t raw_rh   = (d[0] << 8) | d[1];
    int16_t raw_temp = (d[2] << 8) | d[3];
    float rh = raw_rh / 10.0f;
    float tc = (raw_temp & 0x8000) ? -((raw_temp & 0x7FFF)/10.0f) : (raw_temp/10.0f);

    if (out_tc) *out_tc = tc;
    if (out_rh) *out_rh = rh;
    return true;
}

static bool dht22_read(float *out_tc, float *out_rh)
{
    for (int tries = 0; tries < 3; ++tries) {
        if (dht22_try_once(out_tc, out_rh)) return true;
        vTaskDelay(pdMS_TO_TICKS(60));   // let bus settle before retry
    }
    return false;
}

/* ================= Tasks ================= */
// ไฟปลูก: วนรอบ 7 นาที (1–3 OFF, 4–7 ON) เก็บ start_ts ใน NVS
static void grow_cycle_task(void *arg) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    gpio_reset_pin(GROW_RELAY_GPIO);
    gpio_set_direction(GROW_RELAY_GPIO, GPIO_MODE_OUTPUT);
    grow_off();

    button_init();
    wifi_init_sta();     // ไม่บล็อกถาวร (timeout 10s)
    time_sync_wait();    // มี timeout ภายในอยู่แล้ว

    ESP_LOGI(TAG_GROW, "Hold RESET button %d ms to start new cycle.", RESET_HOLD_MS);
    vTaskDelay(pdMS_TO_TICKS(500));
    if (gpio_get_level(RESET_BTN_PIN) == 0 && button_held_ms(RESET_HOLD_MS)) {
        nvs_clear_start_ts();
        ESP_LOGW(TAG_GROW, "Reset requested.");
    }

    time_t start_ts = 0;
    if (nvs_get_start_ts(&start_ts) != ESP_OK || start_ts == 0) {
        time(&start_ts);
        nvs_set_start_ts(start_ts);
    }

    while (1) {
        time_t now; time(&now);
        int m = minutes_since(start_ts, now);
        apply_light_policy_minute(m);

        if (m >= TOTAL_MINUTES) {
            nvs_set_start_ts(now);
            start_ts = now;
            ESP_LOGI(TAG_GROW, "Auto-start new 7-min cycle");
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// Soil → ปั๊มน้ำ
static void soil_pump_task(void *arg)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SOIL_ADC_CH, SOIL_ADC_ATTEN);

    gpio_reset_pin(RELAY_PUMP_GPIO);
    gpio_set_direction(RELAY_PUMP_GPIO, GPIO_MODE_OUTPUT);
    set_pump(false);

    while (1) {
        int raw = soil_read_raw_median();
        int pct = soil_raw_to_pct(raw);
        uint32_t now = ms_now();

        // safety max-on
        if (pumpOn && (now - pump_on_started_ms >= PUMP_MAX_CONT_ON_MS)) {
            set_pump(false);
            ESP_LOGW(TAG_SOIL, "SAFETY: pump forced OFF");
        }

        bool can_on  = (!pumpOn) && (now - pump_last_change_ms >= PUMP_MIN_OFF_MS);
        bool can_off = ( pumpOn) && (now - pump_last_change_ms >= PUMP_MIN_ON_MS);

        if (!pumpOn && can_on && pct <= SOIL_TH_LOW) {
            set_pump(true);
        } else if (pumpOn && can_off && pct >= SOIL_TH_HIGH) {
            set_pump(false);
        }

        ESP_LOGI(TAG_SOIL, "RAW=%d Moisture=%d%% Pump=%s", raw, pct, pumpOn?"ON":"OFF");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// DHT22 → พัดลม
static void dht22_fan_task(void *arg)
{
    gpio_reset_pin(RELAY_FAN_GPIO);
    gpio_set_direction(RELAY_FAN_GPIO, GPIO_MODE_OUTPUT);
    set_fan(false);

    // เตรียมขา DHT + เปิด Pull-up ภายใน + หน่วงให้บัสนิ่ง
    gpio_reset_pin(DHT_PIN);
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DHT_PIN, GPIO_PULLUP_ONLY);
    vTaskDelay(pdMS_TO_TICKS(20));

    TickType_t last = xTaskGetTickCount();
    while (1) {
        float t=0, h=0;
        bool ok = dht22_read(&t,&h);
        if (ok) {
            uint32_t now = ms_now();
            bool can_on  = (!fanOn) && (now - fan_last_change_ms >= FAN_MIN_OFF_MS);
            bool can_off = ( fanOn) && (now - fan_last_change_ms >= FAN_MIN_ON_MS);

            if (!fanOn && can_on && t >= FAN_ON_TEMP_C) {
                set_fan(true);
            } else if (fanOn && can_off && t <= FAN_OFF_TEMP_C) {
                set_fan(false);
            }
            ESP_LOGI(TAG_DHT, "T=%.1fC RH=%.1f%% Fan=%s", t, h, fanOn?"ON":"OFF");
        } else {
            ESP_LOGW(TAG_DHT, "DHT read failed");
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(DHT_INTERVAL_MS));
    }
}

/* ================= app_main ================= */
void app_main(void)
{
    // 3 งานหลัก: ไฟปลูก, ปั๊ม, พัดลม
    xTaskCreatePinnedToCore(grow_cycle_task, "grow_cycle_task", 6144, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(soil_pump_task,  "soil_pump_task",  4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(dht22_fan_task,  "dht22_fan_task",  4096, NULL, 4, NULL, 1);
}
