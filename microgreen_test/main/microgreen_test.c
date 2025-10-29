// ===== microgreen_control.c (calibrated, pulse irrigation, multi-factor fan) =====
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
#define SOIL_ADC_ATTEN          ADC_ATTEN_DB_12
#undef  SOIL_SAMPLES_MEDIAN
#define SOIL_SAMPLES_MEDIAN     21               // เพิ่มความนิ่งด้วย median ที่ยาวขึ้น

/* ===== คาลิเบรตกับชุดจริง (จากการทดลองของคุณ) =====
   ใช้ "ดินชุ่มจริงในถาด" เป็นจุดอ้างอิงฝั่งเปียก (ไม่ใช่จุ่มน้ำล้วน) */
static int rawDry = 2688;           // ดินแห้ง/อากาศ (ค่าจริงจากชุดของคุณ)
static int rawWet = 1565;           // ดินชุ่มในถาดจริงหลังซึม (ค่าจริงจากชุดของคุณ)

/* ===== ฮิสเทอรีซิส % ความชื้นดิน =====
   ชั่วคราวปรับให้แห้งขึ้นเล็กน้อย เพื่อลดน้ำขัง: 65–78%
   (เมื่อระบบนิ่งแล้วค่อยขยับกลับ 70–80%) */
static int SOIL_TH_LOW  = 65;       // ≤65% เปิดรด (เริ่มโดส)
static int SOIL_TH_HIGH = 78;       // ≥78% ปิด/หยุดรด

// เวลาขั้นต่ำ/เซฟตี้ ปั๊มน้ำ (กันสวิง)
static const uint32_t PUMP_MIN_ON_MS      = 15*1000;
static const uint32_t PUMP_MIN_OFF_MS     = 10*1000;
static const uint32_t PUMP_MAX_CONT_ON_MS = 10*60*1000;

/* ===== Pulse Irrigation (ลดอิ่มน้ำ) ===== */
static const uint32_t PUMP_DOSE_MS            = 6000;       // รด 6 วินาทีต่อโดส
static const uint32_t PUMP_LOCKOUT_MS         = 3*60*1000;  // รอให้ซึม 3 นาทีหลังจบโดส
static const int      PUMP_MAX_DOSES_PER_ROUND= 3;          // โดสสูงสุดต่อ "รอบกระหายน้ำ" หนึ่งครั้ง
static const uint32_t PUMP_ROUND_RESET_MS     = 30*60*1000; // รีเซ็ตรอบโดสหลัง 30 นาที
static const int      BELOW_LOW_CONFIRM_N     = 5;          // ต่ำกว่า low ต่อเนื่อง 5 ครั้ง (~5s) ถึงจะเริ่มโดส

/* ---------- DHT22 → คุมพัดลม ---------- */
#define DHT_PIN                 GPIO_NUM_4
static const uint32_t DHT_INTERVAL_MS     = 2000;

/* อุณหภูมิ (คุมเหมือนเดิม) */
static const float    FAN_ON_TEMP_C       = 30.0f;  // ≥30°C เปิด
static const float    FAN_OFF_TEMP_C      = 28.0f;  // ≤28°C ปิด

/* เพิ่มเงื่อนไข RH และ Soil Saturation (อิงผลทดลองจริง) */
static const float    FAN_ON_RH           = 80.0f;  // RH ≥80% เปิด
static const float    FAN_OFF_RH          = 75.0f;  // RH ≤75% ปิด

static const int      SOIL_SAT_ON_PCT     = 85;     // ดินอิ่มน้ำมาก เปิดพัดลมช่วย
static const int      SOIL_SAT_OFF_PCT    = 78;     // ผ่อนคลายแล้วปิด
static const uint32_t SOIL_FAN_DELAY_MS   = 120*1000; // รอหลังเปิดปั๊ม 120s ก่อนใช้เงื่อนไขดินกับพัดลม

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

    esp_wifi_set_ps(WIFI_PS_NONE);

    ESP_LOGI(TAG_NET, "Connecting to Wi-Fi: %s ...", WIFI_SSID);

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
    // insertion sort for median
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
    if (pct < 0)      pct = 0;
    else if (pct > 100) pct = 100;
    return (int)pct; // 0% แห้ง → 100% เปียก
}

/* ================= Shared state / Control ================= */
static bool pumpOn = false;
static bool fanOn  = false;
static uint32_t pump_last_change_ms = 0, pump_on_started_ms = 0;
static uint32_t fan_last_change_ms  = 0;

static volatile int       g_soil_pct = 0;          // แชร์ %ดินให้ลอจิกพัดลม
static volatile uint32_t  g_last_pump_on_ms = 0;   // เวลาเปิดปั๊มล่าสุด

// Pulse irrigation state
static uint32_t pump_dose_started_ms  = 0;
static uint32_t pump_lock_until_ms    = 0;
static uint32_t pump_round_started_ms = 0;
static int      pump_doses_in_round   = 0;
static int      below_low_consec_cnt  = 0;

static inline bool in_pump_lockout(uint32_t now) { return now < pump_lock_until_ms; }
static inline bool round_expired(uint32_t now) {
    return (pump_round_started_ms==0) || (now - pump_round_started_ms >= PUMP_ROUND_RESET_MS);
}

static void set_pump(bool on)
{
    if (pumpOn == on) return;
    pumpOn = on;
    relay2ch_write(RELAY_PUMP_GPIO, on);  // Active-High
    uint32_t now = ms_now();
    pump_last_change_ms = now;

    if (on) {
        pump_on_started_ms   = now;
        pump_dose_started_ms = now;
        g_last_pump_on_ms    = now;

        if (round_expired(now)) {
            pump_round_started_ms = now;
            pump_doses_in_round   = 0;
        }
        pump_doses_in_round++;
    } else {
        // จบโดส → ล็อกช่วงรอให้ซึม
        pump_lock_until_ms = now + PUMP_LOCKOUT_MS;
    }
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

    // Response preamble
    if (dht_wait_level(DHT_PIN, 0, 250) < 0) return false;
    if (dht_wait_level(DHT_PIN, 1, 300) < 0) return false;
    if (dht_wait_level(DHT_PIN, 0, 300) < 0) return false;

    // Read 40 bits
    for (int i = 0; i < 40; i++) {
        if (dht_wait_level(DHT_PIN, 1, 200) < 0) return false;
        int width = dht_wait_level(DHT_PIN, 0, 260);
        if (width < 0) return false;
        d[i/8] <<= 1;
        d[i/8] |= (width > 42) ? 1 : 0;  // ลด threshold รับ '1' ได้แม้พัลส์หด
    }

    uint8_t sum = (uint8_t)(d[0] + d[1] + d[2] + d[3]);
    if (sum != d[4]) return false;
    if (d[0]==0 && d[1]==0 && d[2]==0 && d[3]==0) return false; // กันเฟรมศูนย์ผ่าน checksum

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
    for (int tries = 0; tries < 5; ++tries) {
        if (dht22_try_once(out_tc, out_rh)) return true;
        vTaskDelay(pdMS_TO_TICKS(150));
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
    wifi_init_sta();
    time_sync_wait();

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

// Soil → ปั๊มน้ำ (Pulse irrigation)
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
        g_soil_pct = pct; // แชร์ค่าความชื้นดินให้ลอจิกพัดลมด้วย

        uint32_t now = ms_now();

        // รีเซ็ตรอบโดสหากหมดอายุ
        if (round_expired(now)) {
            pump_round_started_ms = 0;
            pump_doses_in_round   = 0;
        }

        // safety: บังคับ OFF ถ้า ON นานเกิน
        if (pumpOn && (now - pump_on_started_ms >= PUMP_MAX_CONT_ON_MS)) {
            set_pump(false);
            ESP_LOGW(TAG_SOIL, "SAFETY: pump forced OFF (max-cont)");
        }

        // ถ้ากำลังรดเป็นโดส ครบเวลาแล้วให้ปิด เพื่อรอซึม
        if (pumpOn && (now - pump_dose_started_ms >= PUMP_DOSE_MS)) {
            set_pump(false);
        }

        // นับยืนยันต่ำกว่า LOW ต่อเนื่อง (กัน false trigger)
        if (pct <= SOIL_TH_LOW) {
            if (below_low_consec_cnt < BELOW_LOW_CONFIRM_N) below_low_consec_cnt++;
        } else {
            below_low_consec_cnt = 0;
        }

        // เงื่อนไขเริ่มโดสใหม่:
        bool can_try_open =
            (!pumpOn) &&
            !in_pump_lockout(now) &&
            (pump_doses_in_round < PUMP_MAX_DOSES_PER_ROUND) &&
            (now - pump_last_change_ms >= PUMP_MIN_OFF_MS) &&
            (below_low_consec_cnt >= BELOW_LOW_CONFIRM_N);

        if (can_try_open) {
            set_pump(true);
            below_low_consec_cnt = 0;
        }

        // เงื่อนไขหยุดทันที (ถ้าความชื้นแตะ HIGH และผ่าน min-on แล้ว)
        if (pumpOn && pct >= SOIL_TH_HIGH && (now - pump_last_change_ms >= PUMP_MIN_ON_MS)) {
            set_pump(false);
        }

        ESP_LOGI(TAG_SOIL,
            "RAW=%d Moisture=%d%% Pump=%s dose#=%d lock=%s",
            raw, pct, pumpOn?"ON":"OFF", pump_doses_in_round, in_pump_lockout(now)?"Y":"N");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// DHT22 → พัดลม (คุมจาก 3 ปัจจัย: Temp, RH, Soil saturation)
static void dht22_fan_task(void *arg)
{
    gpio_reset_pin(RELAY_FAN_GPIO);
    gpio_set_direction(RELAY_FAN_GPIO, GPIO_MODE_OUTPUT);
    set_fan(false);

    // เตรียมขา DHT + เปิด Pull-up ภายใน + อุ่นบัสก่อนอ่าน
    gpio_reset_pin(DHT_PIN);
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DHT_PIN, GPIO_PULLUP_ONLY);
    vTaskDelay(pdMS_TO_TICKS(2000));  // warm-up 2s

    TickType_t last = xTaskGetTickCount();
    while (1) {
        float t=0, h=0;
        bool ok = dht22_read(&t,&h);
        if (ok) {
            uint32_t now = ms_now();
            bool can_on  = (!fanOn) && (now - fan_last_change_ms >= FAN_MIN_OFF_MS);
            bool can_off = ( fanOn) && (now - fan_last_change_ms >= FAN_MIN_ON_MS);

            int  soil_pct       = g_soil_pct;                 // volatile → copy local
            bool soil_delay_ok  = (now - g_last_pump_on_ms >= SOIL_FAN_DELAY_MS);

            // เปิดถ้าเข้าอย่างน้อยหนึ่งข้อ
            bool need_by_temp = (t >= FAN_ON_TEMP_C);
            bool need_by_rh   = (h >= FAN_ON_RH);
            bool need_by_soil = (soil_delay_ok && soil_pct >= SOIL_SAT_ON_PCT);

            // ปิดเมื่อทุกข้อผ่อนคลาย
            bool relax_temp = (t <= FAN_OFF_TEMP_C);
            bool relax_rh   = (h <= FAN_OFF_RH);
            bool relax_soil = (soil_pct <= SOIL_SAT_OFF_PCT);

            if (!fanOn && can_on && (need_by_temp || need_by_rh || need_by_soil)) {
                set_fan(true);
            } else if (fanOn && can_off && (relax_temp && relax_rh && relax_soil)) {
                set_fan(false);
            }

            ESP_LOGI(TAG_DHT, "T=%.1fC RH=%.1f%% Soil=%d%% Fan=%s",
                     t, h, soil_pct, fanOn?"ON":"OFF");
        } else {
            ESP_LOGW(TAG_DHT, "DHT read failed");
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(DHT_INTERVAL_MS));
    }
}

/* ================= app_main ================= */
void app_main(void)
{
    // 3 งานหลัก: ไฟปลูก, ปั๊ม (pulse), พัดลม (multi-factor)
    xTaskCreatePinnedToCore(grow_cycle_task, "grow_cycle_task", 6144, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(soil_pump_task,  "soil_pump_task",  4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(dht22_fan_task,  "dht22_fan_task",  4096, NULL, 4, NULL, 1);
}
