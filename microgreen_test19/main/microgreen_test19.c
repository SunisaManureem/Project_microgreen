// ===== microgreen_control.c (Wi-Fi via BLE provisioning (microgreen01 / PoP 1234),
// Wi-Fi start→scan fix removed, WDT-safe, RAW-calibrated ADC, pulse irrigation,
// multi-factor fan, REAL-DAY grow light (REL window: start+6h → 14h ON for Day4–7)
// + Firebase REST (ESP32 → Realtime DB) + terminal-friendly STATE logs
// + Crop profiles selectable via Firebase: control/crop_profile
// + Control mode (auto/manual) + manual override for light/pump/fan via Firebase ) =====
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>   // for abs()
#include <ctype.h>    // for isspace()

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_sntp.h"

#include "driver/gpio.h"
#include "driver/adc.h"          // legacy ADC oneshot (simple use OK)
#include "esp_rom_sys.h"
#include "esp_idf_version.h"

/* ==== HTTP / TLS for Firebase ==== */
#include "esp_http_client.h"
#include "esp_crt_bundle.h"

/* ==== Wi-Fi provisioning (BLE) ==== */
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_ble.h"

/* Grow Light (Active-Low) */
#define GROW_RELAY_GPIO         GPIO_NUM_33
#define GROW_ACTIVE_LEVEL       0
#define GROW_INACTIVE_LEVEL     1

/* Relay 2CH (Active-High) */
#define RELAY_PUMP_GPIO         GPIO_NUM_27
#define RELAY_FAN_GPIO          GPIO_NUM_26

/* Reset button */
#define RESET_BTN_PIN           GPIO_NUM_0
#define RESET_HOLD_MS           3000
#define WIFI_CLEAR_HOLD_MS      8000   // long-press to FULL NVS erase (Wi-Fi + provisioning + grow)

/* ================= Grow-light policy (REAL DAYS in Thai time) =================
   Day 1–3: OFF
   Day 4–7: ON (ใช้ "หน้าต่างแบบสัมพัทธ์": start_ts + 6h → เปิด 14h ต่อวัน)
   Day 8+: OFF (no auto reset; press button to reset)
   g_grow_day = 1..7 */
#define TOTAL_DAYS              7
#define OFF_DAYS                3
/* ถ้าอยากให้ครบ 7 วันแล้วเริ่มรอบใหม่เอง ให้ตั้งเป็น 1 */
#define GROW_AUTO_RESET_AFTER_7 0

/* --- โหมดหน้าต่างไฟแบบสัมพัทธ์จากเวลาเริ่มปลูก --- */
#define LIGHT_WINDOW_MODE_RELATIVE   1      // ใช้โหมด RELATIVE
#define REL_LIGHT_OFFSET_H           6      // เริ่มเปิดหลังเริ่มปลูก 6 ชั่วโมง
#define REL_LIGHT_OFFSET_MIN         0
#define REL_LIGHT_DURATION_H         14     // เปิด 14 ชั่วโมง

/* ---------- Soil sensor (ADC) ---------- */
#define SOIL_ADC_CH             ADC1_CHANNEL_6   // GPIO34
#define SOIL_ADC_ATTEN          ADC_ATTEN_DB_12
#undef  SOIL_SAMPLES_MEDIAN
#define SOIL_SAMPLES_MEDIAN     21               // smoother median

/* ===== Calibrated RAW endpoints (from your tray) =====
   ใช้ค่าคาลิเบรตตามที่แนะนำ: ดินแห้ง≈2100, ชุ่มพร้อมปลูก≈1500 */
static int rawDry = 2100;    // dry (field-dry in tray)
static int rawWet = 1500;    // wet (field capacity in tray)

/* Pump timings */
static const uint32_t PUMP_MIN_ON_MS      = 15*1000;
static const uint32_t PUMP_MIN_OFF_MS     = 10*1000;
static const uint32_t PUMP_MAX_CONT_ON_MS = 10*60*1000;

/* Pulse irrigation */
static const uint32_t PUMP_DOSE_MS            = 6000;
static const uint32_t PUMP_LOCKOUT_MS         = 3*60*1000;
static const int      PUMP_MAX_DOSES_PER_ROUND= 3;
static const uint32_t PUMP_ROUND_RESET_MS     = 30*60*1000;
static const int      BELOW_LOW_CONFIRM_N     = 5;

/* ---------- DHT22 → Fan + Crop profiles ---------- */
#define DHT_PIN                 GPIO_NUM_4
static const uint32_t DHT_INTERVAL_MS     = 2000;

/* โปรไฟล์พืช: กำหนด threshold ไว้เป็นชุด ๆ */
typedef struct {
    int   soil_th_low;       // ≤ low → เริ่มรดน้ำ
    int   soil_th_high;      // ≥ high → หยุดรดน้ำทันที
    float fan_on_temp;       // T ≥ → เปิดพัดลม
    float fan_off_temp;      // T ≤ → ปิดพัดลมได้
    float fan_on_rh;         // RH ≥ → เปิดพัดลม
    float fan_off_rh;        // RH ≤ → ปิดพัดลมได้
    int   soil_sat_on_pct;   // ความชื้นดิน ≥ → ถือว่าอิ่มน้ำ ใช้ช่วยตัดสินใจเปิดพัดลม
    int   soil_sat_off_pct;  // ≤ → ถือว่าหายอิ่มน้ำแล้ว
} crop_profile_t;

/* โปรไฟล์ตัวอย่าง (ปรับค่าทีหลังได้) */
static const crop_profile_t PROFILE_SUNFLOWER = {
    .soil_th_low      = 65,
    .soil_th_high     = 78,
    .fan_on_temp      = 30.0f,
    .fan_off_temp     = 28.0f,
    .fan_on_rh        = 80.0f,
    .fan_off_rh       = 75.0f,
    .soil_sat_on_pct  = 85,
    .soil_sat_off_pct = 78,
};

static const crop_profile_t PROFILE_KALE = {
    .soil_th_low      = 60,
    .soil_th_high     = 75,
    .fan_on_temp      = 29.0f,
    .fan_off_temp     = 27.0f,
    .fan_on_rh        = 78.0f,
    .fan_off_rh       = 73.0f,
    .soil_sat_on_pct  = 83,
    .soil_sat_off_pct = 76,
};

static const crop_profile_t PROFILE_BROCCOLI = {
    .soil_th_low      = 65,
    .soil_th_high     = 80,
    .fan_on_temp      = 30.0f,
    .fan_off_temp     = 28.0f,
    .fan_on_rh        = 85.0f,
    .fan_off_rh       = 80.0f,
    .soil_sat_on_pct  = 88,
    .soil_sat_off_pct = 80,
};

/* โปรไฟล์ที่ใช้งานจริงตอนนี้ (ค่าเริ่มต้น sunflower) */
static crop_profile_t g_profile = {
    .soil_th_low      = 65,
    .soil_th_high     = 78,
    .fan_on_temp      = 30.0f,
    .fan_off_temp     = 28.0f,
    .fan_on_rh        = 80.0f,
    .fan_off_rh       = 75.0f,
    .soil_sat_on_pct  = 85,
    .soil_sat_off_pct = 78,
};

/* id ของโปรไฟล์ปัจจุบัน (ไว้เช็คว่าเปลี่ยนหรือยัง) */
static char g_profile_id[16] = "sunflower";

/* ดีเลย์พัดลมกับเวลาเปิดปั๊ม + min on/off (ใช้ร่วมกับ dht22_fan_task) */
static const uint32_t SOIL_FAN_DELAY_MS   = 120*1000;  // 120 วินาทีหลังจากปั๊มเคยเปิด
static const uint32_t FAN_MIN_ON_MS       = 10*1000;   // พัดลมต้องเปิดอย่างน้อย 10 วิ
static const uint32_t FAN_MIN_OFF_MS      = 10*1000;   // พัดลมต้องปิดอย่างน้อย 10 วิ

/* Logging control */
static const uint32_t HEARTBEAT_SOIL_MS   = 2000;   // 2s
static const uint32_t HEARTBEAT_DHT_MS    = 2000;   // 2s
static const uint32_t GROW_STATUS_LOG_MS  = 60000;  // 1m
static const int      DHT_FAIL_MAX        = 10;

/* TAGs */
static const char *TAG_GROW = "GROW";
static const char *TAG_NET  = "NET";
static const char *TAG_SOIL = "SOIL_PUMP";
static const char *TAG_DHT  = "DHT_FAN";

/* Control mode (auto/manual) */
typedef enum {
    CTRL_MODE_AUTO = 0,
    CTRL_MODE_MANUAL = 1
} control_mode_t;
static volatile control_mode_t g_control_mode = CTRL_MODE_AUTO;
static volatile bool g_manual_light = false;
static volatile bool g_manual_pump  = false;
static volatile bool g_manual_fan   = false;

/* ================= Wrap-safe ms helpers ================= */
static inline uint32_t ms32(void){ return (uint32_t)(esp_timer_get_time()/1000ULL); }
static inline bool elapsed_since(uint32_t since_ms, uint32_t dur_ms){ return (uint32_t)(ms32() - since_ms) >= dur_ms; }
/* ใช้ signed compare ป้องกัน underflow */
static inline bool not_yet(uint32_t until_ms){
    if (until_ms == 0) return false;                 // no lock when 0
    return (int32_t)(until_ms - ms32()) > 0;         // signed delta > 0 => still locked
}
static inline uint32_t remain_ms(uint32_t until_ms){
    if (until_ms == 0) return 0;
    int32_t d = (int32_t)(until_ms - ms32());
    return (d > 0) ? (uint32_t)d : 0;
}

/* ================= Time format helpers (for STATE JSON / Firebase) ================= */
static void now_ts_iso(int64_t *out_ms, char *iso, size_t isosz){
    struct timeval tv; gettimeofday(&tv, NULL);
    int64_t ms = (int64_t)tv.tv_sec*1000 + tv.tv_usec/1000;
    if (out_ms) *out_ms = ms;

    time_t t = tv.tv_sec;
    struct tm ti; localtime_r(&t, &ti);
    /* e.g. 2025-11-01T09:33:27+07:00 */
    strftime(iso, isosz, "%Y-%m-%dT%H:%M:%S%z", &ti);
    // turn +0700 -> +07:00
    size_t L = strlen(iso);
    if (L >= 5){
        char z1 = iso[L-5], z2 = iso[L-4], z3 = iso[L-3], z4 = iso[L-2], z5 = iso[L-1];
        if ((z1=='+'||z1=='-') && (z2>='0'&&z2<='9') && (z4>='0'&&z4<='9')) {
            iso[L-5] = z1; iso[L-4] = z2; iso[L-3] = ':'; iso[L-2] = z4; iso[L-1] = z5;
        }
    }
}

/* ================= GPIO helpers ================= */
static volatile bool g_light_on = false;

static inline void grow_on(void)  { gpio_set_level(GROW_RELAY_GPIO, GROW_ACTIVE_LEVEL);   g_light_on = true;  }
static inline void grow_off(void) { gpio_set_level(GROW_RELAY_GPIO, GROW_INACTIVE_LEVEL); g_light_on = false; }
static inline void relay2ch_write(gpio_num_t pin, bool turn_on) { gpio_set_level(pin, turn_on ? 1 : 0); }

/* ================= Shared state ================= */
static bool pumpOn = false;
static bool fanOn  = false;
static uint32_t pump_last_change_ms = 0, pump_on_started_ms = 0, pump_dose_started_ms = 0;
static uint32_t fan_last_change_ms  = 0;
static uint32_t pump_lock_until_ms  = 0;   // short lockout (3 นาที)

static volatile int       g_soil_pct = 0;
static volatile uint32_t  g_last_pump_on_ms = 0;

/* Grow-day counter (1..7; capped at 7) */
static volatile int       g_grow_day = 1;
static int                last_logged_grow_day = -1;
static uint32_t           grow_status_ms = 0; // รายงานสถานะไฟทุก 1 นาที

/* Pulse irrigation round state (30 นาที/รอบ) */
static uint32_t pump_round_started_ms = 0; // 0 = ไม่มีรอบ active
static int      pump_doses_in_round   = 0;
static int      below_low_consec_cnt  = 0;

/* DHT fail-safe */
static int      dht_fail_cnt          = 0;

/* สำหรับรับคำสั่ง start cycle จาก Firebase */
static int64_t  g_last_start_cycle_cmd_ms = 0;

/* ================= Soil RAW helpers ================= */
static int      last_raw = -1;
static int      same_raw_cnt = 0;
static const int RAW_MIN_VALID = 50;
static const int RAW_MAX_VALID = 4045;

/* New tuning knobs */
static const int RAW_TOLERANCE         = 10;     // treat changes within ±10 as "no change"
static const int RAW_STUCK_THRESHOLD   = 3600;   // 3600 consecutive "no-change" ≈ 1 hour

static bool soil_sensor_suspect(int raw){
    if (raw <= RAW_MIN_VALID || raw >= RAW_MAX_VALID) return true;
    if (last_raw < 0) { last_raw = raw; same_raw_cnt = 0; return false; }
    if (abs(raw - last_raw) <= RAW_TOLERANCE) same_raw_cnt++;
    else { last_raw = raw; same_raw_cnt = 0; }
    return (same_raw_cnt >= RAW_STUCK_THRESHOLD);
}

/* Median of RAW samples (simple insertion sort) */
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

/* RAW -> % moisture (wet = smaller RAW) */
static int soil_raw_to_pct(int raw){
    long num = (long)(raw - rawDry) * 100L;
    long den = (long)(rawWet - rawDry);
    if (den == 0) return 0;
    long pct = num / den;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (int)pct;
}

/* ================= Wi-Fi (Provisioning via BLE + event bits + backoff reconnect) ================= */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0   // associated
#define WIFI_GOT_IP_BIT     BIT1   // DHCP ok

static esp_netif_t* s_sta_netif = NULL;
static uint32_t s_next_reconn_ms = 0;    // backoff target (ms)
static uint32_t s_backoff_ms     = 1000; // 1s → max 10s

/* Clear Wi-Fi + provisioning state in NVS (used by 8s button & setup)
   ตอนนี้ปรับให้เป็น FULL NVS ERASE = ล้างข้อมูลทั้งหมดที่เก็บใน NVS:
   - Wi-Fi credentials
   - provisioning state ("prov")
   - grow/start_ts ฯลฯ
*/
static void clear_wifi_creds(void){
    ESP_LOGW(TAG_NET, "FULL NVS erase requested (Wi-Fi credentials + provisioning + grow state)");
    esp_err_t err = nvs_flash_erase();
    if (err != ESP_OK) {
        ESP_LOGE(TAG_NET, "nvs_flash_erase failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGW(TAG_NET, "NVS erased; all stored Wi-Fi/provisioning/grow data cleared");
    }
}

/* legacy BSSID lock helper (ยังใช้ตอนเคลียร์เฉย ๆ — ตอนนี้ไม่ได้เรียกจาก 8s แล้ว) */
static void nvs_wifi_clear_lock(void){
    nvs_handle_t nvh;
    if (nvs_open("wifi", NVS_READWRITE, &nvh) == ESP_OK){
        nvs_erase_key(nvh, "bssid");
        nvs_erase_key(nvh, "chan");
        nvs_commit(nvh);
        nvs_close(nvh);
        ESP_LOGW(TAG_NET, "Cleared saved BSSID/channel lock");
    }
}

/* reason helper */
static const char* wifi_disc_reason_str(int r){
    switch(r){
        case 201: return "AUTH_EXPIRE";
        case 202: return "AUTH_LEAVE";
        case 203: return "ASSOC_EXPIRE";
        case 204: return "ASSOC_FAIL/TOO_MANY";
        case 205: return "NOT_AUTHED/NOT_ASSOCED";
        case 2:   return "AUTH_FAIL";
        case 15:  return "4WAY_HANDSHAKE_TIMEOUT";
        default:  return "OTHER";
    }
}

/* backoff connect */
static void wifi_try_connect_with_backoff(void){
    uint32_t now = ms32();
    if (now < s_next_reconn_ms) return;
    esp_wifi_connect();
    s_next_reconn_ms = now + s_backoff_ms;
    if (s_backoff_ms < 10000) s_backoff_ms += 1000;
}

/* Provisioning event handler (BLE) */
static void wifi_prov_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_base == WIFI_PROV_EVENT) {
        switch (event_id) {
        case WIFI_PROV_START:
            ESP_LOGI(TAG_NET, "Provisioning started (BLE, service=\"microgreen01\")");
            break;
        case WIFI_PROV_CRED_RECV:
            ESP_LOGI(TAG_NET, "Provisioning: Wi-Fi credentials received");
            break;
        case WIFI_PROV_CRED_SUCCESS:
            ESP_LOGI(TAG_NET, "Provisioning: Wi-Fi credentials applied successfully");
            break;
        case WIFI_PROV_CRED_FAIL:
            ESP_LOGW(TAG_NET, "Provisioning: Wi-Fi credentials failed");
            break;
        case WIFI_PROV_END:
            ESP_LOGI(TAG_NET, "Provisioning: stopping and freeing BLE resources");
            wifi_prov_mgr_deinit();
            break;
        default:
            break;
        }
    }
}

/* Wi-Fi/IP event handler */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch(event_id){
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG_NET, "Wi-Fi started (STA)");
            s_backoff_ms = 1000; s_next_reconn_ms = 0;
            break;

        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG_NET, "Associated to AP");
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            break;

        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t*)event_data;
            int reason = d ? d->reason : -1;
            ESP_LOGW(TAG_NET, "Wi-Fi disconnected, reason=%d (%s)", reason, wifi_disc_reason_str(reason));
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT);

            /* simple reconnect with backoff */
            wifi_try_connect_with_backoff();
            break;
        }

        default: break;
        }
    } else if (event_base == IP_EVENT) {
        switch(event_id){
        case IP_EVENT_STA_GOT_IP: {
            ip_event_got_ip_t *e = (ip_event_got_ip_t*)event_data;
            ESP_LOGI(TAG_NET, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
            xEventGroupSetBits(s_wifi_event_group, WIFI_GOT_IP_BIT);
            s_backoff_ms = 1000; s_next_reconn_ms = 0;
            break;
        }
        case IP_EVENT_STA_LOST_IP:
            ESP_LOGW(TAG_NET, "Lost IP");
            xEventGroupClearBits(s_wifi_event_group, WIFI_GOT_IP_BIT);
            break;
        default: break;
        }
    }
}

/* New: Wi-Fi init using BLE provisioning (service_name=microgreen01, PoP=1234) */
static void wifi_init_sta_once(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Country/PHY config (เหมือนของเดิม) */
    wifi_country_t country = { .cc="TH", .schan=1, .nchan=13, .max_tx_power=20, .policy=WIFI_COUNTRY_POLICY_MANUAL };
    ESP_ERROR_CHECK(esp_wifi_set_country(&country));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    /* Register event handlers */
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_LOST_IP, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &wifi_prov_event_handler, NULL));

    /* ใช้ FLASH storage เพื่อให้ config จาก provisioning ถูกเก็บถาวร */
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    /* === Provisioning manager config (BLE) === */
    wifi_prov_mgr_config_t prov_cfg = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM
    };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(prov_cfg));

    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (!provisioned) {
        ESP_LOGI(TAG_NET, "Device not provisioned yet → starting BLE provisioning");

        const char *service_name = "microgreen01";
        const char *pop          = "1234";
        wifi_prov_security_t security = WIFI_PROV_SECURITY_1;

        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(
            security, pop, service_name, NULL));

        /* ไม่ใช้ wifi_prov_print_qr แล้ว
           ให้เปิดแอป ESP BLE Provisioning แล้วหาอุปกรณ์ชื่อ service_name แทน */
    } else {
        ESP_LOGI(TAG_NET, "Already provisioned → starting Wi-Fi STA directly");
        /* provisioning เสร็จแล้ว: ปิด manager แล้วสตาร์ท Wi-Fi เลย */
        wifi_prov_mgr_deinit();
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_ERROR_CHECK(esp_wifi_connect());
    }

    /* Wait up to 30s for (connected + got IP).
       ถ้าผู้ใช้ยังไม่ provision ภายใน 30s ก็จะยังไม่มี IP แต่ระบบจะทำงานต่อไป
       และจะเชื่อมต่อทันทีเมื่อ provisioning เสร็จ */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT,
                                           false, true, pdMS_TO_TICKS(30000));
    if ((bits & (WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT)) ==
        (WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT)) {
        ESP_LOGI(TAG_NET, "Wi-Fi ready (connected + IP)");
    } else {
        ESP_LOGW(TAG_NET,
                 "Wi-Fi not fully ready after 30s (bits=0x%02x). "
                 "If provisioning is still ongoing, it will connect later.",
                 (unsigned)bits);
    }
}

/* ================= SNTP / Time + TZ ================= */
static void time_sync_wait(void) {
    if (!esp_sntp_enabled()) {
        esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
        esp_sntp_setservername(0, "pool.ntp.org");
        esp_sntp_init();
    }
    setenv("TZ", "ICT-7", 1);
    tzset();

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

/* ================= NVS (start_ts) ================= */
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
        ESP_LOGW(TAG_GROW, "start_ts cleared");
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

/* -------- helper: ABSOLUTE window (คงไว้เผื่อใช้ในอนาคต แต่โหมดปัจจุบันใช้ REL) -------- */
static bool is_in_light_window_absolute(time_t now_ts, int start_h, int start_m, int dur_h) {
    struct tm ti;
    localtime_r(&now_ts, &ti);
    int minutes = ti.tm_hour * 60 + ti.tm_min;
    int start = start_h * 60 + start_m;
    int end   = (start + dur_h * 60) % (24*60);
    if (start < end) {
        return (minutes >= start && minutes < end);
    } else {
        return (minutes >= start || minutes < end);
    }
}

/* -------- helper: RELATIVE window (start_ts + offset → duration) -------- */
static bool is_in_light_window_relative(time_t start_ts, time_t now_ts) {
    if (now_ts < start_ts) return false;
    long elapsed = (long)difftime(now_ts, start_ts);        // วินาทีตั้งแต่เริ่มปลูก
    int day_idx  = (int)(elapsed / 86400L) + 1;             // Day 1..N
    int in_day   = (int)(elapsed % 86400L);                 // วินาทีในวันนั้น (0..86399)

    if (day_idx < 4 || day_idx > 7) return false;          // Day 1–3 OFF, Day 8+ OFF

    int start_s = REL_LIGHT_OFFSET_H*3600 + REL_LIGHT_OFFSET_MIN*60;
    int end_s   = (start_s + REL_LIGHT_DURATION_H*3600) % 86400;

    if (start_s < end_s) {
        return (in_day >= start_s && in_day < end_s);
    } else {
        // ข้ามเที่ยงคืน
        return (in_day >= start_s || in_day < end_s);
    }
}

/* ================= Grow-Light (REAL-DAY 7-day counter + REL window) ================= */
static int days_since(time_t start_ts, time_t now) {
    if (now <= start_ts) return 0;
    double diff_sec = difftime(now, start_ts);
    return (int)(diff_sec / 86400.0);  // full days elapsed
}
static void apply_light_policy_day(int d, time_t now_ts, time_t start_ts) {
#if LIGHT_WINDOW_MODE_RELATIVE
    // d = 0 → Day 1
    if (d < OFF_DAYS) {
        grow_off();
        ESP_LOGI(TAG_GROW, "Day %d: LIGHT = OFF (seedling dark phase, REL)", d + 1);
    } else if (d < TOTAL_DAYS) {
        bool inwin = is_in_light_window_relative(start_ts, now_ts);
        if (inwin) {
            grow_on();
            ESP_LOGI(TAG_GROW, "Day %d: LIGHT = ON (REL: start+%dh for %dh)", d + 1,
                     REL_LIGHT_OFFSET_H, REL_LIGHT_DURATION_H);
        } else {
            grow_off();
            ESP_LOGI(TAG_GROW, "Day %d: LIGHT = OFF (REL window)", d + 1);
        }
    } else {
        grow_off();
        ESP_LOGI(TAG_GROW, "Day %d+: LIGHT = OFF (beyond planned 7 days, REL)", d + 1);
    }
#else
    // fallback (absolute 06:00–20:00) — ไม่ถูกใช้ในโหมดปัจจุบัน
    if (d < OFF_DAYS) {
        grow_off();
        ESP_LOGI(TAG_GROW, "Day %d: LIGHT = OFF (seedling dark phase)", d + 1);
    } else if (d < TOTAL_DAYS) {
        if (is_in_light_window_absolute(now_ts, 6, 0, 14)) {
            grow_on();
            ESP_LOGI(TAG_GROW, "Day %d: LIGHT = ON (14h window ABS)", d + 1);
        } else {
            grow_off();
            ESP_LOGI(TAG_GROW, "Day %d: LIGHT = OFF (outside 14h window ABS)", d + 1);
        }
    } else {
        grow_off();
        ESP_LOGI(TAG_GROW, "Day %d+: LIGHT = OFF (beyond planned 7 days ABS)", d + 1);
    }
#endif
}

/* ================= Firebase REST helpers (ESP32 -> Realtime DB) ================= */

#define FIREBASE_BASE_URL "https://microgreen-iot-c9bed-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_DEVICE_ID "microgreen-01"

static esp_err_t firebase_put(const char *path, const char *json)
{
    char url[256];
    snprintf(url, sizeof(url), "%s/%s.json", FIREBASE_BASE_URL, path);

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_PUT,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 8000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        ESP_LOGE("FIREBASE", "init client fail");
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, json, strlen(json));

    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        ESP_LOGE("FIREBASE", "perform fail: %s", esp_err_to_name(err));
    } else {
        int status = esp_http_client_get_status_code(client);
        int len    = esp_http_client_get_content_length(client);
        ESP_LOGI("FIREBASE", "PUT %s HTTP %d len=%d", path, status, len);
        if (status < 200 || status >= 300) {
            err = ESP_FAIL;
        }
    }

    esp_http_client_cleanup(client);
    return err;
}

/* GET helper สำหรับอ่านค่า int64 จาก Firebase (ใช้กับ control/start_cycle_ms) */
static esp_err_t firebase_get_int64(const char *path, int64_t *out_val)
{
    char url[256];
    snprintf(url, sizeof(url), "%s/%s.json", FIREBASE_BASE_URL, path);

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        ESP_LOGE("FIREBASE", "GET(init,int64) fail");
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE("FIREBASE", "GET(open,int64) fail: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    /* อ่าน header (จะใช้หรือไม่ใช้ก็ได้) */
    esp_http_client_fetch_headers(client);

    char buf[64];
    int read_len = esp_http_client_read_response(client, buf, sizeof(buf) - 1);
    if (read_len < 0) {
        ESP_LOGE("FIREBASE", "GET(read,int64) fail");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }
    buf[read_len] = '\0';

    int status = esp_http_client_get_status_code(client);

    ESP_LOGI("FIREBASE", "GET(int64) body for %s: %s", path, buf);

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (status != 200) {
        ESP_LOGW("FIREBASE", "GET(int64) %s HTTP %d", path, status);
        return ESP_FAIL;
    }

    if (strcmp(buf, "null") == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    int64_t v = atoll(buf);
    if (out_val) *out_val = v;
    return ESP_OK;
}

/* GET helper สำหรับอ่าน string จาก Firebase (ใช้กับ control/crop_profile, control/mode) */
static esp_err_t firebase_get_string(const char *path, char *out, size_t out_sz)
{
    if (!out || out_sz == 0) return ESP_ERR_INVALID_ARG;

    char url[256];
    snprintf(url, sizeof(url), "%s/%s.json", FIREBASE_BASE_URL, path);

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        ESP_LOGE("FIREBASE", "GET(init,str) fail");
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE("FIREBASE", "GET(open,str) fail: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    /* อ่าน header (ไม่จำเป็นต้องใช้ค่า แต่ช่วยให้ flow ถูกต้อง) */
    esp_http_client_fetch_headers(client);

    char buf[128];
    int read_len = esp_http_client_read_response(client, buf, sizeof(buf) - 1);
    if (read_len < 0) {
        ESP_LOGE("FIREBASE", "GET(read,str) fail");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }
    buf[read_len] = '\0';

    int status = esp_http_client_get_status_code(client);

    ESP_LOGI("FIREBASE", "GET(str) body for %s: %s", path, buf);

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (status != 200) {
        ESP_LOGW("FIREBASE", "GET(str) %s HTTP %d", path, status);
        return ESP_FAIL;
    }

    if (strcmp(buf, "null") == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    // trim leading spaces
    char *p = buf;
    while (*p && isspace((unsigned char)*p)) p++;

    // ถ้าเป็น string แบบ "sunflower" / "kale"
    if (*p == '"') {
        p++;
        char *q = strchr(p, '"');
        if (!q) q = p + strlen(p);
        size_t n = (size_t)(q - p);
        if (n >= out_sz) n = out_sz - 1;
        memcpy(out, p, n);
        out[n] = '\0';
    } else {
        // plain text
        strncpy(out, p, out_sz - 1);
        out[out_sz - 1] = '\0';
    }

    return ESP_OK;
}

/* GET helper สำหรับอ่าน JSON manual control (light/pump/fan) */
static esp_err_t firebase_get_manual_control(const char *path,
                                             bool *light,
                                             bool *pump,
                                             bool *fan)
{
    char url[256];
    snprintf(url, sizeof(url), "%s/%s.json", FIREBASE_BASE_URL, path);

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        ESP_LOGE("FIREBASE", "GET(init,manual) fail");
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE("FIREBASE", "GET(open,manual) fail: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    esp_http_client_fetch_headers(client);

    char buf[256];
    int read_len = esp_http_client_read_response(client, buf, sizeof(buf) - 1);
    if (read_len < 0) {
        ESP_LOGE("FIREBASE", "GET(read,manual) fail");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }
    buf[read_len] = '\0';

    int status = esp_http_client_get_status_code(client);

    ESP_LOGI("FIREBASE", "GET(manual) body for %s: %s", path, buf);

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (status != 200) {
        ESP_LOGW("FIREBASE", "GET(manual) %s HTTP %d", path, status);
        return ESP_FAIL;
    }

    if (strcmp(buf, "null") == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    bool l = light ? *light : false;
    bool p = pump  ? *pump  : false;
    bool f = fan   ? *fan   : false;

    char *pos, *colon, *val;

    /* light */
    pos = strstr(buf, "\"light\"");
    if (pos) {
        colon = strchr(pos, ':');
        if (colon) {
            val = colon + 1;
            while (*val && isspace((unsigned char)*val)) val++;
            if (strncmp(val, "true", 4) == 0)      l = true;
            else if (strncmp(val, "false", 5) == 0) l = false;
        }
    }

    /* pump */
    pos = strstr(buf, "\"pump\"");
    if (pos) {
        colon = strchr(pos, ':');
        if (colon) {
            val = colon + 1;
            while (*val && isspace((unsigned char)*val)) val++;
            if (strncmp(val, "true", 4) == 0)      p = true;
            else if (strncmp(val, "false", 5) == 0) p = false;
        }
    }

    /* fan */
    pos = strstr(buf, "\"fan\"");
    if (pos) {
        colon = strchr(pos, ':');
        if (colon) {
            val = colon + 1;
            while (*val && isspace((unsigned char)*val)) val++;
            if (strncmp(val, "true", 4) == 0)      f = true;
            else if (strncmp(val, "false", 5) == 0) f = false;
        }
    }

    if (light) *light = l;
    if (pump)  *pump  = p;
    if (fan)   *fan   = f;

    return ESP_OK;
}

/* --- send wrappers for each module --- */

static void firebase_send_grow(int day, bool light, bool in_window)
{
    int64_t tsms; char iso[40];
    now_ts_iso(&tsms, iso, sizeof(iso));
    const char *ctrl_mode = (g_control_mode == CTRL_MODE_AUTO) ? "auto" : "manual";

    char body[256];
    snprintf(body, sizeof(body),
             "{\"ts\":%lld,\"iso\":\"%s\",\"day\":%d,"
             "\"light\":%s,\"in_window\":%s,\"ctrl_mode\":\"%s\"}",
             (long long)tsms, iso, day,
             light ? "true" : "false",
             in_window ? "true" : "false",
             ctrl_mode);

    char path[128];
    snprintf(path, sizeof(path),
             "devices/%s/grow", FIREBASE_DEVICE_ID);

    esp_err_t err = firebase_put(path, body);
    if (err != ESP_OK) {
        ESP_LOGE("FIREBASE", "PUT %s fail: %s", path, esp_err_to_name(err));
    }
}

static void firebase_send_soil(int moisture, int raw,
                               bool pump, int dose,
                               bool lock, uint32_t lock_rem_s,
                               int doses_in_round, bool round_limit,
                               uint32_t round_reset_s)
{
    int64_t tsms; char iso[40];
    now_ts_iso(&tsms, iso, sizeof(iso));

    char body[400];
    snprintf(body, sizeof(body),
             "{\"ts\":%lld,\"iso\":\"%s\",\"moisture\":%d,"
             "\"raw\":%d,\"pump\":%s,"
             "\"dose\":%d,"
             "\"lock\":%s,\"lock_rem\":%u,"
             "\"round\":{\"doses\":%d,\"limit\":%s,\"reset_in\":%u}}",
             (long long)tsms, iso, moisture,
             raw, pump ? "true" : "false",
             dose,
             lock ? "true" : "false", (unsigned)lock_rem_s,
             doses_in_round, round_limit ? "true" : "false",
             (unsigned)round_reset_s);

    char path[128];
    snprintf(path, sizeof(path),
             "devices/%s/soil", FIREBASE_DEVICE_ID);

    esp_err_t err = firebase_put(path, body);
    if (err != ESP_OK) {
        ESP_LOGE("FIREBASE", "PUT %s fail: %s", path, esp_err_to_name(err));
    }
}

static void firebase_send_env(float t, float rh, int soil, bool fan, int dht_fail)
{
    int64_t tsms; char iso[40];
    now_ts_iso(&tsms, iso, sizeof(iso));

    char body[256];
    snprintf(body, sizeof(body),
             "{\"ts\":%lld,\"iso\":\"%s\",\"t\":%.1f,\"rh\":%.1f,"
             "\"soil\":%d,\"fan\":%s,\"dht_fail\":%d}",
             (long long)tsms, iso, t, rh,
             soil, fan ? "true" : "false", dht_fail);

    char path[128];
    snprintf(path, sizeof(path),
             "devices/%s/env", FIREBASE_DEVICE_ID);

    esp_err_t err = firebase_put(path, body);
    if (err != ESP_OK) {
        ESP_LOGE("FIREBASE", "PUT %s fail: %s", path, esp_err_to_name(err));
    }
}

/* ================= Setters ================= */
static void set_pump(bool on)
{
    if (pumpOn == on) return;
    pumpOn = on;
    relay2ch_write(RELAY_PUMP_GPIO, on);
    pump_last_change_ms = ms32();

    if (on) {
        pump_on_started_ms   = pump_last_change_ms;
        pump_dose_started_ms = pump_last_change_ms;
        g_last_pump_on_ms    = pump_last_change_ms;

        // เริ่ม/รีเฟรชรอบเมื่อเพิ่งเริ่มหรือรอบหมดอายุ
        if (pump_round_started_ms==0 || elapsed_since(pump_round_started_ms, PUMP_ROUND_RESET_MS)) {
            pump_round_started_ms = pump_last_change_ms;
            pump_doses_in_round   = 0;
        }
        pump_doses_in_round++;
    } else {
        // ล็อกสั้นหลังจบโดส (3 นาที)
        pump_lock_until_ms = pump_last_change_ms + PUMP_LOCKOUT_MS;
    }

    ESP_LOGI(TAG_SOIL, "Pump %s (dose#=%d)", pumpOn?"ON":"OFF", pump_doses_in_round);
}
static void set_fan(bool on)
{
    if (fanOn == on) return;
    fanOn = on;
    relay2ch_write(RELAY_FAN_GPIO, on);
    fan_last_change_ms = ms32();
    ESP_LOGI(TAG_DHT, "Fan %s", fanOn?"ON":"OFF");
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
    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_PIN, 0);
    esp_rom_delay_us(1200);
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
    esp_rom_delay_us(30);

    if (dht_wait_level(DHT_PIN, 0, 250) < 0) return false;
    if (dht_wait_level(DHT_PIN, 1, 300) < 0) return false;
    if (dht_wait_level(DHT_PIN, 0, 300) < 0) return false;

    for (int i = 0; i < 40; i++) {
        if (dht_wait_level(DHT_PIN, 1, 200) < 0) return false;
        int width = dht_wait_level(DHT_PIN, 0, 260);
        if (width < 0) return false;
        d[i/8] <<= 1;
        d[i/8] |= (width > 42) ? 1 : 0;
    }

    uint8_t sum = (uint8_t)(d[0] + d[1] + d[2] + d[3]);
    if (sum != d[4]) return false;
    if (d[0]==0 && d[1]==0 && d[2]==0 && d[3]==0) return false;

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

/* ================= Crop profile helper ================= */
static void apply_crop_profile_by_id(const char *id)
{
    const crop_profile_t *p = NULL;

    if (strcmp(id, "sunflower") == 0) {
        p = &PROFILE_SUNFLOWER;
    } else if (strcmp(id, "kale") == 0) {
        p = &PROFILE_KALE;
    } else if (strcmp(id, "broccoli") == 0) {
        p = &PROFILE_BROCCOLI;
    }

    if (!p) {
        ESP_LOGW(TAG_GROW, "Unknown crop_profile \"%s\" (keep current profile: %s)", id, g_profile_id);
        return;
    }

    g_profile = *p;
    strncpy(g_profile_id, id, sizeof(g_profile_id) - 1);
    g_profile_id[sizeof(g_profile_id) - 1] = '\0';

    ESP_LOGI(TAG_GROW,
             "Applied crop profile: %s (soil %d–%d%%, fan T>=%.1fC/RH>=%.1f%%)",
             g_profile_id,
             g_profile.soil_th_low, g_profile.soil_th_high,
             g_profile.fan_on_temp, g_profile.fan_on_rh);
}

/* ================= Tasks ================= */
// Grow light: REAL-DAY cycle with REL window (Day 1–3 OFF, Day 4–7 start+6h → 14h ON; Day 8+ OFF)
static void grow_cycle_task(void *arg) {
    gpio_reset_pin(GROW_RELAY_GPIO);
    gpio_set_direction(GROW_RELAY_GPIO, GPIO_MODE_OUTPUT);
    grow_off();

    button_init();

    vTaskDelay(pdMS_TO_TICKS(500));
    if (gpio_get_level(RESET_BTN_PIN) == 0 && button_held_ms(WIFI_CLEAR_HOLD_MS)) {
        /* 8s hold during boot: FULL NVS erase (Wi-Fi + provisioning + grow) */
        clear_wifi_creds();
        ESP_LOGW(TAG_GROW, "Boot button(8s): FULL NVS erase (Wi-Fi + provisioning + grow), restarting…");
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_restart();
    } else if (gpio_get_level(RESET_BTN_PIN) == 0 && button_held_ms(RESET_HOLD_MS)) {
        nvs_clear_start_ts();
        ESP_LOGW(TAG_GROW, "Boot button(3s): reset grow cycle (start_ts cleared), restarting…");
        vTaskDelay(pdMS_TO_TICKS(50));
        esp_restart();
    }

    time_t start_ts = 0;
    if (nvs_get_start_ts(&start_ts) != ESP_OK || start_ts == 0) {
        time(&start_ts);
        nvs_set_start_ts(start_ts);
    }

    // ยิงสถานะทันทีหลังบูต
    grow_status_ms = ms32() - GROW_STATUS_LOG_MS;

    esp_task_wdt_add(NULL);

    static uint32_t hb_ms = 0;
    static uint32_t btn_ms = 0;   // ===== runtime button hold timer =====
    static uint32_t ctrl_poll_ms = 0; // ===== poll control จาก Firebase =====
    while (1) {
        // ===== RUNTIME BUTTON HANDLER =====
        if (gpio_get_level(RESET_BTN_PIN) == 0) {
            if (btn_ms == 0) btn_ms = ms32();
            // ≥8s → FULL NVS erase → restart
            if (elapsed_since(btn_ms, WIFI_CLEAR_HOLD_MS)) {
                clear_wifi_creds();
                ESP_LOGW(TAG_GROW, "Button(8s): FULL NVS erase (Wi-Fi + provisioning + grow), restarting…");
                vTaskDelay(pdMS_TO_TICKS(50));
                esp_restart();
            }
        } else {
            if (btn_ms && elapsed_since(btn_ms, RESET_HOLD_MS) && !elapsed_since(btn_ms, WIFI_CLEAR_HOLD_MS)) {
                // ≥3s and <8s → reset start_ts → restart
                nvs_clear_start_ts();
                ESP_LOGW(TAG_GROW, "Button(3s): reset grow cycle (start_ts cleared), restarting…");
                vTaskDelay(pdMS_TO_TICKS(50));
                esp_restart();
            }
            btn_ms = 0;
        }
        // ===== END BUTTON HANDLER =====

        // ===== เช็คคำสั่งจาก Firebase: start new cycle + crop_profile + mode + manual =====
        if (elapsed_since(ctrl_poll_ms, 10000)) { // ทุก 10 วินาที
            ctrl_poll_ms = ms32();

            /* 1) start_cycle_ms: เริ่มรอบปลูกใหม่ */
            int64_t cmd_ms = 0;
            esp_err_t e = firebase_get_int64(
                "devices/" FIREBASE_DEVICE_ID "/control/start_cycle_ms",
                &cmd_ms
            );
            if (e == ESP_OK && cmd_ms > 0 && cmd_ms != g_last_start_cycle_cmd_ms) {
                g_last_start_cycle_cmd_ms = cmd_ms;

                time(&start_ts);
                nvs_set_start_ts(start_ts);
                last_logged_grow_day = -1;   // ให้ log Day 1 ใหม่
                ESP_LOGW(TAG_GROW,
                         "Start cycle command from Firebase (ms=%lld) → reset Day 1",
                         (long long)cmd_ms);
            }

            /* 2) crop_profile: เลือกชนิดพืชจากแอป */
            char new_profile[16] = {0};
            e = firebase_get_string(
                "devices/" FIREBASE_DEVICE_ID "/control/crop_profile",
                new_profile, sizeof(new_profile)
            );
            if (e == ESP_OK && new_profile[0] != '\0'
                && strcmp(new_profile, g_profile_id) != 0) {

                ESP_LOGI(TAG_GROW, "New crop_profile from Firebase: \"%s\"", new_profile);
                apply_crop_profile_by_id(new_profile);
            } else if (e != ESP_OK && e != ESP_ERR_NOT_FOUND) {
                ESP_LOGW(TAG_GROW, "GET crop_profile fail: %s", esp_err_to_name(e));
            }

            /* 3) control/mode: auto / manual */
            char mode_str[16] = {0};
            e = firebase_get_string(
                "devices/" FIREBASE_DEVICE_ID "/control/mode",
                mode_str, sizeof(mode_str)
            );
            if (e == ESP_OK && mode_str[0] != '\0') {
                control_mode_t new_mode = CTRL_MODE_AUTO;
                if (strcmp(mode_str, "manual") == 0) new_mode = CTRL_MODE_MANUAL;
                else if (strcmp(mode_str, "auto") != 0) {
                    ESP_LOGW(TAG_GROW, "Unknown control mode \"%s\", keep %s",
                             mode_str,
                             (g_control_mode == CTRL_MODE_AUTO ? "auto" : "manual"));
                }
                if (new_mode != g_control_mode) {
                    g_control_mode = new_mode;
                    ESP_LOGI(TAG_GROW, "Control mode from Firebase: %s", mode_str);
                }
            } else if (e != ESP_OK && e != ESP_ERR_NOT_FOUND) {
                ESP_LOGW(TAG_GROW, "GET control/mode fail: %s", esp_err_to_name(e));
            }

            /* 4) control/manual: light/pump/fan */
            bool ml = g_manual_light;
            bool mp = g_manual_pump;
            bool mf = g_manual_fan;
            e = firebase_get_manual_control(
                "devices/" FIREBASE_DEVICE_ID "/control/manual",
                &ml, &mp, &mf
            );
            if (e == ESP_OK) {
                g_manual_light = ml;
                g_manual_pump  = mp;
                g_manual_fan   = mf;
                ESP_LOGI(TAG_GROW, "Manual control flags: light=%d pump=%d fan=%d",
                         (int)ml, (int)mp, (int)mf);
            } else if (e != ESP_OK && e != ESP_ERR_NOT_FOUND) {
                ESP_LOGW(TAG_GROW, "GET control/manual fail: %s", esp_err_to_name(e));
            }
        }
        // ===== END เช็คคำสั่ง Firebase =====

        time_t now; time(&now);
        int d = days_since(start_ts, now);

        int day_display = (d < TOTAL_DAYS) ? (d + 1) : TOTAL_DAYS;
        g_grow_day = day_display;
        if (day_display != last_logged_grow_day) {
            last_logged_grow_day = day_display;
            ESP_LOGI(TAG_GROW, "Grow Day %d/%d started (ts=%ld)", day_display, TOTAL_DAYS, (long)now);
        }

        /* Apply light policy ตามโหมด */
        if (g_control_mode == CTRL_MODE_AUTO) {
            apply_light_policy_day(d, now, start_ts);
        } else {
            // MANUAL: ใช้คำสั่งจากแอป
            if (g_manual_light) {
                grow_on();
            } else {
                grow_off();
            }
        }

        // รายงานสถานะไฟปลูกทุก 1 นาที + JSON (โหมด REL)
        if (elapsed_since(grow_status_ms, GROW_STATUS_LOG_MS)) {
            grow_status_ms = ms32();
            bool inwin_auto = is_in_light_window_relative(start_ts, now);
            bool inwin = (g_control_mode == CTRL_MODE_AUTO) ? inwin_auto : false;

            const char *ctrl_mode_str = (g_control_mode == CTRL_MODE_AUTO) ? "AUTO" : "MANUAL";
            const char *ctrl_mode_json = (g_control_mode == CTRL_MODE_AUTO) ? "auto" : "manual";

            // human log
            ESP_LOGI(TAG_GROW,
                     "STATUS(1m): day=%d LIGHT=%s window=%s (REL: +%dh/%dh, ctrl=%s)",
                     day_display, g_light_on?"ON":"OFF", inwin? "IN":"OUT",
                     REL_LIGHT_OFFSET_H, REL_LIGHT_DURATION_H, ctrl_mode_str);

            // JSON → terminal
            int64_t tsms; char iso[40];
            now_ts_iso(&tsms, iso, sizeof(iso));
            ESP_LOGI("STATE",
                     "{\"ts\":%lld,\"iso\":\"%s\",\"mod\":\"grow\",\"day\":%d,"
                     "\"light\":%s,\"in_window\":%s,\"mode\":\"REL\",\"offset_h\":%d,\"dur_h\":%d,"
                     "\"ctrl_mode\":\"%s\"}",
                     (long long)tsms, iso, day_display,
                     g_light_on? "true":"false",
                     inwin? "true":"false",
                     REL_LIGHT_OFFSET_H, REL_LIGHT_DURATION_H,
                     ctrl_mode_json);

            // JSON → Firebase
            firebase_send_grow(day_display, g_light_on, inwin);
        }

#if GROW_AUTO_RESET_AFTER_7
        if (d >= TOTAL_DAYS) {
            nvs_set_start_ts(now);
            start_ts = now;
            ESP_LOGI(TAG_GROW, "Auto-start new 7-day cycle");
        }
#endif

        if (elapsed_since(hb_ms, 5000)) {
            hb_ms = ms32();
            ESP_LOGD(TAG_GROW, "HB: d=%d day_display=%d", d, day_display);
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Soil → Pump (pulse irrigation) — RAW calibrated + fail-safe (tolerant stuck detector)
static void soil_pump_task(void *arg)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SOIL_ADC_CH, SOIL_ADC_ATTEN);

    gpio_reset_pin(RELAY_PUMP_GPIO);
    gpio_set_direction(RELAY_PUMP_GPIO, GPIO_MODE_OUTPUT);
    set_pump(false);
    pump_lock_until_ms = 0;   // explicit no lock at boot

    esp_task_wdt_add(NULL);

    static uint32_t hb_ms = 0;
    while (1) {
        int raw = soil_read_raw_median();
        bool suspect = soil_sensor_suspect(raw);

        bool hard_invalid = (raw <= RAW_MIN_VALID || raw >= RAW_MAX_VALID);

        int pct      = soil_raw_to_pct(raw);
        g_soil_pct   = pct;

        uint32_t now = ms32();

        // หมดรอบ 30 นาที → เคลียร์สถานะรอบ
        if (pump_round_started_ms!=0 && elapsed_since(pump_round_started_ms, PUMP_ROUND_RESET_MS)) {
            pump_round_started_ms = 0;
            pump_doses_in_round   = 0;
        }

        // ความปลอดภัยปั๊ม (max continuous time) — ใช้ทั้ง auto/manual
        if (pumpOn && elapsed_since(pump_on_started_ms, PUMP_MAX_CONT_ON_MS)) {
            set_pump(false);
            ESP_LOGW(TAG_SOIL, "SAFETY: pump forced OFF (max-cont)");
        }

        bool is_manual = (g_control_mode == CTRL_MODE_MANUAL);

        if (is_manual) {
            /* โหมด MANUAL: ปั๊มตามคำสั่งจากแอป โดยยังมี max-cont safety ด้านบน */
            below_low_consec_cnt = 0;  // รีเซ็ต counter hysteresis
            if (g_manual_pump) {
                if (!pumpOn) set_pump(true);
            } else {
                if (pumpOn) set_pump(false);
            }
        } else {
            /* ===== AUTO MODE: hysteresis + pulse irrigation ===== */

            /* ตัดปั๊มทันทีเมื่อความชื้นถึง HIGH (ตามโปรไฟล์) */
            if (pumpOn && pct >= g_profile.soil_th_high) {
                set_pump(false);  // ยังมี short lock 3 นาทีตามเดิม
                ESP_LOGI(TAG_SOIL, "Cutoff: moisture ≥ %d%% → stop now", g_profile.soil_th_high);
            }

            // ปิดเมื่อครบโดส 6 วิ
            if (pumpOn && elapsed_since(pump_dose_started_ms, PUMP_DOSE_MS)) {
                set_pump(false);
            }

            // คอนเฟิร์มต่ำกว่า LOW ต่อเนื่อง (ตามโปรไฟล์)
            if (!hard_invalid && pct <= g_profile.soil_th_low) {
                if (below_low_consec_cnt < BELOW_LOW_CONFIRM_N) below_low_consec_cnt++;
            } else {
                below_low_consec_cnt = 0;
            }

            bool off_gap_ok = elapsed_since(pump_last_change_ms, PUMP_MIN_OFF_MS);
            bool short_locked = not_yet(pump_lock_until_ms);

            // round-limit?
            bool round_limit = (pump_round_started_ms!=0 && pump_doses_in_round >= PUMP_MAX_DOSES_PER_ROUND);
            uint32_t round_reset_left_ms = 0;
            if (pump_round_started_ms!=0 && !elapsed_since(pump_round_started_ms, PUMP_ROUND_RESET_MS)) {
                round_reset_left_ms = PUMP_ROUND_RESET_MS - (now - pump_round_started_ms);
            }

            bool can_try_open =
                (!pumpOn) &&
                !hard_invalid &&
                !short_locked &&
                !round_limit &&
                off_gap_ok &&
                (below_low_consec_cnt >= BELOW_LOW_CONFIRM_N);

            // ตัดสินใจ (human log)
            ESP_LOGI(TAG_SOIL,
                     "DECIDE: lock=%d hard_invalid=%d suspect=%d below_cnt=%d off_gap_ok=%d doses=%d round_limit=%d th_lo=%d th_hi=%d",
                     short_locked?1:0, hard_invalid?1:0, suspect?1:0,
                     below_low_consec_cnt, off_gap_ok?1:0, pump_doses_in_round, round_limit?1:0,
                     g_profile.soil_th_low, g_profile.soil_th_high);

            if (can_try_open) {
                set_pump(true);
                below_low_consec_cnt = 0;
            }

            /* เซนเซอร์ผิดปกติ: ใช้เฉพาะในโหมด auto (ถ้าใช้ manual จะทำให้ปั๊มกระพริบ) */
            if (suspect) {
                static uint32_t last_suspect_log_ms = 0;
                if (pumpOn && elapsed_since(pump_last_change_ms, PUMP_MIN_ON_MS)) {
                    set_pump(false);
                }
                if (elapsed_since(last_suspect_log_ms, 10000)) {
                    last_suspect_log_ms = now;
                    ESP_LOGW(TAG_SOIL, "Soil sensor suspect: RAW=%d same_cnt=%d (tol=%d thr=%d)",
                             raw, same_raw_cnt, RAW_TOLERANCE, RAW_STUCK_THRESHOLD);
                }
            }
        }

        // รายงาน (human + JSON) ทุก 2 วินาที
        if (elapsed_since(hb_ms, HEARTBEAT_SOIL_MS)) {
            hb_ms = now;

            bool short_locked = not_yet(pump_lock_until_ms);
            uint32_t short_left = remain_ms(pump_lock_until_ms);
            uint32_t short_left_s = (short_left+999)/1000;

            bool round_limit = (pump_round_started_ms!=0 && pump_doses_in_round >= PUMP_MAX_DOSES_PER_ROUND);
            uint32_t round_reset_left_ms = 0;
            if (pump_round_started_ms!=0 && !elapsed_since(pump_round_started_ms, PUMP_ROUND_RESET_MS)) {
                round_reset_left_ms = PUMP_ROUND_RESET_MS - (now - pump_round_started_ms);
            }
            uint32_t round_left_s = (round_reset_left_ms+999)/1000;

            // human
            ESP_LOGI(TAG_SOIL,
                     "STATUS(2s): Moisture=%d%% RAW=%d Pump=%s dose#=%d lock=%s",
                     pct, raw, pumpOn?"ON":"OFF", pump_doses_in_round,
                     short_locked?"Y":"N");
            if (short_locked) {
                ESP_LOGI(TAG_SOIL, "LOCKOUT: short 3m remaining ~%us", (unsigned)(short_left/1000));
            }
            if (round_limit) {
                ESP_LOGI(TAG_SOIL, "ROUND lock: dose limit reached (3/%d), reset in %um %us",
                         PUMP_MAX_DOSES_PER_ROUND,
                         (unsigned)(round_reset_left_ms/60000),
                         (unsigned)((round_reset_left_ms/1000)%60));
            }

            // JSON → terminal
            int64_t tsms; char iso[40];
            now_ts_iso(&tsms, iso, sizeof(iso));

            ESP_LOGI("STATE",
                     "{\"ts\":%lld,\"iso\":\"%s\",\"mod\":\"soil\","
                     "\"moisture\":%d,\"raw\":%d,\"pump\":%s,"
                     "\"dose\":%d,"
                     "\"lock\":%s,\"lock_rem\":%u,"
                     "\"round\":{\"doses\":%d,\"limit\":%s,\"reset_in\":%u}}",
                     (long long)tsms, iso, pct, raw, pumpOn?"true":"false",
                     pump_doses_in_round,
                     short_locked?"true":"false", (unsigned)short_left_s,
                     pump_doses_in_round, round_limit?"true":"false", (unsigned)round_left_s);

            // JSON → Firebase
            firebase_send_soil(pct, raw, pumpOn,
                               pump_doses_in_round,
                               short_locked, short_left_s,
                               pump_doses_in_round, round_limit, round_left_s);
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// DHT22 → Fan (multi-factor) + min on/off + fail counter + JSON logs
static void dht22_fan_task(void *arg)
{
    gpio_reset_pin(RELAY_FAN_GPIO);
    gpio_set_direction(RELAY_FAN_GPIO, GPIO_MODE_OUTPUT);
    set_fan(false);

    gpio_reset_pin(DHT_PIN);
    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DHT_PIN, GPIO_PULLUP_ONLY);
    vTaskDelay(pdMS_TO_TICKS(2000));  // warm-up

    TickType_t last = xTaskGetTickCount();

    esp_task_wdt_add(NULL);

    static uint32_t hb_ms = 0;
    while (1) {
        float t=0, h=0;
        bool ok = dht22_read(&t,&h);
        if (!ok) {
            dht_fail_cnt++;
            if (dht_fail_cnt == DHT_FAIL_MAX) {
                ESP_LOGW(TAG_DHT, "DHT read failed x%d, holding fan state (fail-safe)", dht_fail_cnt);
            } else if (dht_fail_cnt > DHT_FAIL_MAX && (dht_fail_cnt % 20 == 0)) {
                ESP_LOGW(TAG_DHT, "DHT still failing x%d", dht_fail_cnt);
            }
        } else {
            dht_fail_cnt = 0;

            uint32_t now = ms32();
            int  soil_pct = g_soil_pct;

            if (g_control_mode == CTRL_MODE_MANUAL) {
                /* โหมด MANUAL: ให้พัดลมทำตามคำสั่งจากแอปโดยตรง (ยังอ่าน DHT เพื่อส่งค่าไป Firebase) */
                set_fan(g_manual_fan);
            } else {
                /* AUTO MODE: multi-factor control */
                bool can_on  = (!fanOn) && elapsed_since(fan_last_change_ms, FAN_MIN_OFF_MS);
                bool can_off = ( fanOn) && elapsed_since(fan_last_change_ms, FAN_MIN_ON_MS);

                bool soil_delay_ok  = elapsed_since(g_last_pump_on_ms, SOIL_FAN_DELAY_MS);

                bool need_by_temp = (t >= g_profile.fan_on_temp);
                bool need_by_rh   = (h >= g_profile.fan_on_rh);
                bool need_by_soil = (soil_delay_ok && soil_pct >= g_profile.soil_sat_on_pct);

                bool relax_temp = (t <= g_profile.fan_off_temp);
                bool relax_rh   = (h <= g_profile.fan_off_rh);
                bool relax_soil = (soil_pct <= g_profile.soil_sat_off_pct);

                if (!fanOn && can_on && (need_by_temp || need_by_rh || need_by_soil)) {
                    set_fan(true);
                } else if (fanOn && can_off && (relax_temp && relax_rh && relax_soil)) {
                    set_fan(false);
                }
            }

            // human + JSON ทุก 2 วินาที
            if (elapsed_since(hb_ms, HEARTBEAT_DHT_MS)) {
                hb_ms = now;

                // human
                ESP_LOGI(TAG_DHT, "STATUS(2s): T=%.1fC RH=%.1f%% Soil=%d%% Fan=%s (fail=%d)",
                         t, h, soil_pct, fanOn?"ON":"OFF", dht_fail_cnt);

                // JSON → terminal
                int64_t tsms; char iso[40];
                now_ts_iso(&tsms, iso, sizeof(iso));
                ESP_LOGI("STATE",
                         "{\"ts\":%lld,\"iso\":\"%s\",\"mod\":\"env\","
                         "\"t\":%.1f,\"rh\":%.1f,\"soil\":%d,"
                         "\"fan\":%s,\"dht_fail\":%d}",
                         (long long)tsms, iso, t, h, soil_pct,
                         fanOn?"true":"false", dht_fail_cnt);

                // JSON → Firebase
                firebase_send_env(t, h, soil_pct, fanOn, dht_fail_cnt);
            }
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(DHT_INTERVAL_MS));
    }
}

/* ================= app_main ================= */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_task_wdt_config_t twdt_cfg = {
        .timeout_ms     = 5000,
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic  = true
    };
    esp_err_t wdt_err = esp_task_wdt_init(&twdt_cfg);
    if (wdt_err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW("WDT", "TWDT already initialized, keep existing config");
    } else {
        ESP_ERROR_CHECK(wdt_err);
    }

    /* Wi-Fi + provisioning (BLE) */
    wifi_init_sta_once();

    /* SNTP after Wi-Fi (ถ้า Wi-Fi ยังไม่พร้อมจะพยายาม sync ภายใน 30s) */
    time_sync_wait();

    xTaskCreatePinnedToCore(grow_cycle_task, "grow_cycle_task", 6144, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(soil_pump_task,  "soil_pump_task",  4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(dht22_fan_task,  "dht22_fan_task",  4096, NULL, 4, NULL, 1);
}
// --- EOF ---
