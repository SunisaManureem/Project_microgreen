// ===== microgreen_control.c (Wi-Fi start→scan fix, BSSID lock, backoff, WDT-safe,
// RAW-calibrated ADC, pulse irrigation, multi-factor fan,
// REAL-DAY grow light with 7-day counter + 14h/day window
// + TEST MODE: 1 minute ≡ 1 day, minute 1–3 OFF, 4–7 ON (sec 6–19), 8+ OFF) =====
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>   // for abs()
#include <stdbool.h>  // for bool
#include <sys/time.h> // <<< added for gettimeofday()

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

/* ======================== USER DEFAULTS (used when NVS empty) ======================== */
#define WIFI_SSID_DEFAULT  "Areena"
#define WIFI_PASS_DEFAULT  "1234567890"

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
#define WIFI_CLEAR_HOLD_MS      8000   // long-press to clear Wi-Fi creds + lock

/* ================= Grow-light policy (REAL DAYS in Thai time) =================
   Day 1–3: OFF
   Day 4–7: ON (แต่เปิดตาม "หน้าต่างเวลา" 14 ชม./วัน)
   Day 8+: OFF (no auto reset; press button to reset)
   มีตัวนับวันปลูก g_grow_day = 1..7 */
#define TOTAL_DAYS              7
#define OFF_DAYS                3
/* ถ้าอยากให้ครบ 7 วันแล้วเริ่มรอบใหม่เอง ให้ตั้งเป็น 1 */
#define GROW_AUTO_RESET_AFTER_7 0

/* ---- 14 ชั่วโมง/วัน (ตั้งเวลาได้) ---- */
#define LIGHT_ON_START_HOUR     6   // เริ่ม 06:00 น.
#define LIGHT_ON_START_MIN      0
#define LIGHT_ON_DURATION_H     14  // เปิด 14 ชั่วโมง/วัน

/* ==================== TEST MODE (FOR EASY DEMO) ====================
   ตั้ง 1 นาที = 1 วัน:
   นาที 1–3 ปิด,
   นาที 4–7 เปิดเฉพาะ "วินาที 6–19" ของแต่ละนาที,
   นาที >=8 ปิดยาวจนกดปุ่มรีเซ็ต NVS start_ts
*/
#define GROW_TEST_MINUTE_DAY       1        // 1=เปิดโหมดทดสอบ, 0=โหมดจริง
#define TEST_LIGHT_SEC_START       6        // เริ่มเปิดตั้งแต่วินาทีที่ 6
#define TEST_LIGHT_SEC_END         19       // จนถึงวินาทีที่ 19 (รวมปลาย)
/* ================================================================== */

/* ---------- Soil sensor (ADC) ---------- */
#define SOIL_ADC_CH             ADC1_CHANNEL_6   // GPIO34
#define SOIL_ADC_ATTEN          ADC_ATTEN_DB_12
#undef  SOIL_SAMPLES_MEDIAN
#define SOIL_SAMPLES_MEDIAN     21               // smoother median

/* ===== Calibrated RAW endpoints (from your tray) =====
   ใช้ค่าคาลิเบรตตามที่แนะนำ: ดินแห้ง≈2100, ชุ่มพร้อมปลูก≈1500 */
static int rawDry = 2100;    // dry (field-dry in tray)
static int rawWet = 1500;    // wet (field capacity in tray)

/* Moisture hysteresis (%) — tuned to 65–80 per recommendation */
static int SOIL_TH_LOW  = 65;    // ≤65% start dosing
static int SOIL_TH_HIGH = 78;    // ≥78% stop dosing

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

/* ---------- DHT22 → Fan ---------- */
#define DHT_PIN                 GPIO_NUM_4
static const uint32_t DHT_INTERVAL_MS     = 2000;
static const float    FAN_ON_TEMP_C       = 30.0f;
static const float    FAN_OFF_TEMP_C      = 28.0f;
static const float    FAN_ON_RH           = 80.0f;
static const float    FAN_OFF_RH          = 75.0f;
static const int      SOIL_SAT_ON_PCT     = 85;
static const int      SOIL_SAT_OFF_PCT    = 78;
static const uint32_t SOIL_FAN_DELAY_MS   = 120*1000;
/* Align with your calibrated file: 10s min ON/OFF (was 30s) */
static const uint32_t FAN_MIN_ON_MS       = 10*1000;
static const uint32_t FAN_MIN_OFF_MS      = 10*1000;

/* Logging control */
static const uint32_t HEARTBEAT_MS        = 5000;
static const int      DHT_FAIL_MAX        = 10;

/* TAGs */
static const char *TAG_GROW = "GROW";
static const char *TAG_NET  = "NET";
static const char *TAG_SOIL = "SOIL_PUMP";
static const char *TAG_DHT  = "DHT_FAN";

/* ================= Wrap-safe ms helpers ================= */
static inline uint32_t ms32(void){ return (uint32_t)(esp_timer_get_time()/1000ULL); }
static inline bool elapsed_since(uint32_t since_ms, uint32_t dur_ms){ return (uint32_t)(ms32() - since_ms) >= dur_ms; }
/* FIXED: use signed compare to avoid unsigned underflow keeping lock forever */
static inline bool not_yet(uint32_t until_ms){
    if (until_ms == 0) return false;                 // no lock when 0
    return (int32_t)(until_ms - ms32()) > 0;         // signed delta > 0 => still locked
}

/* ================= GPIO helpers ================= */
static inline void grow_on(void)  { gpio_set_level(GROW_RELAY_GPIO, GROW_ACTIVE_LEVEL); }
static inline void grow_off(void) { gpio_set_level(GROW_RELAY_GPIO, GROW_INACTIVE_LEVEL); }
static inline void relay2ch_write(gpio_num_t pin, bool turn_on) { gpio_set_level(pin, turn_on ? 1 : 0); }

/* ================= Shared state ================= */
static bool pumpOn = false;
static bool fanOn  = false;
static uint32_t pump_last_change_ms = 0, pump_on_started_ms = 0;
static uint32_t fan_last_change_ms  = 0;

static volatile int       g_soil_pct = 0;
static volatile uint32_t  g_last_pump_on_ms = 0;

/* New: grow-day counter (1..7; capped at 7) */
static volatile int       g_grow_day = 1;
static int                last_logged_grow_day = -1;

/* Pulse irrigation state */
static uint32_t pump_dose_started_ms  = 0;
static uint32_t pump_lock_until_ms    = 0;
static uint32_t pump_round_started_ms = 0;
static int      pump_doses_in_round   = 0;
static int      below_low_consec_cnt  = 0;

/* DHT fail-safe */
static int      dht_fail_cnt          = 0;

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
static int soil_read_raw_median(void) {
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

/* ================= Wi-Fi (NVS creds + scan/lock BSSID + backoff) ================= */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT  BIT0   // associated
#define WIFI_GOT_IP_BIT     BIT1   // DHCP ok

static esp_netif_t* s_sta_netif = NULL;
static uint32_t s_next_reconn_ms = 0;    // backoff target (ms)
static uint32_t s_backoff_ms     = 1000; // 1s → max 10s
static int      s_disc_streak    = 0;    // consecutive disconnects

/* NVS helpers for Wi-Fi */
static esp_err_t nvs_get_str2(nvs_handle_t nvh, const char* key, char* buf, size_t bufsize){
    size_t len = bufsize;
    esp_err_t e = nvs_get_str(nvh, key, buf, &len);
    if (e == ESP_OK && len>0) return ESP_OK;
    return e;
}
static void nvs_wifi_store_bssid_chan(const uint8_t bssid[6], uint8_t chan){
    nvs_handle_t nvh;
    if (nvs_open("wifi", NVS_READWRITE, &nvh) == ESP_OK){
        nvs_set_blob(nvh, "bssid", bssid, 6);
        nvs_set_u8(nvh, "chan", chan);
        nvs_commit(nvh);
        nvs_close(nvh);
        ESP_LOGI(TAG_NET, "Saved BSSID %02X:%02X:%02X:%02X:%02X:%02X ch=%u",
                 bssid[0],bssid[1],bssid[2],bssid[3],bssid[4],bssid[5], chan);
    }
}
static bool nvs_wifi_load_bssid_chan(uint8_t bssid[6], uint8_t *chan){
    nvs_handle_t nvh;
    size_t len=6;
    if (nvs_open("wifi", NVS_READONLY, &nvh) != ESP_OK) return false;
    if (nvs_get_blob(nvh, "bssid", bssid, &len) != ESP_OK || len!=6){ nvs_close(nvh); return false; }
    uint8_t c=0;
    if (nvs_get_u8(nvh, "chan", &c) != ESP_OK){ nvs_close(nvh); return false; }
    nvs_close(nvh);
    *chan = c;
    return true;
}
static void nvs_wifi_clear_lock(void){
    nvs_handle_t nvh;
    if (nvs_open("wifi", NVS_READWRITE, &nvh) == ESP_OK){
        nvs_erase_key(nvh, "bssid"); nvs_erase_key(nvh, "chan");
        nvs_commit(nvh); nvs_close(nvh);
        ESP_LOGW(TAG_NET, "Cleared saved BSSID/channel lock");
    }
}

/* creds load/init */
static void load_or_init_wifi_creds(char* ssid_out, size_t ssid_sz, char* pass_out, size_t pass_sz){
    nvs_handle_t nvh;
    if (nvs_open("wifi", NVS_READWRITE, &nvh) == ESP_OK){
        if (nvs_get_str2(nvh, "ssid", ssid_out, ssid_sz) != ESP_OK){
            strncpy(ssid_out, WIFI_SSID_DEFAULT, ssid_sz-1);
            nvs_set_str(nvh, "ssid", ssid_out);
            nvs_commit(nvh);
            ESP_LOGW(TAG_NET, "NVS wifi.ssid empty → set default");
        }
        if (nvs_get_str2(nvh, "pass", pass_out, pass_sz) != ESP_OK){
            strncpy(pass_out, WIFI_PASS_DEFAULT, pass_sz-1);
            nvs_set_str(nvh, "pass", pass_out);
            nvs_commit(nvh);
            ESP_LOGW(TAG_NET, "NVS wifi.pass empty → set default");
        }
        nvs_close(nvh);
    } else {
        strncpy(ssid_out, WIFI_SSID_DEFAULT, ssid_sz-1);
        strncpy(pass_out, WIFI_PASS_DEFAULT, pass_sz-1);
        ESP_LOGW(TAG_NET, "Open NVS wifi namespace failed → using defaults");
    }
}
static void clear_wifi_creds(void){
    nvs_handle_t nvh;
    if (nvs_open("wifi", NVS_READWRITE, &nvh) == ESP_OK){
        nvs_erase_all(nvh);
        nvs_commit(nvh);
        nvs_close(nvh);
        ESP_LOGW(TAG_NET, "Wi-Fi credentials cleared in NVS");
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

/* scan for target SSID and lock best BSSID+channel
   NOTE: must be called AFTER esp_wifi_start() */
static bool wifi_scan_and_lock_target(const char* target_ssid, uint8_t out_bssid[6], uint8_t *out_chan){
    uint8_t ssid_bytes[33] = {0};
    size_t sl = strlen(target_ssid);
    if (sl > 32) sl = 32;
    memcpy(ssid_bytes, target_ssid, sl);

    wifi_scan_config_t sc = {
        .ssid = ssid_bytes,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time = { .active = { .min=100, .max=300 } }
    };
    ESP_ERROR_CHECK(esp_wifi_scan_start(&sc, true)); // block until done

    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    if (n == 0) { ESP_LOGW(TAG_NET, "Scan: no AP found for SSID '%s'", target_ssid); return false; }
    wifi_ap_record_t *recs = calloc(n, sizeof(*recs));
    if (!recs) return false;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&n, recs));

    int best = -1; int best_rssi = -127;
    for (int i=0;i<n;i++){
        if (strncmp((const char*)recs[i].ssid, target_ssid, 32)==0){
            if (recs[i].rssi > best_rssi){
                best_rssi = recs[i].rssi;
                best = i;
            }
        }
    }
    bool ok=false;
    if (best>=0){
        memcpy(out_bssid, recs[best].bssid, 6);
        *out_chan = recs[best].primary;
        ESP_LOGI(TAG_NET, "Lock target: %s RSSI=%d ch=%u BSSID=%02X:%02X:%02X:%02X:%02X:%02X",
                 target_ssid, recs[best].rssi, recs[best].primary,
                 recs[best].bssid[0],recs[best].bssid[1],recs[best].bssid[2],
                 recs[best].bssid[3],recs[best].bssid[4],recs[best].bssid[5]);
        ok=true;
    } else {
        ESP_LOGW(TAG_NET, "Scan: SSID '%s' not found in records", target_ssid); // <-- fixed format
    }
    free(recs);
    return ok;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        switch(event_id){
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG_NET, "Wi-Fi started");
            s_backoff_ms = 1000; s_next_reconn_ms = 0; s_disc_streak = 0;
            break;

        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG_NET, "Associated to AP");
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            s_disc_streak = 0;
            break;

        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t*)event_data;
            int reason = d ? d->reason : -1;
            ESP_LOGW(TAG_NET, "Wi-Fi disconnected, reason=%d (%s)", reason, wifi_disc_reason_str(reason));
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT);
            s_disc_streak++;

            if (s_disc_streak >= 3 || reason==201 || reason==204 || reason==205){
                char ssid[33]={0}, pass[65]={0};
                load_or_init_wifi_creds(ssid, sizeof(ssid), pass, sizeof(pass));
                uint8_t bssid[6]; uint8_t ch=0;
                if (wifi_scan_and_lock_target(ssid, bssid, &ch)) {
                    nvs_wifi_store_bssid_chan(bssid, ch);
                    wifi_config_t cfg; memset(&cfg,0,sizeof(cfg));
                    esp_wifi_get_config(WIFI_IF_STA, &cfg);
                    strncpy((char*)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid)-1);
                    strncpy((char*)cfg.sta.password, pass, sizeof(cfg.sta.password)-1);
                    cfg.sta.bssid_set = true; memcpy(cfg.sta.bssid, bssid, 6);
                    cfg.sta.channel = ch;
                    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
                }
            }
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
            s_backoff_ms = 1000; s_next_reconn_ms = 0; s_disc_streak = 0;
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

static void wifi_init_sta_once(void) {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();

    wifi_country_t country = { .cc="TH", .schan=1, .nchan=13, .max_tx_power=20, .policy=WIFI_COUNTRY_POLICY_MANUAL };
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_country(&country));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_LOST_IP, &wifi_event_handler, NULL, NULL));

    char ssid[33]={0}, pass[65]={0};
    load_or_init_wifi_creds(ssid, sizeof(ssid), pass, sizeof(pass));

    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid)-1);
    strncpy((char*)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password)-1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
#if ESP_IDF_VERSION_MAJOR >= 5
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
#ifdef CONFIG_WPA3_SAE_PWE_H2E
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
#endif
#endif
    wifi_config.sta.listen_interval = 0;
    wifi_config.sta.scan_method   = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.sort_method   = WIFI_CONNECT_AP_BY_SIGNAL;

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());                // <<< START FIRST

    uint8_t bssid[6]; uint8_t ch=0;
    bool locked = nvs_wifi_load_bssid_chan(bssid, &ch);
    if (!locked) {
        if (wifi_scan_and_lock_target(ssid, bssid, &ch)) {
            nvs_wifi_store_bssid_chan(bssid, ch);
            locked = true;
        }
    }
    if (locked) {
        wifi_config_t cfg2; memset(&cfg2,0,sizeof(cfg2));
        esp_wifi_get_config(WIFI_IF_STA, &cfg2);
        strncpy((char*)cfg2.sta.ssid, ssid, sizeof(cfg2.sta.ssid)-1);
        strncpy((char*)cfg2.sta.password, pass, sizeof(cfg2.sta.password)-1);
        cfg2.sta.bssid_set = true; memcpy(cfg2.sta.bssid, bssid, 6);
        cfg2.sta.channel = ch;
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg2));
        ESP_LOGI(TAG_NET, "Applying BSSID lock ch=%u and connecting…", ch);
    } else {
        ESP_LOGW(TAG_NET, "No BSSID lock available, connecting by SSID only…");
    }

    ESP_ERROR_CHECK(esp_wifi_connect());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT,
                                           false, true, pdMS_TO_TICKS(30000));
    if ((bits & (WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT)) == (WIFI_CONNECTED_BIT | WIFI_GOT_IP_BIT)) {
        ESP_LOGI(TAG_NET, "Wi-Fi ready (connected + IP)");
    } else {
        ESP_LOGW(TAG_NET, "Not fully ready after 30s (bits=0x%02x). Will keep reconnecting.", (unsigned)bits);
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

/* -------- helper: อยู่ในหน้าต่างเวลาไฟปลูก (โหมดจริง) หรือไม่ -------- */
static bool is_in_light_window(time_t now_ts) {
    struct tm ti;
    localtime_r(&now_ts, &ti);
    int minutes = ti.tm_hour * 60 + ti.tm_min;
    int start = LIGHT_ON_START_HOUR * 60 + LIGHT_ON_START_MIN;
    int end   = (start + LIGHT_ON_DURATION_H * 60) % (24*60);
    if (start < end) {
        return (minutes >= start && minutes < end);
    } else {
        // เผื่อกรณีข้ามเที่ยงคืน (เช่น เริ่ม 20:00 เปิด 14 ชม.)
        return (minutes >= start || minutes < end);
    }
}

/* ======== TEST helper: เปิดเฉพาะ "วินาที 6–19" ของแต่ละนาที (ช่วงนาที 4–7 เท่านั้น) ======== */
#if GROW_TEST_MINUTE_DAY
static bool is_in_light_window_test(time_t now_ts) {
    (void)now_ts; // ใช้เวลาปัจจุบันจาก gettimeofday() เพื่อความละเอียดสูง
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int sec  = (int)(tv.tv_sec % 60);
    int usec = (int)tv.tv_usec; // 0..999999

    // เปิดเมื่อช่วง [6.000s, 19.999s] ของแต่ละนาที (inclusive)
    if (sec < TEST_LIGHT_SEC_START) return false;
    if (sec > TEST_LIGHT_SEC_END)   return false;
    // sec อยู่ใน [6,19] แล้ว → ทั้งช่วงถือว่า ON (รวมวินาทีที่ 19 จนก่อนข้ามไป 20)
    return true;
}
#endif

/* ================= Grow-Light (REAL-DAY 7-day counter + 14h window) ================= */
#if !GROW_TEST_MINUTE_DAY
static int days_since(time_t start_ts, time_t now) {
    if (now <= start_ts) return 0;
    double diff_sec = difftime(now, start_ts);
    return (int)(diff_sec / 86400.0);  // full days elapsed
}
#endif

/* =================== Generic "units since start" (day or minute) =================== */
static int units_since_start(time_t start_ts, time_t now) {
#if GROW_TEST_MINUTE_DAY
    if (now <= start_ts) return 0;
    double diff_sec = difftime(now, start_ts);
    return (int)(diff_sec / 60.0);
    // 1 unit = 1 minute (TEST)
#else
    return days_since(start_ts, now);  // 1 unit = 1 day (REAL)
#endif
}

/* ============= Apply policy per unit (test uses minute; real uses day) ============= */
#if !GROW_TEST_MINUTE_DAY
static void apply_light_policy_day(int d, time_t now_ts) {
    // d = 0 → Day 1
    if (d < OFF_DAYS) {
        grow_off();
        ESP_LOGI(TAG_GROW, "Day %d: LIGHT = OFF (seedling dark phase)", d + 1);
    } else if (d < TOTAL_DAYS) {
        if (is_in_light_window(now_ts)) {
            grow_on();
            ESP_LOGI(TAG_GROW, "Day %d: LIGHT = ON (14h window)", d + 1);
        } else {
            grow_off();
            ESP_LOGI(TAG_GROW, "Day %d: LIGHT = OFF (outside 14h window)", d + 1);
        }
    } else {
        grow_off();
        ESP_LOGI(TAG_GROW, "Day %d+: LIGHT = OFF (beyond planned 7 days)", d + 1);
    }
}
#endif

/* ----------------------- TEST policy (minute-based) ----------------------- */
#if GROW_TEST_MINUTE_DAY
static void apply_light_policy_minute(int m, time_t now_ts) {
    // log เฉพาะเมื่อสถานะไฟเปลี่ยนจริง ลดสแปม log และแม่นยำตามวินาที
    static int last_light = -1; // -1=unknown, 0=OFF, 1=ON
    int want_on = 0;
    const char* reason = NULL;

    if (m < OFF_DAYS) {
        // นาที 1–3 : OFF
        want_on = 0;
        reason = "seedling dark phase";
    } else if (m < TOTAL_DAYS) {
        // นาที 4–7 : ON เฉพาะวินาที 6–19
        bool inwin = is_in_light_window_test(now_ts);
        want_on = inwin ? 1 : 0;
        reason = inwin ? "sec 6–19 of each minute" : "outside sec 6–19 window";
    } else {
        // นาที 8+ : OFF
        want_on = 0;
        reason = "beyond planned 7 minutes";
    }

    if (last_light != want_on) {
        last_light = want_on;
        if (want_on) {
            grow_on();
            ESP_LOGI(TAG_GROW, "Minute %d: LIGHT = ON (%s)", m + 1, reason);
        } else {
            grow_off();
            ESP_LOGI(TAG_GROW, "Minute %d: LIGHT = OFF (%s)", m + 1, reason);
        }
    }
}
#endif

/* ===================== Setters (pump / fan) ===================== */
static void set_pump(bool on) {
    if (pumpOn == on) return;
    pumpOn = on;
    relay2ch_write(RELAY_PUMP_GPIO, on);
    pump_last_change_ms = ms32();

    if (on) {
        pump_on_started_ms   = pump_last_change_ms;
        pump_dose_started_ms = pump_last_change_ms;
        g_last_pump_on_ms    = pump_last_change_ms;

        if (pump_round_started_ms==0 || elapsed_since(pump_round_started_ms, PUMP_ROUND_RESET_MS)) {
            pump_round_started_ms = pump_last_change_ms;
            pump_doses_in_round   = 0;
        }
        pump_doses_in_round++;
    } else {
        pump_lock_until_ms = pump_last_change_ms + PUMP_LOCKOUT_MS;
    }

    ESP_LOGI(TAG_SOIL, "Pump %s (dose#=%d)", pumpOn?"ON":"OFF", pump_doses_in_round);
}

static void set_fan(bool on) {
    if (fanOn == on) return;
    fanOn = on;
    relay2ch_write(RELAY_FAN_GPIO, on);
    fan_last_change_ms = ms32();
    ESP_LOGI(TAG_DHT, "Fan %s", fanOn?"ON":"OFF");
}

/* ================= DHT22 (robust bit-bang) ================= */
static int dht_wait_level(gpio_num_t pin, int level, int timeout_us) {
    for (int t = 0; t <= timeout_us; ++t) {
        if (gpio_get_level(pin) == level) return t;
        esp_rom_delay_us(1);
    }
    return -1;
}
static bool dht22_try_once(float *out_tc, float *out_rh) {
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
static bool dht22_read(float *out_tc, float *out_rh) {
    for (int tries = 0; tries < 5; ++tries) {
        if (dht22_try_once(out_tc, out_rh)) return true;
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    return false;
}

/* ================= Tasks ================= */
// Grow light: REAL-DAY cycle or TEST-MINUTE cycle (config above)
static void grow_cycle_task(void *arg) {
    gpio_reset_pin(GROW_RELAY_GPIO);
    gpio_set_direction(GROW_RELAY_GPIO, GPIO_MODE_OUTPUT);
    grow_off();

    button_init();  // ใช้ปุ่ม BOOT (GPIO0) Active-LOW

    time_t start_ts = 0;
    if (nvs_get_start_ts(&start_ts) != ESP_OK || start_ts == 0) {
        time(&start_ts);
        nvs_set_start_ts(start_ts);
    }

    // --- ปุ่ม: วัดระยะค้างและตัดสินใจเมื่อ "ปล่อยปุ่ม" ---
    uint32_t press_start_ms = 0;
    bool     was_low        = false;

    esp_task_wdt_add(NULL);

    static uint32_t hb_ms = 0;
    while (1) {
        // ===== ปุ่มกดตลอดเวลา (non-blocking) =====
        int lvl   = gpio_get_level(RESET_BTN_PIN); // 0=กด
        uint32_t nowms = ms32();

        if (lvl == 0) { // กำลังกด
            if (!was_low) { was_low = true; press_start_ms = nowms; }
        } else { // ปล่อย
            if (was_low && press_start_ms != 0) {
                uint32_t held = nowms - press_start_ms;
                was_low = false;
                press_start_ms = 0;

                if (held >= WIFI_CLEAR_HOLD_MS) {
                    clear_wifi_creds();
                    nvs_wifi_clear_lock();
                    nvs_clear_start_ts();
                    time(&start_ts);
                // เริ่มรอบใหม่ทันที
                    nvs_set_start_ts(start_ts);
                    last_logged_grow_day = -1;       // บังคับ log รอบใหม่
                    ESP_LOGW(TAG_NET,  "Long press >= %u ms: clear Wi-Fi + BSSID lock", WIFI_CLEAR_HOLD_MS);
                    ESP_LOGW(TAG_GROW, "start_ts cleared & restarted");
                } else if (held >= RESET_HOLD_MS) {
                    nvs_clear_start_ts();
                    time(&start_ts);
                // เริ่มรอบใหม่ทันที
                    nvs_set_start_ts(start_ts);
                    last_logged_grow_day = -1;
                    ESP_LOGW(TAG_GROW, "Long press >= %u ms: reset grow cycle", RESET_HOLD_MS);
                } else {
                    ESP_LOGI(TAG_GROW, "Short press ignored (%u ms)", held);
                }
            }
        }

        // ===== คำนวณนับวัน/นาที =====
        time_t now; time(&now);

        int u = units_since_start(start_ts, now);  // day or minute per mode
        int display = (u < TOTAL_DAYS) ? (u + 1) : TOTAL_DAYS;
        g_grow_day = display;                      // ใช้ตัวแปรเดิมเพื่อโชว์สถานะ

        if (display != last_logged_grow_day) {
            last_logged_grow_day = display;
#if GROW_TEST_MINUTE_DAY
            ESP_LOGI(TAG_GROW, "Test Minute %d/%d started (ts=%ld)", display, TOTAL_DAYS, (long)now);
#else
            ESP_LOGI(TAG_GROW, "Grow Day %d/%d started (ts=%ld)", display, TOTAL_DAYS, (long)now);
#endif
        }

#if GROW_TEST_MINUTE_DAY
        apply_light_policy_minute(u, now);
#else
        apply_light_policy_day(u, now);
#endif

#if GROW_AUTO_RESET_AFTER_7
        if (u >= TOTAL_DAYS) {
            nvs_set_start_ts(now);
            start_ts = now;
            ESP_LOGI(TAG_GROW, "Auto-start new 7-unit cycle");
        }
#endif

        if (elapsed_since(hb_ms, HEARTBEAT_MS)) {
            hb_ms = ms32();
            ESP_LOGD(TAG_GROW, "HB: u=%d display=%d", u, display);
        }

        esp_task_wdt_reset();
#if GROW_TEST_MINUTE_DAY
        vTaskDelay(pdMS_TO_TICKS(200));   // <<< finer tick in TEST for precise sec 6–19
#else
        vTaskDelay(pdMS_TO_TICKS(1000));
#endif
    }
}

// Soil → Pump (pulse irrigation) — RAW calibrated + fail-safe (tolerant stuck detector)
static void soil_pump_task(void *arg) {
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

        if (pump_round_started_ms==0 || elapsed_since(pump_round_started_ms, PUMP_ROUND_RESET_MS)) {
            pump_round_started_ms = 0;
            pump_doses_in_round   = 0;
        }

        if (pumpOn && elapsed_since(pump_on_started_ms, PUMP_MAX_CONT_ON_MS)) {
            set_pump(false);
            ESP_LOGW(TAG_SOIL, "SAFETY: pump forced OFF (max-cont)");
        }

        if (pumpOn && elapsed_since(pump_dose_started_ms, PUMP_DOSE_MS)) {
            set_pump(false);
        }

        if (!hard_invalid && pct <= SOIL_TH_LOW) {
            if (below_low_consec_cnt < BELOW_LOW_CONFIRM_N) below_low_consec_cnt++;
        } else {
            below_low_consec_cnt = 0;
        }

        bool off_gap_ok = elapsed_since(pump_last_change_ms, PUMP_MIN_OFF_MS);

        bool can_try_open =
            (!pumpOn) &&
            !hard_invalid &&
            !not_yet(pump_lock_until_ms) &&
            (pump_doses_in_round < PUMP_MAX_DOSES_PER_ROUND) &&
            off_gap_ok &&
            (below_low_consec_cnt >= BELOW_LOW_CONFIRM_N);

        ESP_LOGI(TAG_SOIL, "DECIDE: lock=%d hard_invalid=%d suspect=%d below_cnt=%d off_gap_ok=%d doses=%d",
                 not_yet(pump_lock_until_ms)?1:0, hard_invalid?1:0, suspect?1:0,
                 below_low_consec_cnt, off_gap_ok?1:0, pump_doses_in_round);

        if (can_try_open) {
            set_pump(true);
            below_low_consec_cnt = 0;
        }

        if (pumpOn && pct >= SOIL_TH_HIGH && elapsed_since(pump_last_change_ms, PUMP_MIN_ON_MS)) {
            set_pump(false);
        }

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

        if (elapsed_since(hb_ms, HEARTBEAT_MS)) {
            hb_ms = now;
            static int last_soil_pct_logged = -1;
            if (last_soil_pct_logged < 0 || (abs(pct - last_soil_pct_logged) >= 3)) {
                last_soil_pct_logged = pct;
                ESP_LOGI(TAG_SOIL, "Moisture=%d%% RAW=%d Pump=%s dose#=%d lock=%s",
                         pct, raw, pumpOn?"ON":"OFF", pump_doses_in_round, not_yet(pump_lock_until_ms)?"Y":"N");
            } else {
                ESP_LOGD(TAG_SOIL, "HB: pct=%d pump=%d", pct, pumpOn);
            }
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// DHT22 → Fan (multi-factor) + min on/off + fail counter
static void dht22_fan_task(void *arg) {
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
            bool can_on  = (!fanOn) && elapsed_since(fan_last_change_ms, FAN_MIN_OFF_MS);
            bool can_off = ( fanOn) && elapsed_since(fan_last_change_ms, FAN_MIN_ON_MS);

            int  soil_pct       = g_soil_pct;
            bool soil_delay_ok  = elapsed_since(g_last_pump_on_ms, SOIL_FAN_DELAY_MS);

            bool need_by_temp = (t >= FAN_ON_TEMP_C);
            bool need_by_rh   = (h >= FAN_ON_RH);
            bool need_by_soil = (soil_delay_ok && soil_pct >= SOIL_SAT_ON_PCT);

            bool relax_temp = (t <= FAN_OFF_TEMP_C);
            bool relax_rh   = (h <= FAN_OFF_RH);
            bool relax_soil = (soil_pct <= SOIL_SAT_OFF_PCT);

            if (!fanOn && can_on && (need_by_temp || need_by_rh || need_by_soil)) {
                set_fan(true);
            } else if (fanOn && can_off && (relax_temp && relax_rh && relax_soil)) {
                set_fan(false);
            }

            if (elapsed_since(hb_ms, HEARTBEAT_MS)) {
                hb_ms = now;
                ESP_LOGI(TAG_DHT, "T=%.1fC RH=%.1f%% Soil=%d%% Fan=%s (fail=%d)",
                         t, h, soil_pct, fanOn?"ON":"OFF", dht_fail_cnt);
            }
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&last, pdMS_TO_TICKS(DHT_INTERVAL_MS));
    }
}

/* ================= app_main ================= */
void app_main(void) {
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

    wifi_init_sta_once();
    time_sync_wait();

    xTaskCreatePinnedToCore(grow_cycle_task, "grow_cycle_task", 6144, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(soil_pump_task,  "soil_pump_task",  4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(dht22_fan_task,  "dht22_fan_task",  4096, NULL, 4, NULL, 1);
}
