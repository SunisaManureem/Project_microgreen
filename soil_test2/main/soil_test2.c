#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "SOIL_TEST";

// ปรับตรงนี้ตามขาที่คุณเสียบ
#define SOIL_ADC_UNIT      ADC_UNIT_1
#define SOIL_ADC_CHANNEL   ADC_CHANNEL_6   // GPIO34 = ADC1_CH6

// ค่าแคลิเบรต "คร่าวๆ" (คุณค่อยปรับทีหลัง)
// DRY  = ค่าตอน "แห้ง" (ยกหัววัดลอยอากาศ)
// WET  = ค่าตอน "เปียก" (จุ่มน้ำ/ดินชุ่ม)
static int DRY_RAW = 3000;
static int WET_RAW = 1500;

static int clampi(int v, int lo, int hi){
    if(v < lo) return lo;
    if(v > hi) return hi;
    return v;
}

static int raw_to_percent(int raw){
    // ถ้า raw สูง = แห้ง (DRY_RAW)
    // ถ้า raw ต่ำ = เปียก (WET_RAW)
    int pct = (DRY_RAW - raw) * 100 / (DRY_RAW - WET_RAW);
    return clampi(pct, 0, 100);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Soil ADC test start (GPIO34 / ADC1_CH6).");

    // 1) init oneshot
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = SOIL_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_11,      // รับได้ถึง ~3.3V (เหมาะกับ ESP32)
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, SOIL_ADC_CHANNEL, &chan_cfg));

    // 2) init calibration (optional but nice)
    adc_cali_handle_t cali_handle = NULL;
    bool cali_ok = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = SOIL_ADC_UNIT,
        .chan = SOIL_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle) == ESP_OK) {
        cali_ok = true;
        ESP_LOGI(TAG, "ADC calibration: curve fitting enabled");
    }
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = SOIL_ADC_UNIT,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &cali_handle) == ESP_OK) {
        cali_ok = true;
        ESP_LOGI(TAG, "ADC calibration: line fitting enabled");
    }
#endif

    while (1) {
        int raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, SOIL_ADC_CHANNEL, &raw));

        int mv = -1;
        if (cali_ok) {
            adc_cali_raw_to_voltage(cali_handle, raw, &mv);
        }

        int pct = raw_to_percent(raw);

        if (cali_ok) {
            ESP_LOGI(TAG, "RAW=%4d  Voltage=%4d mV  Moisture=%3d%%", raw, mv, pct);
        } else {
            ESP_LOGI(TAG, "RAW=%4d  Moisture=%3d%% (no cali)", raw, pct);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
