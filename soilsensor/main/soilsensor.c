#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define TAG               "SOIL_TEST"
#define SOIL_CH           ADC1_CHANNEL_6   // GPIO34
#define SAMPLE_MS         500         // อ่านทุก 10 วินาที
#define N_SAMPLES         16

// ==== ค่าอ้างอิง (ปรับตามของจริง) ====
#define MV_DRY  400     // ค่าต่ำสุดเมื่อดินแห้ง
#define MV_WET  2500    // ค่าสูงสุดเมื่อดินเปียก

static esp_adc_cal_characteristics_t adc_chars;

// อ่านค่าแรงดันเฉลี่ยจาก ADC
static uint32_t read_mv(void) {
    uint32_t acc = 0;
    for (int i = 0; i < N_SAMPLES; ++i)
        acc += adc1_get_raw(SOIL_CH);
    uint32_t raw = acc / N_SAMPLES;
    return esp_adc_cal_raw_to_voltage(raw, &adc_chars);
}

// แปลงค่า mV → เปอร์เซ็นต์ความชื้น (0–100%)
static int mv_to_percent(uint32_t mv) {
    int dry = MV_DRY, wet = MV_WET;
    if (wet <= dry) wet = dry + 1;
    int pct = (int)(((long)(mv - dry) * 100) / (wet - dry));
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return pct;
}

void app_main(void) {
    // ตั้งค่า ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SOIL_CH, ADC_ATTEN_DB_11);

    esp_adc_cal_value_t cal = esp_adc_cal_characterize(
        ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars
    );
    ESP_LOGI(TAG, "Cal: %s",
        cal == ESP_ADC_CAL_VAL_EFUSE_TP   ? "eFuse TwoPoint" :
        cal == ESP_ADC_CAL_VAL_EFUSE_VREF ? "eFuse Vref"     :
                                            "Default 1100mV");

    // ลูปอ่านค่าทุก 10 วิ
    while (1) {
        uint32_t mv = read_mv();
        int pct = mv_to_percent(mv);

        const char* status =
            (pct < 30) ? "DRY" :
            (pct < 70) ? "MOIST" : "WET";

        ESP_LOGI(TAG, "Soil: %4u mV | %3d%% | %s", mv, pct, status);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));  // หน่วง 10 วิ
    }
}
