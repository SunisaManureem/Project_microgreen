#include <stdio.h>
#include <inttypes.h>                   // ✅ ใช้สำหรับ PRIu32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include "driver/adc.h"

#define TAG "SOIL_TEST"

// ==== CONFIG ====
#define SOIL_CH        ADC1_CHANNEL_6   // GPIO34 → เปลี่ยนตามที่คุณต่อจริง
#define SAMPLE_DELAY   2000             // หน่วงอ่านทุก 2 วินาที
#define N_SAMPLES      16               // อ่านหลายครั้งแล้วเฉลี่ย

static esp_adc_cal_characteristics_t adc_chars;

// ===== ฟังก์ชันอ่านค่าเฉลี่ยจาก ADC แล้วแปลงเป็นแรงดัน (mV) =====
static uint32_t read_mv(void) {
    uint32_t acc = 0;
    for (int i = 0; i < N_SAMPLES; ++i) {
        acc += adc1_get_raw(SOIL_CH);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    uint32_t raw = acc / N_SAMPLES;
    return esp_adc_cal_raw_to_voltage(raw, &adc_chars);
}

void app_main(void) {
    // ===== ตั้งค่า ADC =====
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SOIL_CH, ADC_ATTEN_DB_12); // ✅ ใช้ DB_12 แทน DB_11
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12,
                             1100, &adc_chars);

    ESP_LOGI(TAG, "Start soil sensor test...");

    while (1) {
        uint32_t mv = read_mv();
        // ✅ แก้ format specifier ให้ตรงชนิดข้อมูล
        printf("Soil Sensor = %" PRIu32 " mV\n", mv);

        // ถ้าต้องการคำนวณความชื้นคร่าว ๆ (หลังคาลิเบรต)
         static int rawDry = 2900;
         static int rawWet = 1700;
         int pct = (mv > rawDry) ? 0 :
                   (mv < rawWet) ? 100 :
                   (rawDry - mv) * 100 / (rawDry - rawWet);
         printf("Soil = %" PRIu32 " mV → %d%%\n", mv, pct);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY));
    }
}
