#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

/* ================= USER CONFIG =================
 * ต่อ IN1 ของรีเลย์ -> GPIO26
 * ต่อ IN2 ของรีเลย์ -> GPIO27
 * โหลดฝั่งคอนแทครีเลย์: ใช้ COM+NO (ปกติปิด)
 * โหมด Active HIGH = เขียน HIGH -> รีเลย์ทำงาน (อุปกรณ์ ON)
 * ============================================== */
#define RELAY_PUMP_GPIO  GPIO_NUM_26   // พัดลม
#define RELAY_FAN_GPIO   GPIO_NUM_27   // ปั๊ม
#define TAG "RELAY2CH_AH"

/* เขียนสถานะรีเลย์แบบ Active HIGH */
static inline void relay_write(gpio_num_t pin, bool turn_on) {
    gpio_set_level(pin, turn_on ? 1 : 0);
}

void app_main(void)
{
    // ตั้งค่า GPIO เป็นเอาต์พุต และตั้งค่าเริ่ม "ปิด" ทั้งสองช่อง
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<RELAY_PUMP_GPIO) | (1ULL<<RELAY_FAN_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    relay_write(RELAY_PUMP_GPIO, false);  // LOW = OFF (Active HIGH)
    relay_write(RELAY_FAN_GPIO,  false);
    ESP_LOGI(TAG, "Relay test ready (Active HIGH). Pump/Fan OFF.");

    while (1) {
        ESP_LOGI(TAG, "Pump ON, Fan ON");
        relay_write(RELAY_PUMP_GPIO, true);   // HIGH = ON
        relay_write(RELAY_FAN_GPIO,  true);
        vTaskDelay(pdMS_TO_TICKS(5000));

        ESP_LOGI(TAG, "Pump OFF, Fan OFF");
        relay_write(RELAY_PUMP_GPIO, false);  // LOW = OFF
        relay_write(RELAY_FAN_GPIO,  false);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}