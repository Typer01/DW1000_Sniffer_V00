/**
 * @file main.c
 * @brief DW1000 transmitter for ESP32, used to validate the DW1000_Sniffer_V00 receiver
 *
 * Continuously transmits a standard IEEE 802.15.4 "Blink" frame using the
 * same channel/PRF/preamble settings as the receiver's rx_config (see
 * ../../main/main.c), so the full receive pipeline -- DW1000 PHY, the
 * receiver firmware, uwb_live.py, and Wireshark's 802.15.4 dissector -- can
 * all be validated end-to-end against a known-good standard frame.
 */

// Include DW1000 driver
#include "deca_spi.h"
#include "deca_gpio.h"
#include "deca_device_api.h"
#include "deca_regs.h"

#include <stdio.h>
#include "hardware_defs.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"
#include "i2c.h"
#include "SSD1306.h"

#define I2C_CLK_SPEED_HZ 400000 // SSD1306 max Fast-mode I2C clock

/**
 * @brief Main Application
 */
void app_main(void)
{
    ESP_LOGI("MAIN", "Starting Display Test");

    i2c_master_bus_handle_t i2c_bus;
    i2c_master_dev_handle_t i2c_dev = NULL;

    esp_err_t err = i2c_bus_initialize(&i2c_bus, I2C_SDA, I2C_SCL, I2C_CLK_SPEED_HZ);
    if (err != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize I2C bus");
        return;
    }
    esp_err_t dev_err = i2c_device_initialization(&i2c_bus, &i2c_dev, I2C_ADDR_BIT_LEN_7, 0x3C, I2C_CLK_SPEED_HZ);
    if (dev_err != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize I2C device");
        return;
    }

    if (SSD1306_init(i2c_dev) != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize SSD1306");
        return;
    }

    SSD1306_clear(i2c_dev);
    vTaskDelay(pdMS_TO_TICKS(500));
    SSD1306_draw_pixel(10, 10);
    SSD1306_update(i2c_dev);
    vTaskDelay(pdMS_TO_TICKS(500));
    SSD1306_draw_pixel(20, 20);
    SSD1306_update(i2c_dev);
    vTaskDelay(pdMS_TO_TICKS(500));
    SSD1306_draw_pixel(30, 30);
    SSD1306_update(i2c_dev);

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
