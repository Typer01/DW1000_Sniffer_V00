#include <stdio.h>
#include "SSD1306.h"
#include "i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

void SSD1306_init(i2c_master_dev_handle_t i2c_dev)
{
    // Initialize the SSD1306 display
    uint8_t init_sequence[] = { /** @todo Fill in the initialization sequence */
        0xAE, // Display off
        0xD5, 0x80, // Set display clock divide ratio/oscillator frequency
        0xA8, 0x3F, // Set multiplex ratio (1 to 64)
        0xD3, 0x00, // Set display offset
        0x40, // Set start line address
        0x8D, 0x14, // Charge pump setting (enable)
        0x20, 0x00, // Memory addressing mode (horizontal)
        0xA1, // Set segment re-map (column address 127 is mapped to SEG0)
        0xC8, // Set COM output scan direction (remapped mode)
        0xDA, 0x12, // Set COM pins hardware configuration
        0x81, 0xCF, // Set contrast control
        0xD9, 0xF1, // Set pre-charge period
        0xDB, 0x40, // Set VCOMH deselect level
        0xA4, // Entire display ON (resume to RAM content display)
        0xA6, // Set normal display (not inverted)
        0xAF // Display ON
    };

    i2c_write_byte(i2c_dev, init_sequence, sizeof(init_sequence), pdMS_TO_TICKS(100));
}

void SSD1306_clear(i2c_master_dev_handle_t i2c_dev)
{
    // Clear the display by writing zeros to the entire display RAM
    uint8_t clear_buffer[1024] = {0}; // Assuming a 128x64 display (8 pages of 128 bytes each)
    i2c_write_byte(i2c_dev, clear_buffer, 1024, pdMS_TO_TICKS(100));

    return;
}

void SSD1306_draw_pixel(i2c_master_dev_handle_t i2c_dev, uint8_t x, uint8_t y)
{
    // Draw a pixel at (x, y) by setting the corresponding bit in the display RAM
    if (x >= 128 || y >= 64) {
        ESP_LOGE("SSD1306", "Pixel coordinates out of bounds: (%d, %d)", x, y);
        return; // Out of bounds
    }

    uint8_t page = y / 8;
    uint8_t bit_position = y % 8;

    uint8_t data[2] = {0xB0 | page, 0x00 | x}; // Set page and column address

    // Read the current byte from the display RAM
    uint8_t current_byte;
    i2c_read_byte(i2c_dev, &current_byte, 1, pdMS_TO_TICKS(100));

    // Set the bit for the pixel
    current_byte |= (1 << bit_position);

    // Write back the modified byte to the display RAM
    i2c_write_byte(i2c_dev, data, sizeof(data), pdMS_TO_TICKS(100));
    i2c_write_byte(i2c_dev, &current_byte, 1, pdMS_TO_TICKS(100));

    return;
}
