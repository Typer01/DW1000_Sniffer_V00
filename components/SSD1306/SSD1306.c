#include <stdio.h>
#include <string.h>
#include "SSD1306.h"
#include "i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "SSD1306";

#define SSD1306_XFER_TIMEOUT_MS 100

// Local copy of display RAM. draw_pixel only modifies this buffer; call
// SSD1306_update() to push it out over I2C.
static uint8_t framebuffer[SSD1306_PAGES][SSD1306_WIDTH];

esp_err_t SSD1306_init(i2c_master_dev_handle_t i2c_dev)
{
    // Initialize the SSD1306 display
    uint8_t init_sequence[] = {
        0x00, // Control byte: command stream follows
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

    esp_err_t ret = i2c_write_byte(i2c_dev, init_sequence, sizeof(init_sequence), SSD1306_XFER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send init sequence: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t SSD1306_clear(i2c_master_dev_handle_t i2c_dev)
{
    memset(framebuffer, 0, sizeof(framebuffer));
    return SSD1306_update(i2c_dev);
}

void SSD1306_draw_pixel(uint8_t x, uint8_t y)
{
    // Draw a pixel at (x, y) by setting the corresponding bit in the local framebuffer
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        ESP_LOGE(TAG, "Pixel coordinates out of bounds: (%d, %d)", x, y);
        return;
    }

    uint8_t page = y / 8;
    uint8_t bit_position = y % 8;

    framebuffer[page][x] |= (1 << bit_position);
}

esp_err_t SSD1306_update(i2c_master_dev_handle_t i2c_dev)
{
    // Point the controller's addressing window at the full display before
    // streaming the framebuffer (horizontal addressing mode, matches SSD1306_init).
    uint8_t addr_cmd[] = {
        0x00, // Control byte: command stream follows
        0x21, 0x00, SSD1306_WIDTH - 1,  // Set column address range
        0x22, 0x00, SSD1306_PAGES - 1,  // Set page address range
    };

    esp_err_t ret = i2c_write_byte(i2c_dev, addr_cmd, sizeof(addr_cmd), SSD1306_XFER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set addressing window: %s", esp_err_to_name(ret));
        return ret;
    }

    static uint8_t tx_buffer[1 + sizeof(framebuffer)];
    tx_buffer[0] = 0x40; // Control byte: data stream follows
    memcpy(&tx_buffer[1], framebuffer, sizeof(framebuffer));

    ret = i2c_write_byte(i2c_dev, tx_buffer, sizeof(tx_buffer), SSD1306_XFER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write framebuffer: %s", esp_err_to_name(ret));
    }
    return ret;
}
