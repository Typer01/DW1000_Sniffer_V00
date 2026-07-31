#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include "i2c.h"

#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64
#define SSD1306_PAGES  (SSD1306_HEIGHT / 8)

esp_err_t SSD1306_init(i2c_master_dev_handle_t i2c_dev);

esp_err_t SSD1306_clear(i2c_master_dev_handle_t i2c_dev);

void SSD1306_draw_pixel(uint8_t x, uint8_t y);

esp_err_t SSD1306_update(i2c_master_dev_handle_t i2c_dev);

#endif // SSD1306_H
