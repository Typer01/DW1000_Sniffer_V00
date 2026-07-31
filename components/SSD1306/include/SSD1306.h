#ifndef SSD1306_H
#define SSD1306_H

#include "i2c.h"

void SSD1306_init(i2c_master_dev_handle_t i2c_dev);

void SSD1306_clear(i2c_master_dev_handle_t i2c_dev);

void SSD1306_draw_pixel(i2c_master_dev_handle_t i2c_dev, uint8_t x, uint8_t y);


#endif // SSD1306_H

