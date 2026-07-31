/**
 * @file i2c.c
 * @brief I2C master bus and device helper functions built on top of the
 *        ESP-IDF `driver/i2c_master.h` API.
 */
#include "i2c.h"
#include "esp_err.h"
#include "esp_log.h"

/**
 * @brief Add and configure an I2C device on an already-initialized bus.
 */
esp_err_t i2c_device_initialization(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle, i2c_addr_bit_len_t addr_len, uint8_t device_address, uint32_t scl_speed_hz)
{
    esp_err_t ret = ESP_OK;
    i2c_device_config_t dev_config = {
        .dev_addr_length = addr_len,
        .device_address = device_address,
        .scl_speed_hz = scl_speed_hz,
    };

    ret = i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle);

    ESP_LOGD(TAG, "I2C device initialized with address 0x%02X, address length %d bits, SCL speed %d Hz", device_address, addr_len == I2C_ADDR_BIT_LEN_7 ? 7 : 10, scl_speed_hz);

    return ret;
}

/**
 * @brief Initialize an I2C master bus on the given SDA/SCL pins.
 */
esp_err_t i2c_bus_initialize(i2c_master_bus_handle_t *bus_handle, gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed_hz)
{
    esp_err_t ret = ESP_OK;
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT, /** @todo Check what this is */
        .i2c_port = I2C_NUM_0, /** @todo Check what this is */
        .scl_io_num = scl_pin,
        .sda_io_num = sda_pin,
        .glitch_ignore_cnt = 7, /** @todo Check what this is */
    };

    ret = i2c_new_master_bus(&i2c_mst_config, bus_handle);

    ESP_LOGI(TAG, "I2C bus initialized with SDA pin %d, SCL pin %d, clock speed %d Hz", sda_pin, scl_pin, clk_speed_hz);

    return ret;
}

/**
 * @brief Write a buffer of bytes to an I2C device.
 */
esp_err_t i2c_write_byte(i2c_master_dev_handle_t i2c_dev, const uint8_t *write_buffer, size_t write_size, int xfer_timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ret = i2c_master_transmit(i2c_dev, write_buffer, write_size, xfer_timeout_ms);
    return ret;
}

/**
 * @brief Read a buffer of bytes from an I2C device.
 */
esp_err_t i2c_read_byte(i2c_master_dev_handle_t i2c_dev, uint8_t *read_buffer, size_t read_size, int xfer_timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ret = i2c_master_receive(i2c_dev, read_buffer, read_size, xfer_timeout_ms);
    return ret;
}

/**
 * @brief Write a buffer to an I2C device, then read a response back (e.g. register read).
 */
esp_err_t i2c_write_read(i2c_master_dev_handle_t i2c_dev, const uint8_t *write_buffer, size_t write_size, uint8_t *read_buffer, size_t read_size, int xfer_timeout_ms)
{
    esp_err_t ret = ESP_OK;
    ret = i2c_master_transmit(i2c_dev, write_buffer, write_size, xfer_timeout_ms);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_master_receive(i2c_dev, read_buffer, read_size, xfer_timeout_ms);
    return ret;
}