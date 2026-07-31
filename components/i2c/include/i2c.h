/**
 * @file i2c.h
 * @brief I2C master bus and device helper functions built on top of the
 *        ESP-IDF `driver/i2c_master.h` API.
 */
#ifndef i2c_h
#define i2c_h

#include "driver/i2c_master.h"
#include "esp_err.h"

/**
 * @brief Add and configure an I2C device on an already-initialized bus.
 *
 * @param bus_handle Pointer to the master bus handle returned by i2c_bus_initialize().
 * @param dev_handle Pointer to the device handle to populate on success.
 * @param addr_len Length of the device's I2C address (7-bit or 10-bit).
 * @param device_address I2C address of the device.
 * @param scl_speed_hz SCL clock speed to use for this device, in Hz.
 * @return ESP_OK on success, or an esp_err_t error code from i2c_master_bus_add_device() on failure.
 */
esp_err_t i2c_device_initialization(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle, i2c_addr_bit_len_t addr_len, uint8_t device_address, uint32_t scl_speed_hz);

/**
 * @brief Initialize an I2C master bus on the given SDA/SCL pins.
 *
 * @param bus_handle Pointer to the master bus handle to populate on success.
 * @param sda_pin GPIO number to use for the SDA line.
 * @param scl_pin GPIO number to use for the SCL line.
 * @param clk_speed_hz Bus clock speed, in Hz.
 * @return ESP_OK on success, or an esp_err_t error code from i2c_new_master_bus() on failure.
 */
esp_err_t i2c_bus_initialize(i2c_master_bus_handle_t *bus_handle, gpio_num_t sda_pin, gpio_num_t scl_pin, uint32_t clk_speed_hz);

/**
 * @brief Write a buffer of bytes to an I2C device.
 *
 * @param i2c_dev Handle of the target I2C device.
 * @param write_buffer Buffer containing the bytes to write.
 * @param write_size Number of bytes to write from write_buffer.
 * @param xfer_timeout_ms Transfer timeout, in milliseconds.
 * @return ESP_OK on success, or an esp_err_t error code from i2c_master_transmit() on failure.
 */
esp_err_t i2c_write_byte(i2c_master_dev_handle_t i2c_dev, const uint8_t *write_buffer, size_t write_size, int xfer_timeout_ms);

/**
 * @brief Read a buffer of bytes from an I2C device.
 *
 * @param i2c_dev Handle of the target I2C device.
 * @param read_buffer Buffer to receive the read bytes.
 * @param read_size Number of bytes to read into read_buffer.
 * @param xfer_timeout_ms Transfer timeout, in milliseconds.
 * @return ESP_OK on success, or an esp_err_t error code from i2c_master_receive() on failure.
 */
esp_err_t i2c_read_byte(i2c_master_dev_handle_t i2c_dev, uint8_t *read_buffer, size_t read_size, int xfer_timeout_ms);

/**
 * @brief Write a buffer to an I2C device, then read a response back (e.g. register read).
 *
 * Performs a transmit followed by a receive. If the transmit fails, the
 * receive is skipped and the transmit error is returned.
 *
 * @param i2c_dev Handle of the target I2C device.
 * @param write_buffer Buffer containing the bytes to write.
 * @param write_size Number of bytes to write from write_buffer.
 * @param read_buffer Buffer to receive the read bytes.
 * @param read_size Number of bytes to read into read_buffer.
 * @param xfer_timeout_ms Transfer timeout, in milliseconds, applied to both the write and read phases.
 * @return ESP_OK on success, or an esp_err_t error code from i2c_master_transmit() or i2c_master_receive() on failure.
 */
esp_err_t i2c_write_read(i2c_master_dev_handle_t i2c_dev, const uint8_t *write_buffer, size_t write_size, uint8_t *read_buffer, size_t read_size, int xfer_timeout_ms);
#endif /* i2c_h */