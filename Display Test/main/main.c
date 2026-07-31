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

#define DATA_LENGTH 4;

/**
 * @brief Main Application
 */
void app_main(void)
{
    
        
    
}
