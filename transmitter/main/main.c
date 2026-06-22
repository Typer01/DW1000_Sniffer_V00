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

static const char *TAG = "TX";

// Matches scan_matrix[1] in main/main.c: CH3, 64M PRF, PHR_EXT, PLEN1024/PAC32/850K
static dwt_config_t tx_config = {
    3,                       /* Channel number. */
    DWT_PRF_64M,             /* Pulse repetition frequency. */
    DWT_PLEN_1024,           /* Preamble length. */
    DWT_PAC32,               /* Preamble acquisition chunk size. */
    9,                       /* TX preamble code. */
    9,                       /* RX preamble code. */
    0,                       /* Standard SFD. */
    DWT_BR_850K,             /* Data rate. */
    DWT_PHRMODE_EXT,         /* PHY header mode. */
    (1024 + 1 + 8 - 32)      /* SFD timeout. */
};

/* Minimal classic IEEE 802.15.4 Data frame, with no addressing at all, so
 * it dissects cleanly as valid 802.15.4 in Wireshark with no ambiguity
 * about field lengths -- this validates the full RX -> uwb_live.py ->
 * Wireshark pipeline against a known-good standard frame, not just the
 * DW1000 PHY-layer preamble/CRC.
 *     - byte 0/1: Frame Control Field, little-endian 0x0001 -- Frame Type
 *       = Data, Security/Pending/AckReq/PANIDCompression = 0, Dest/Source
 *       Addressing Mode = None, Frame Version = 0. With both addressing
 *       modes set to None there is no PAN ID or address field at all, so
 *       the sequence number is immediately followed by the payload.
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3 -> 9: message content.
 *     - byte 10/11: frame check-sum, automatically set by DW1000. */
static uint8_t tx_msg[] = {0x01, 0x00, 0x00 /* seq */, 0xDE, 0xCA, 0x01, 0x23, 0x45, 0x67, 0x89, 0x00, 0x00 /* FCS */};
#define TX_MSG_SN_IDX 2

#define TX_DELAY_MS 200

/**
 * @brief Main Application
 */
void app_main(void)
{
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = (gpio_num_t)SPI_MOSI_PIN,
        .miso_io_num = (gpio_num_t)SPI_MISO_PIN,
        .sclk_io_num = (gpio_num_t)SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024,
        .flags = 0,
        .intr_flags = 0};

    if (dw1000_spi_init(SPI2_HOST, (gpio_num_t)UWB_CS_PIN, &spi_bus_cfg) != 0)
    {
        ESP_LOGE(TAG, "SPI init failed");
        while (1);
    }

    if (dw1000_gpio_init((gpio_num_t)UWB_RST_PIN, (gpio_num_t)UWB_IRQ_PIN, GPIO_NUM_NC) != 0)
    {
        ESP_LOGE(TAG, "GPIO init failed");
        while (1);
    }

    dw1000_hard_reset();
    dw1000_spi_fix_bug();

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        ESP_LOGE(TAG, "dwt_initialise failed");
        while (1);
    }

    spi_set_rate_high();

    dwt_configure(&tx_config);

    ESP_LOGI(TAG, "DW1000 Ready, chan=%u prf=%u plen=%u rate=%u phr=%u",
             tx_config.chan, tx_config.prf, tx_config.txPreambLength, tx_config.dataRate, tx_config.phrMode);

    while (1)
    {
        int write_ret = dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
        if (write_ret != DWT_SUCCESS)
        {
            ESP_LOGE(TAG, "dwt_writetxdata failed with code %d", write_ret);
        }

        dwt_writetxfctrl(sizeof(tx_msg), 0, 0); /* Zero offset in TX buffer, no ranging. */

        int tx_ret = dwt_starttx(DWT_START_TX_IMMEDIATE);
        if (tx_ret != DWT_SUCCESS)
        {
            ESP_LOGE(TAG, "dwt_starttx failed with code %d", tx_ret);
        }

        uint32_t status_reg = 0;
        int64_t poll_start = esp_timer_get_time();
        bool tx_done = false;

        while ((esp_timer_get_time() - poll_start) < 50000) /* 50ms timeout */
        {
            status_reg = dwt_read32bitreg(SYS_STATUS_ID);
            if (status_reg & SYS_STATUS_TXFRS)
            {
                tx_done = true;
                break;
            }
        }

        if (!tx_done)
        {
            ESP_LOGE(TAG, "TX TIMEOUT waiting for TXFRS, SYS_STATUS=0x%08" PRIx32, status_reg);
            dwt_forcetrxoff();
        }
        else
        {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
            ESP_LOGI(TAG, "TX seq=%u SYS_STATUS=0x%08" PRIx32, tx_msg[TX_MSG_SN_IDX], status_reg);
        }

        vTaskDelay(pdMS_TO_TICKS(TX_DELAY_MS));

        tx_msg[TX_MSG_SN_IDX]++;
    }
}
