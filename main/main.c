/**
 * @file UWB_Init_Test.c
 * @brief DW1000 initialization test for ESP32-S3
 * 
 * This application tests the basic initialization sequence of the DW1000 UWB transceiver.
 * It configures SPI, GPIO, resets the device, loads microcode, and verifies the device ID.
 * Used to verify hardware connections and basic DW1000 functionality.
 */

// Include DW1000 driver
#include "deca_spi.h"
#include "deca_gpio.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include <math.h>

#include <stdio.h>
#include "hardware_defs.h"
#include "esp_log.h"

#include "esp_timer.h"
static const char *TAG = "MAIN";

/* Buffer to store received frame. See NOTE 1 below. */
#define FRAME_LEN_MAX 127

// Wall-clock time spent listening on each config before moving to the next.
#define CONFIG_DWELL_US (5 * 1000000)

    // Data Rate to Reccomended PLEN
    // 6.8M - 64 or 128 or 256
    // 850k - 256 or 512 or 1024
    // 110k - 2048 or 4096

    // Reccomended PAC size Expected PLEN - PAC Size
    // 64 - 8
    // 128 - 8
    // 256 - 16
    // 512 - 16
    // 1024 - 32
    // 1536 - 64
    // 2048 - 64
    // 4096 - 64

static dwt_config_t scan_matrix[] = {
    // CH3, 64M PRF, PHR_EXT — confirmed hits with real data during the full sweep
    {3, DWT_PRF_64M, DWT_PLEN_2048, DWT_PAC64, 9, 9, 0, DWT_BR_110K, DWT_PHRMODE_EXT, (2048 + 1 + 8 - 64)},
    {3, DWT_PRF_64M, DWT_PLEN_1024, DWT_PAC32, 9, 9, 0, DWT_BR_850K, DWT_PHRMODE_EXT, (1024 + 1 + 8 - 32)},
};
#define NUM_CONFIGS (sizeof(scan_matrix) / sizeof(scan_matrix[0]))

/**
 * @brief Main Application
 */
void app_main(void)
{
    uint8_t rx_buffer[FRAME_LEN_MAX];

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

    if (dw1000_gpio_init((gpio_num_t)UWB_RST_PIN,(gpio_num_t)UWB_IRQ_PIN,GPIO_NUM_NC) != 0)
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

    ESP_LOGI(TAG, "DW1000 Ready");

    while (1)
    {
        for (size_t cfg_idx = 0; cfg_idx < NUM_CONFIGS; cfg_idx++)
        {
            dwt_config_t *cfg = &scan_matrix[cfg_idx];

            dwt_forcetrxoff();
            dwt_configure(cfg);

            ESP_LOGI(TAG, "=== Config %u/%u: prf=%u plen=%u pac=%u rate=%u phr=%u nsSFD=%u ===",
                     (unsigned)(cfg_idx + 1), (unsigned)NUM_CONFIGS,
                     cfg->prf, cfg->txPreambLength, cfg->rxPAC, cfg->dataRate, cfg->phrMode, cfg->nsSFD);

            uint32_t preamble_count = 0, sfd_count = 0, good_count = 0;
            uint32_t phe_count = 0, fce_count = 0, rfsl_count = 0, hw_to_count = 0;

            bool preamble_seen = false;
            bool sfd_seen = false;

            // Reset status flags
            dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_RXFCG |SYS_STATUS_RXFCE |SYS_STATUS_RXPHE |SYS_STATUS_RXRFSL |SYS_STATUS_RXRFTO |SYS_STATUS_RXPRD |SYS_STATUS_RXSFDD);

            // Enable RX immediately, since the device is in idle mode after initialization
            int rx_ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);

            if (rx_ret != DWT_SUCCESS)
            {
                ESP_LOGE(TAG, "RX enable failed");
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            int64_t config_start = esp_timer_get_time();

            while ((esp_timer_get_time() - config_start) < CONFIG_DWELL_US)
            {
                uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);

                if ((status_reg & SYS_STATUS_RXPRD) && !preamble_seen)
                {
                    ESP_LOGI(TAG, "PREAMBLE DETECTED");
                    preamble_seen = true;
                    preamble_count++;
                }

                if ((status_reg & SYS_STATUS_RXSFDD) && !sfd_seen)
                {
                    ESP_LOGI(TAG, "SFD DETECTED");
                    sfd_seen = true;
                    sfd_count++;
                }

                if (status_reg & SYS_STATUS_RXFCG)
                {
                    good_count++;

                    uint32_t rx_finfo = dwt_read32bitreg(RX_FINFO_ID);
                    uint32_t frame_len = rx_finfo & RX_FINFO_RXFL_MASK_1023;

                    ESP_LOGI(TAG, "GOOD FRAME RECEIVED, LEN=%lu, SYS_STATUS=0x%08" PRIx32, frame_len, status_reg);
                    ESP_LOGI(TAG, "RX_FINFO=0x%08" PRIx32, rx_finfo);

                    dwt_rxdiag_t rx_diag;
                    dwt_readdiagnostics(&rx_diag);
                    ESP_LOGI(TAG, "RX_FQUAL: stdNoise=%u maxNoise=%u firstPath=%u firstPathAmp1=%u firstPathAmp2=%u firstPathAmp3=%u maxGrowthCIR=%u rxPreamCount=%u",
                             rx_diag.stdNoise, rx_diag.maxNoise, rx_diag.firstPath,
                             rx_diag.firstPathAmp1, rx_diag.firstPathAmp2, rx_diag.firstPathAmp3,
                             rx_diag.maxGrowthCIR, rx_diag.rxPreamCount);

                    if (frame_len > 0 && frame_len <= FRAME_LEN_MAX)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);

                        for (uint32_t i = 0; i < frame_len; i++)
                        {
                            printf("%02X ", rx_buffer[i]);
                        }

                        printf("\n");
                    }

                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

                    preamble_seen = false;
                    sfd_seen = false;
                    dwt_rxenable(DWT_START_RX_IMMEDIATE);

                    continue;
                }

                if (status_reg & SYS_STATUS_RXPHE)
                {
                    ESP_LOGW(TAG,"PHY HEADER ERROR  SYS_STATUS=0x%08" PRIx32,status_reg);

                    phe_count++;

                    dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_RXPHE |SYS_STATUS_RXSFDD | SYS_STATUS_RXPRD);

                    dwt_rxreset();

                    preamble_seen = false;
                    sfd_seen = false;

                    dwt_rxenable(DWT_START_RX_IMMEDIATE);

                    vTaskDelay(pdMS_TO_TICKS(1));

                    continue;
                }

                if (status_reg & SYS_STATUS_RXFCE)
                {
                    ESP_LOGW(TAG,"FCS ERROR SYS_STATUS=0x%08" PRIx32,status_reg);

                    fce_count++;

                    dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_RXFCE |SYS_STATUS_RXSFDD |SYS_STATUS_RXPRD);

                    dwt_rxreset();

                    preamble_seen = false;
                    sfd_seen = false;

                    dwt_rxenable(DWT_START_RX_IMMEDIATE);

                    vTaskDelay(pdMS_TO_TICKS(1));

                    continue;
                }

                if (status_reg & SYS_STATUS_RXRFSL)
                {
                    ESP_LOGW(TAG,"REED SOLOMON ERROR SYS_STATUS=0x%08" PRIx32,status_reg);

                    rfsl_count++;

                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFSL |SYS_STATUS_RXSFDD |SYS_STATUS_RXPRD);

                    dwt_rxreset();

                    preamble_seen = false;
                    sfd_seen = false;

                    dwt_rxenable(DWT_START_RX_IMMEDIATE);

                    vTaskDelay(pdMS_TO_TICKS(1));

                    continue;
                }

                if (status_reg & SYS_STATUS_RXRFTO)
                {
                    ESP_LOGW(TAG,"FRAME WAIT TIMEOUT");

                    hw_to_count++;

                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFTO);

                    dwt_rxreset();

                    preamble_seen = false;
                    sfd_seen = false;

                    dwt_rxenable(DWT_START_RX_IMMEDIATE);

                    continue;
                }

                vTaskDelay(pdMS_TO_TICKS(5));
            }

            ESP_LOGI(TAG, "Config %u summary: preamble=%lu sfd=%lu good=%lu phe=%lu fce=%lu rfsl=%lu hw_to=%lu",
                     (unsigned)(cfg_idx + 1), preamble_count, sfd_count, good_count, phe_count, fce_count, rfsl_count, hw_to_count);
        }

        ESP_LOGI(TAG, "=== Sweep complete, restarting from config 0 ===");
    }
}