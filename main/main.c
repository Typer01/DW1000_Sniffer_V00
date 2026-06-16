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

static dwt_config_t non_dps_scan_matrix[] = {
    // This matrix consists of all non-dps combinations of preamble parameters. Additionally, this currently assumes that a standard SFD mode is used
        // - 110k Data Rate 
            // --- Pcode 5
            {3, DWT_PRF_16M, DWT_PLEN_2048, DWT_PAC64, 5, 5, 0, DWT_BR_110K, DWT_PHRMODE_STD, (2048 + 1 + 8 - 64)}, // adjusting for test, 110K, EXT gets most hits
    // 850k not hitting much at start of TT
};

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

    const int cfg_idx = 0;

    dwt_configure(&non_dps_scan_matrix[cfg_idx]);

    ESP_LOGI(TAG, "Using configuration %d", cfg_idx);

    while (1)
    {
        uint32_t status_reg = 0;
        uint32_t RX_INFO_reg = 0;

        bool preamble_seen = false;
        bool sfd_seen = false;

        dwt_forcetrxoff();

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

        int64_t t_start = esp_timer_get_time();

        while (1)
        {
            status_reg = dwt_read32bitreg(SYS_STATUS_ID);

            if ((status_reg & SYS_STATUS_RXPRD) && !preamble_seen)
            {
                ESP_LOGI(TAG, "PREAMBLE DETECTED");
                preamble_seen = true;
            }

            if ((status_reg & SYS_STATUS_RXSFDD) && !sfd_seen)
            {
                ESP_LOGI(TAG, "SFD DETECTED");
                sfd_seen = true;
            }

            if (status_reg & SYS_STATUS_RXFCG)
            {
                // RX_INFO_reg = dwt_read32bitreg(RX_FQUAL_ID);
                // ESP_LOGI(TAG, "RX_FQUAL = 0x%08" PRIx32,RX_INFO_reg);

                // uint32 cir_pwr;
                // uint16 rxpacc;

                // cir_pwr = dwt_read32bitoffsetreg(RX_FQUAL_ID,
                //                                 0x06); // 0x06 is the offset for CIR_PWR in the RX_FQUAL register bank

                // rxpacc = dwt_read16bitoffsetreg(RX_FINFO_ID,
                //                                 0x02); // 0x02 is the offset for RXPACC in the RX_FINFO register bank


                // ESP_LOGI(TAG,"RX_PWR = %d", cir_pwr);
                // ESP_LOGI(TAG,"RX_PACC = %d", rxpacc);
                // // ESP_LOGI(TAG,"CALULCATED POWER: %d dbm", (int)(10 * log10(cir_pwr *(131072)/ (rxpacc * rxpacc))));

                ESP_LOGI(TAG, "GOOD FRAME RECEIVED");

                uint32_t finfo = dwt_read32bitreg(RX_FINFO_ID);
            

                ESP_LOGI(TAG,"SYS_STATUS = 0x%08" PRIx32,status_reg);

                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

                break;
            }

            if (status_reg & SYS_STATUS_RXPHE)
            {
                ESP_LOGW(TAG,"PHY HEADER ERROR  SYS_STATUS=0x%08" PRIx32,status_reg);

                // RX_INFO_reg = dwt_read32bitreg(RX_FINFO_ID);
                // ESP_LOGI(TAG, "RX_FINFO = 0x%08" PRIx32,RX_INFO_reg);

                dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_RXPHE |SYS_STATUS_RXSFDD | SYS_STATUS_RXPRD);

                dwt_rxreset();

                // CHANGE #3
                preamble_seen = false;
                sfd_seen = false;

                dwt_rxenable(DWT_START_RX_IMMEDIATE);

                // CHANGE #2
                vTaskDelay(pdMS_TO_TICKS(1));

                continue;
            }

            if (status_reg & SYS_STATUS_RXFCE)
            {
                ESP_LOGW(TAG,"FCS ERROR SYS_STATUS=0x%08" PRIx32,status_reg);

                // CHANGE #1
                dwt_write32bitreg(SYS_STATUS_ID,SYS_STATUS_RXFCE |SYS_STATUS_RXSFDD |SYS_STATUS_RXPRD);

                dwt_rxreset();

                // CHANGE #3
                preamble_seen = false;
                sfd_seen = false;

                dwt_rxenable(DWT_START_RX_IMMEDIATE);

                // CHANGE #2
                vTaskDelay(pdMS_TO_TICKS(1));

                continue;
            }

            if (status_reg & SYS_STATUS_RXRFSL)
            {
                ESP_LOGW(TAG,"REED SOLOMON ERROR SYS_STATUS=0x%08" PRIx32,status_reg);

                // RX_INFO_reg = dwt_read32bitreg(RX_FINFO_ID);
                // ESP_LOGI(TAG, "RX_FINFO = 0x%08" PRIx32,RX_INFO_reg);

                // CHANGE #1
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFSL |SYS_STATUS_RXSFDD |SYS_STATUS_RXPRD);

                dwt_rxreset();

                // CHANGE #3
                preamble_seen = false;
                sfd_seen = false;

                dwt_rxenable(DWT_START_RX_IMMEDIATE);

                // CHANGE #2
                vTaskDelay(pdMS_TO_TICKS(1));

                continue;
            }

            if (status_reg & SYS_STATUS_RXRFTO)
            {
                ESP_LOGW(TAG,"FRAME WAIT TIMEOUT");

                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFTO);

                dwt_rxreset();

                break;
            }


            if (status_reg & SYS_STATUS_RXFCG)
            {
                uint32_t frame_len =
                    dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;

                ESP_LOGI(TAG, "GOOD FRAME RECEIVED");
                ESP_LOGI(TAG, "FRAME LENGTH = %lu", frame_len);

                if (frame_len <= FRAME_LEN_MAX)
                {
                    dwt_readrxdata(rx_buffer, frame_len, 0);

                    ESP_LOGI(TAG, "FRAME DATA:");

                    for (uint32_t i = 0; i < frame_len; i++)
                    {
                        printf("%02X ", rx_buffer[i]);
                    }

                    printf("\n");
                }

                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

                break;
            }

            if ((esp_timer_get_time() - t_start) > 5000000)
            {
                ESP_LOGI(TAG, "LISTEN TIMEOUT");
                
                // RX_INFO_reg = dwt_read32bitreg(RX_FINFO_ID);
                // ESP_LOGI(TAG, "RX_FINFO = 0x%08" PRIx32,RX_INFO_reg);

                dwt_forcetrxoff();

                // CHANGE #4
                vTaskDelay(pdMS_TO_TICKS(10));

                break;
            }

            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
}