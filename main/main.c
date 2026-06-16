/**
 * @file UWB_Init_Test.c
 * @brief DW1000 initialization test for ESP32-S3
 * 
 * This application tests the basic initialization sequence of the DW1000 UWB transceiver.
 * It configures SPI, GPIO, resets the device, loads microcode, and verifies the device ID.
 * Used to verify hardware connections and basic DW1000 functionality.
 */

// #include <Arduino.h>
// #include <HardwareDefs.hpp>
// #include <Blink.hpp>

// Include DW1000 driver
#include "deca_spi.h"
#include "deca_gpio.h"
#include "deca_device_api.h"
#include "deca_regs.h"

#include <stdio.h>
#include "hardware_defs.h"
#include "esp_log.h"

#include "esp_timer.h"
static const char *TAG = "MAIN";

/* Buffer to store received frame. See NOTE 1 below. */
#define FRAME_LEN_MAX 127



// /* DW1000 configuration - Channel 5, 850kbps, 16MHz PRF */
// static dwt_config_t dw1000_config = {
//     3,                /* Channel number. - 3 is known for this system */
//     DWT_PRF_16M,      /* Pulse repetition frequency. */
//     DWT_PLEN_256,     /* Preamble length. Used in TX only.  - */
//     DWT_PAC16,        /* Preamble acquisition chunk size. Used in RX only. */
//     3,                /* TX preamble code. Used in TX only. */
//     3,                /* RX preamble code. Used in RX only. */
//     0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
//     DWT_BR_850K,      /* Data rate. */
//     DWT_PHRMODE_STD,  /* PHY header mode. */
//     (256 + 1 + 8 - 8) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
// };

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

    // ----- 16MHz PRF -----
        // - 6.8M Data Rate 
            // --- Pcode 5
            {3, DWT_PRF_16M, DWT_PLEN_64, DWT_PAC8, 5, 5, 0, DWT_BR_6M8, DWT_PHRMODE_STD, (64 + 1 + 8 - 8)},
            {3, DWT_PRF_16M, DWT_PLEN_128, DWT_PAC8, 5, 5, 0, DWT_BR_6M8, DWT_PHRMODE_STD, (128 + 1 + 8 - 8)},
            {3, DWT_PRF_16M, DWT_PLEN_256, DWT_PAC16, 5, 5, 0, DWT_BR_6M8, DWT_PHRMODE_STD, (256 + 1 + 8 - 16)},
            // --- Pcode 6
            {3, DWT_PRF_16M, DWT_PLEN_64, DWT_PAC8, 6, 6, 0, DWT_BR_6M8, DWT_PHRMODE_STD, (64 + 1 + 8 - 8)},
            {3, DWT_PRF_16M, DWT_PLEN_128, DWT_PAC8, 6, 6, 0, DWT_BR_6M8, DWT_PHRMODE_STD, (128 + 1 + 8 - 8)},
            {3, DWT_PRF_16M, DWT_PLEN_256, DWT_PAC16, 6, 6, 0, DWT_BR_6M8, DWT_PHRMODE_STD, (256 + 1 + 8 - 16)},
        
        // - 850k Data Rate 
            // --- Pcode 5
            {3, DWT_PRF_16M, DWT_PLEN_256, DWT_PAC16, 5, 5, 0, DWT_BR_850K, DWT_PHRMODE_STD, (256 + 1 + 8 - 16)},
            {3, DWT_PRF_16M, DWT_PLEN_512, DWT_PAC16, 5, 5, 0, DWT_BR_850K, DWT_PHRMODE_STD, (512 + 1 + 8 - 16)},
            {3, DWT_PRF_16M, DWT_PLEN_1024, DWT_PAC32, 5, 5, 0, DWT_BR_850K, DWT_PHRMODE_STD, (1024 + 1 + 8 - 32)},
            // --- Pcode 6
            {3, DWT_PRF_16M, DWT_PLEN_256, DWT_PAC16, 6, 6, 0, DWT_BR_850K, DWT_PHRMODE_STD, (256 + 1 + 8 - 16)},
            {3, DWT_PRF_16M, DWT_PLEN_512, DWT_PAC16, 6, 6, 0, DWT_BR_850K, DWT_PHRMODE_STD, (512 + 1 + 8 - 16)},
            {3, DWT_PRF_16M, DWT_PLEN_1024, DWT_PAC32, 6, 6, 0, DWT_BR_850K, DWT_PHRMODE_STD, (1024 + 1 + 8 - 32)},

        // - 110k Data Rate 
            // --- Pcode 5
            {3, DWT_PRF_16M, DWT_PLEN_2048, DWT_PAC64, 5, 5, 0, DWT_BR_110K, DWT_PHRMODE_STD, (2048 + 1 + 8 - 64)},
            {3, DWT_PRF_16M, DWT_PLEN_4096, DWT_PAC64, 5, 5, 0, DWT_BR_110K, DWT_PHRMODE_STD, (4096 + 1 + 8 - 64)},
            
            // --- Pcode 6
            {3, DWT_PRF_16M, DWT_PLEN_2048, DWT_PAC64, 6, 6, 0, DWT_BR_110K, DWT_PHRMODE_STD, (2048 + 1 + 8 - 64)},
            {3, DWT_PRF_16M, DWT_PLEN_4096, DWT_PAC64, 6, 6, 0, DWT_BR_110K, DWT_PHRMODE_STD, (4096 + 1 + 8 - 64)},

    // 64MHz PRF (Pcode 9,10,11,12)
    // To be added/Tested later
    
};

/**
 * @brief Main Application
 */
void app_main(void)
{
    // /* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
    // static uint16 frame_len = 0;
    uint8_t NUM_CONFIGS = 16;

    // Include LED or Display blink to show active status

    /* Configure SPI bus */
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = (gpio_num_t)SPI_MOSI_PIN,
        .miso_io_num = (gpio_num_t)SPI_MISO_PIN,
        .sclk_io_num = (gpio_num_t)SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024,
        .flags = 0,
        .intr_flags = 0};

    /* Initialize DW1000 SPI */
    if (dw1000_spi_init(SPI2_HOST, (gpio_num_t)UWB_CS_PIN, &spi_bus_cfg) != 0)
    {
        ESP_LOGE(TAG,"ERROR: SPI init failed!");
        while (1);
    }

    /* Configure DW1000 GPIO */
    if (dw1000_gpio_init((gpio_num_t)UWB_RST_PIN, (gpio_num_t)UWB_IRQ_PIN, GPIO_NUM_NC) != 0)
    {
         ESP_LOGE(TAG,"ERROR: GPIO init failed!");
        while (1);
    }

    /* Reset DW1000 */
    dw1000_hard_reset();
    dw1000_spi_fix_bug(); // Apply SPI bug fix after reset
    ESP_LOGI(TAG,"DW1000 reset complete\n");

    /* Initialize DW1000 with microcode */
   printf("Calling dwt_initialise(DWT_LOADUCODE)...");
    int init_result = dwt_initialise(DWT_LOADUCODE);
    if (init_result == DWT_ERROR)
    {
        ESP_LOGE(TAG,"ERROR: dwt_initialise failed! : %d", init_result);
        while (1);
    }

    ESP_LOGI(TAG,"SUCCESS: dwt_initialise passed");

    /* Set SPI to high speed */
    spi_set_rate_high();
    printf("SPI speed set to high (16 MHz)");


    // ------ Sniffing Loop ------

     
    static uint32_t status_reg = 0;

    ESP_LOGI(TAG, "Starting Sniff");

    for(int i = 0; i < NUM_CONFIGS; i++)
    {
        dwt_configure(&non_dps_scan_matrix[i]);
        
        ESP_LOGI(TAG,"Configuration %d applied", i);

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_RXPRD | SYS_STATUS_ALL_RX_ERR); // Clear all RX status bits before starting reception
        ESP_LOGI(TAG, "Cleared RX Status Bits");

        /* Activate reception immediately. See NOTE 3 below. */
        int rx_enable_ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);
        if (rx_enable_ret != DWT_SUCCESS)
        {
            ESP_LOGE(TAG,"ERROR: dwt_rxenable failed with code %d\n", rx_enable_ret);
        }
        ESP_LOGI(TAG, "Reception Activated");

        int64_t t_start = esp_timer_get_time();

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_RXPRD)))
        {
            
            if (esp_timer_get_time() - t_start > 2000000) // 2 second timeout
            {
                ESP_LOGE(TAG, "Next Config: prev: %d\n", i);
                if (status_reg & SYS_STATUS_ALL_RX_ERR)
                    {
                        ESP_LOGI(TAG,
                            "Config %d error status = 0x%08lx",
                            i,
                            status_reg);
                            break;
                    }
                t_start = esp_timer_get_time();
                break;
                
            }
            vTaskDelay(1);
        };

        if (status_reg & SYS_STATUS_RXPRD)
            {
                ESP_LOGI(TAG, "PREAMBLE DETECTED: Iteration Number : %d", i);
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXPRD); // Clear preamble detect event
                break; // Move to next configuration after preamble detection
            }

        if (status_reg & SYS_STATUS_RXFCG)
        {

            ESP_LOGI(TAG, "DATA RECEIVED: Iteration Number : %d", i);

            /* Clear good RX frame event in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
        }
        else
        {
            // Serial.println("RX Error occurred");
            // /* Clear RX error events in the DW1000 status register. */
            // dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            // Serial.println("RX error events cleared");
        }
    }
    
}

