/**
 * @file main.c
 * @brief DW1000 receiver for ESP32 — continuous single-config receive with rich RX diagnostics
 *
 * Listens on a single confirmed-working DW1000 config and logs detailed
 * signal diagnostics (noise/amplitude, frequency offset, RX timestamp,
 * decoded RX_FINFO fields) for every reception, including near-misses
 * (FCS/PHY header errors), to help characterize an unknown transmitter.
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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"

static const char *TAG = "MAIN";

/* Buffer to store received frame. Sized for DWT_PHRMODE_EXT (up to 1023 bytes). */
#define FRAME_LEN_MAX 1023

typedef enum {
    RX_EVENT_VALID,
    RX_EVENT_PHY_ERROR,
    RX_EVENT_RS_ERROR   // Reed-Solomon
} rx_event_type_t;

typedef struct {
    rx_event_type_t type;
    uint32_t timestamp;

    // Only meaningful when type == RX_EVENT_VALID
    uint8_t payload[FRAME_LEN_MAX];  // Maximum payload size for DWT_PHRMODE_EXT
    size_t   payload_len;
    uint32_t rx_qual;     // verify actual register name/width in DW1000 UM
    uint32_t rx_info;     // same — confirm exact register before using
} rx_event_t;

/** @todo this the right place to declare this? Or should it be in a header file? */
QueueHandle_t xRxEventQueue;
xRxEventQueue = xQueueCreate(20, sizeof(rx_event_t));


// CH3, 64M PRF, PHR_EXT, PLEN1024/PAC32/850K - Confirmed working config.
static const dwt_config_t rx_config = {3, DWT_PRF_64M, DWT_PLEN_1024, DWT_PAC32, 9, 9, 0, DWT_BR_850K, DWT_PHRMODE_EXT, (1024 + 1 + 8 - 32)};


/** @todo Review this code
 * Logs RX signal diagnostics common to any resolved reception (good frame
 * or error): noise/amplitude, frequency offset of the remote TX relative to
 * our own clock, and RX timestamp. Not called on a bare frame-wait-timeout,
 * since nothing was actually detected there.
 */
static void log_rx_diagnostics(const char *label, uint32_t status_reg)
{
    dwt_rxdiag_t rx_diag;
    dwt_readdiagnostics(&rx_diag);

    int32_t carrier_integrator = dwt_readcarrierintegrator();
    double freq_offset_hz = carrier_integrator * FREQ_OFFSET_MULTIPLIER;
    double freq_offset_ppm = freq_offset_hz * HERTZ_TO_PPM_MULTIPLIER_CHAN_3;

    uint32_t rx_timestamp = dwt_readrxtimestamphi32();

    ESP_LOGI(TAG, "[%s] RX_TIMESTAMP=0x%08" PRIx32 " freqOffset=%.1fHz (%.2fppm)",
             label, rx_timestamp, freq_offset_hz, freq_offset_ppm);
    ESP_LOGI(TAG, "[%s] RX_FQUAL: stdNoise=%u maxNoise=%u firstPath=%u firstPathAmp1=%u firstPathAmp2=%u firstPathAmp3=%u maxGrowthCIR=%u rxPreamCount=%u",
             label, rx_diag.stdNoise, rx_diag.maxNoise, rx_diag.firstPath,
             rx_diag.firstPathAmp1, rx_diag.firstPathAmp2, rx_diag.firstPathAmp3,
             rx_diag.maxGrowthCIR, rx_diag.rxPreamCount);
}


void DW1000_Receiver_Task(void *pvParameters)
{
    /** @todo for checking for valid reception and logging diagnostics. Store RX data in buffer and have second core print this out. */
    rx_event_t rx_event; // Struct to hold RX event data for queueing to the print task
    uint8_t rx_buffer[FRAME_LEN_MAX];
    // DW1000 initialization and configuration code goes here, similar to app_main().
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

    dwt_forcetrxoff();
    dwt_configure((dwt_config_t *)&rx_config);

    uint32_t preamble_count = 0, sfd_count = 0, good_count = 0;
    uint32_t phe_count = 0, fce_count = 0, rfsl_count = 0, hw_to_count = 0;

    bool preamble_seen = false;
    bool sfd_seen = false;

    // Reset status flags
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_RXFCE | SYS_STATUS_RXPHE | SYS_STATUS_RXRFSL | SYS_STATUS_RXRFTO | SYS_STATUS_RXPRD | SYS_STATUS_RXSFDD);

    // Enable RX immediately, since the device is in idle mode after initialization
    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS)
    {
        ESP_LOGE(TAG, "RX enable failed");
        while (1); /** @todo Handle error more gracefully, possibly with a retry mechanism or a reset of the DW1000. */
    }

    // Main loop for receiving frames and logging diagnostics.

    while (1)
    {
        uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);
        

        if (status_reg & SYS_STATUS_RXFCG)
        {

            uint32_t rx_finfo = dwt_read32bitreg(RX_FINFO_ID);
            uint32_t frame_len = rx_finfo & RX_FINFO_RXFL_MASK_1023;

        
            // log_rx_diagnostics("GOOD", status_reg); /** @todo Implement logging function */

        
            dwt_readrxdata(rx_buffer, frame_len, 0);

            rx_event.type = RX_EVENT_VALID;
            rx_event.rx_qual = read_rx_qual_register();  // verify actual function/register
            rx_event.rx_info = rx_finfo;  // store RX_FINFO for diagnostics
            rx_event.timestamp = dwt_readrxtimestamphi32();
            rx_event.payload_len = frame_len;
            /** @todo Implement safety here to ensure that if the FRAM_LEN_MAX is exceeded, the payload is truncated safely (i.e. the PHR_Mode is changed to standard) */
            memcpy(rx_event.payload, rx_buffer, frame_len);  // Copy the received payload into the event struct
            
            // fill payload, payload_len
            xQueueSend(xRxEventQueue, &rx_event, portMAX_DELAY);

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            preamble_seen = false;
            sfd_seen = false;
            dwt_rxenable(DWT_START_RX_IMMEDIATE);

            continue;
        }

        if (status_reg & SYS_STATUS_RXPHE)
        {
            ESP_LOGW(TAG, "PHY HEADER ERROR  SYS_STATUS=0x%08" PRIx32, status_reg);

            phe_count++;

            // Frame length/PHR is unreliable when the header itself failed to
            // decode, so there's no trustworthy payload to dump here — but the
            // LDE/CIR diagnostics ran before the PHR stage and are still valid.
            log_rx_diagnostics("PHE", status_reg);

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXPHE | SYS_STATUS_RXSFDD | SYS_STATUS_RXPRD);

            dwt_rxreset();

            preamble_seen = false;
            sfd_seen = false;

            dwt_rxenable(DWT_START_RX_IMMEDIATE);

            continue;
        }

        if (status_reg & SYS_STATUS_RXFCE)
        {
            ESP_LOGW(TAG, "FCS ERROR SYS_STATUS=0x%08" PRIx32, status_reg);

            fce_count++;

            // PHR decoded fine here (only the CRC failed), so the frame length
            // and payload bytes are still meaningful — dump them for inspection.
            // This intentionally does NOT use the "GOOD FRAME RECEIVED" header,
            // so uwb_live.py's pcap capture won't pick it up; it's a near-miss
            // for the human reading the log, not a validated frame.
            log_rx_diagnostics("FCE", status_reg);

            uint32_t rx_finfo = dwt_read32bitreg(RX_FINFO_ID);
            uint32_t frame_len = rx_finfo & RX_FINFO_RXFL_MASK_1023;

            if (frame_len > 0 && frame_len <= FRAME_LEN_MAX)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);

                ESP_LOGW(TAG, "[FCE] payload (CRC invalid), LEN=%lu:", frame_len);
                for (uint32_t i = 0; i < frame_len; i++)
                {
                    printf("%02X ", rx_buffer[i]);
                }
                printf("\n");
            }

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCE | SYS_STATUS_RXSFDD | SYS_STATUS_RXPRD);

            dwt_rxreset();

            preamble_seen = false;
            sfd_seen = false;

            dwt_rxenable(DWT_START_RX_IMMEDIATE);

            vTaskDelay(pdMS_TO_TICKS(1));

            continue;
        }

        if (status_reg & SYS_STATUS_RXRFSL)
        {
            ESP_LOGW(TAG, "REED SOLOMON ERROR SYS_STATUS=0x%08" PRIx32, status_reg);

            rfsl_count++;

            // Reed-Solomon FEC only applies at 110k; unreachable at the current
            // 850K config, kept here for parity with the other error branches.
            log_rx_diagnostics("RFSL", status_reg);

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFSL | SYS_STATUS_RXSFDD | SYS_STATUS_RXPRD);

            dwt_rxreset();

            preamble_seen = false;
            sfd_seen = false;

            dwt_rxenable(DWT_START_RX_IMMEDIATE);

            vTaskDelay(pdMS_TO_TICKS(1));

            continue;
        }

        if (status_reg & SYS_STATUS_RXRFTO)
        {
            ESP_LOGW(TAG, "FRAME WAIT TIMEOUT");

            hw_to_count++;

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFTO);

            dwt_rxreset();

            preamble_seen = false;
            sfd_seen = false;

            dwt_rxenable(DWT_START_RX_IMMEDIATE);

            continue;
        }

        /** @todo Implement Heartbeat feature utilizing onboard LED */

    }
}

void print_from_buffer_task(void *pvParameters)
{
    /** @todo for printing out the RX data buffer. */

    // This task will run on the second core and print the received data from the buffer. @todo Need to determine how to share the buffer between tasks safely (e.g., using a queue or mutex).
}


/**
 * @brief Main Application
 */
void app_main(void)
{   
    xTaskCreatePinnedToCore(DW1000_Receiver_Task, "Receiver Task", 2048, NULL, 1, NULL, 0); /** @todo Need to determine stack size and priority for these tasks */
    xTaskCreatePinnedToCore(print_from_buffer_task, "Print Buffer Task", 2048, NULL, 1, NULL, 1);

    // Here down is old code.
    

   
    
    
}
