/**
 * @file main.c
 * @brief ESP32 Makerfabs DW1000 Receiver. Utilizing FreeRTOS based reception and logging.
 * @todo Update brief
 * 
 */

/** Task List
 * @todo Update all debugging functions to work under FreeRTOS setup
 * @todo Investigate and clean up includes in driver files
 * @todo MISRA C Compliance Check
 * @todo Update and standardize Log levels, add functionality to filter logging
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
#include "freertos/queue.h"
#include "freertos/atomic.h"
#include <stdatomic.h>

// ESP_LOG Tag for Main file
/** @todo Add more descriptive logging Tags for main file*/
static const char *TAG = "MAIN";

/* Buffer to store received frame. Sized for DWT_PHRMODE_EXT (up to 1023 bytes). */
#define FRAME_LEN_MAX 1023
// --- Atomic Counters ---
_Atomic uint32_t RX_FCG = 0; // FCS Good 
_Atomic uint32_t RX_PHE = 0; // PHY Header Error 
_Atomic uint32_t RX_RFSL = 0; // Reed Solomon Error
_Atomic uint32_t RX_RFTO = 0; // Frame Wait Timeout
_Atomic uint32_t RX_FCE = 0; // Receiver FCS Error


/** @todo Determine if PHY and RS error Enumerations will ever be used. Can consolidate memnory in rx_event_t struct */
typedef enum {
    RX_EVENT_VALID,
    RX_EVENT_PHY_ERROR,
    RX_EVENT_RS_ERROR   // Reed-Solomon
} rx_event_type_t;

typedef struct {
    rx_event_type_t type; // potentially unecesary 
    uint32_t timestamp;

    // Only meaningful when type == RX_EVENT_VALID
    uint8_t payload[FRAME_LEN_MAX];  // Maximum payload size for DWT_PHRMODE_EXT
    size_t payload_len;
    uint32_t rx_qual;     // verify actual register name/width in DW1000 UM
    uint32_t rx_info;     // same — confirm exact register before using
} rx_event_t;

QueueHandle_t xRxEventQueue;


// CH3, 64M PRF, PHR_EXT, PLEN1024/PAC32/850K - Confirmed working config.
static const dwt_config_t rx_config = {3, DWT_PRF_64M, DWT_PLEN_1024, DWT_PAC32, 9, 9, 0, DWT_BR_850K, DWT_PHRMODE_EXT, (1024 + 1 + 8 - 32)};


/** @todo Review this code and refactor for Task/Queue structure.  
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
    rx_event_t rx_event; // Struct to hold RX event data for queueing to the print task
    uint8_t rx_buffer[FRAME_LEN_MAX];
    uint32_t status_reg, rx_finfo, frame_len;

    // DW1000 and SPI initialization 
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
        while (1); /** @todo Handle SPI Bus initialization error more gracefully */
    }

    if (dw1000_gpio_init((gpio_num_t)UWB_RST_PIN,(gpio_num_t)UWB_IRQ_PIN,GPIO_NUM_NC) != 0)
    {
        ESP_LOGE(TAG, "GPIO init failed");
        while (1); /** @todo Handle DW1000 GPIO initialization error more gracefully */
    }

    dw1000_hard_reset();
    dw1000_spi_fix_bug();

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        ESP_LOGE(TAG, "dwt_initialise failed");
        while (1); /** @todo Handle DW1000 initialization error more gracefully */
    }

    spi_set_rate_high();

    dwt_forcetrxoff();
    dwt_configure((dwt_config_t *)&rx_config);

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
        status_reg = dwt_read32bitreg(SYS_STATUS_ID);
        // --- Valid Receive ---
        if (status_reg & SYS_STATUS_RXFCG)
        {

            rx_finfo = dwt_read32bitreg(RX_FINFO_ID);
            frame_len = rx_finfo & RX_FINFO_RXFL_MASK_1023;
        
            /** @todo Update this function to work with Queuelog_rx_diagnostics("GOOD", status_reg); */
            dwt_readrxdata(rx_buffer, frame_len, 0);

            rx_event.type = RX_EVENT_VALID;
            rx_event.rx_qual = read_rx_qual_register();  // verify actual function/register
            rx_event.rx_info = rx_finfo;  // store RX_FINFO for diagnostics
            rx_event.timestamp = dwt_readrxtimestamphi32();
            rx_event.payload_len = frame_len;

            /** @todo Implement safety here to ensure that if the FRAM_LEN_MAX is exceeded, the payload is truncated safely (i.e. the PHR_Mode is changed to standard) */
            memcpy(rx_event.payload, rx_buffer, frame_len);  // Copy the received payload into the event struct
            
            // Queue RX Structure and increment RX_FCG Counter
            xQueueSend(xRxEventQueue, &rx_event, portMAX_DELAY);
            Atomic_Add_u32(&RX_FCG, 1); // Increments counter

            // Reset Sequence
            /** @todo Investigate this reset sequence, determine if this is the appropriate response when getting a valid receive */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            continue; 
        }
        // --- PHY Error ---
        if (status_reg & SYS_STATUS_RXPHE)
        {

            /** For Debugging - Uncomment here @todo Make the RXPHE Functional */
            //     ESP_LOGW(TAG, "PHY HEADER ERROR  SYS_STATUS=0x%08" PRIx32, status_reg);
            //     log_rx_diagnostics("PHE", status_reg);

            Atomic_Add_u32(&RX_PHE, 1); // Increments counter

            // Reset Sequence
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXPHE | SYS_STATUS_RXSFDD | SYS_STATUS_RXPRD);
            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            continue;
        }
        // Receive CRC Error
        if (status_reg & SYS_STATUS_RXFCE)
        {
        /**    Uncomment for debugging @todo Make the RXFCE debug functional */
        //     ESP_LOGW(TAG, "FCS ERROR SYS_STATUS=0x%08" PRIx32, status_reg);
        //     uint32_t rx_finfo = dwt_read32bitreg(RX_FINFO_ID);
        //     uint32_t frame_len = rx_finfo & RX_FINFO_RXFL_MASK_1023;
        //     ESP_LOGW(TAG, "[FCE] payload (CRC invalid), LEN=%lu:", frame_len);

            Atomic_Add_u32(&RX_FCE, 1); // Increments counter
        // Reset Sequence
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCE | SYS_STATUS_RXSFDD | SYS_STATUS_RXPRD);
            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            continue;
        }
        // Receive Reed Solomon Error
        if (status_reg & SYS_STATUS_RXRFSL)
        {
            /**    Uncomment for debugging @todo Make the RXFSL debug functional */
            // ESP_LOGW(TAG, "REED SOLOMON ERROR SYS_STATUS=0x%08" PRIx32, status_reg);
            // log_rx_diagnostics("RFSL", status_reg);

            Atomic_Add_u32(&RX_RFSL, 1); // Increments counter
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFSL | SYS_STATUS_RXSFDD | SYS_STATUS_RXPRD);
            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            continue;
        }
        // Receive Frame Timeout Error
        if (status_reg & SYS_STATUS_RXRFTO)
        {
            /** Uncomment for debugging @todo Make the Frame Wait Timeout Debug more Robust */
            // ESP_LOGW(TAG, "FRAME WAIT TIMEOUT");

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFTO);
            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            continue;
        }

        /** Uncomment for Debugging @todo Implement Heartbeat feature utilizing onboard TFT */
        // ESP_LOGD(TAG, "Badump Badump"); // Temp Heart beat
    }
}

void print_from_buffer_task(void *pvParameters)
{
    
    /** @todo for printing out the RX data buffer. */
    // This task will run on the second core and print the received data from the buffer.

    rx_event_t rx_event; // Struct to hold RX event data from queue

    // Wait until Receiver is initialized

    printf("timestamp,payload_len,rx_qual,rx_info,payload\n");

    while(1)
    {
        uint32_t FCG = atomic_load_explicit(&RX_FCG, memory_order_relaxed);
        // Receive struct from queue
        if(xQueueReceive(xRxEventQueue, &rx_event, portMAX_DELAY) == pdTRUE) // Checks if Queue contains 
        {
            ESP_LOGD(TAG, "Received RX Event from Queue: Type=%d, Timestamp=0x%08" PRIx32 ", Payload Length=%zu", rx_event.type, rx_event.timestamp, rx_event.payload_len);
            // CSV Print Format
            // Timestamp, Payload Length, RX Quality, RX Info, Payload Data
            printf("%" PRIu32 ",%zu,%" PRIu32 ",0x%08" PRIX32 ",",
                   rx_event.timestamp, rx_event.payload_len,
                   rx_event.rx_qual, rx_event.rx_info);
            for(size_t i = 0; i < rx_event.payload_len; i++)
            {
                printf("%02X", rx_event.payload[i]);
            }
            printf("\n");
            
        };


        // Log the received packets into a "friendly format" for the user to read. This will include the timestamp, signal quality, and the actual payload data.
        /** @todo Determine proper logging format and method */
        /** @todo Add heartbeat feature utilizing onboard TFT for this task */

    }
    
}


/**
 * @brief Main Application
 */
void app_main(void)
{   
    xRxEventQueue = xQueueCreate(20, sizeof(rx_event_t)); /** @todo Determine proper queue size */
    ESP_LOGI(TAG, "Queue Created: %p", xRxEventQueue);

    xTaskCreatePinnedToCore(DW1000_Receiver_Task, "Receiver Task", 2048, NULL, 1, NULL, 0); /** @todo Need to determine stack size and priority for these tasks */
    xTaskCreatePinnedToCore(print_from_buffer_task, "Print Buffer Task", 2048, NULL, 1, NULL, 1);

    

   
    
    
}
