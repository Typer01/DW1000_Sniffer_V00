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
_Atomic uint32_t RX_DROPPED = 0; // Valid frames dropped because xRxEventQueue was full
_Atomic uint32_t RX_OVRR = 0;   // True receiver overrun (RX buffer not read in time)
_Atomic uint32_t RX_PTO = 0;    // Preamble detection timeout
_Atomic uint32_t RX_SFDTO = 0;  // SFD detection timeout


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
    uint64_t rx_qual;     // verify actual register name/width in DW1000 UM
    uint32_t rx_info;     // same — confirm exact register before using
    // Diagnostics data for logging purposes
    uint16_t maxNoise;      // LDE max value of noise
    uint16_t firstPathAmp1; // Amplitude at floor(index FP) + 1
    uint16_t stdNoise;      // Standard deviation of noise
    uint16_t firstPathAmp2; // Amplitude at floor(index FP) + 2
    uint16_t firstPathAmp3; // Amplitude at floor(index FP) + 3
    uint16_t maxGrowthCIR;  // Channel Impulse Response max growth CIR
    uint16_t rxPreamCount;  // Count of preamble symbols accumulated
    uint16_t firstPath;     // First path index (10.6 bits fixed point integer)
} rx_event_t;

QueueHandle_t xRxEventQueue;


// CH3, 64M PRF, PHR_EXT, PLEN1024/PAC32/850K - Confirmed working config for Tags
static const dwt_config_t rx_config = {1, DWT_PRF_64M, DWT_PLEN_256, DWT_PAC32, 9, 9, 0, DWT_BR_6M8, DWT_PHRMODE_EXT, (256 + 1 + 8 - 32)}; 


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

    // ESP_LOGI(TAG, "[%s] RX_TIMESTAMP=0x%08" PRIx32 " freqOffset=%.1fHz (%.2fppm)",
    //          label, rx_timestamp, freq_offset_hz, freq_offset_ppm);
    // ESP_LOGI(TAG, "[%s] RX_FQUAL: stdNoise=%u maxNoise=%u firstPath=%u firstPathAmp1=%u firstPathAmp2=%u firstPathAmp3=%u maxGrowthCIR=%u rxPreamCount=%u",
    //          label, rx_diag.stdNoise, rx_diag.maxNoise, rx_diag.firstPath,
    //          rx_diag.firstPathAmp1, rx_diag.firstPathAmp2, rx_diag.firstPathAmp3,
    //          rx_diag.maxGrowthCIR, rx_diag.rxPreamCount);
}


void DW1000_Receiver_Task(void *pvParameters)
{
    rx_event_t rx_event; // Struct to hold RX event data for queueing to the print task
    dwt_rxdiag_t rx_diag;
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
        
            /** @todo Update this function to work with Queue 
             * log_rx_diagnostics("GOOD", status_reg);
             **/

            
            dwt_readdiagnostics(&rx_diag);
            dwt_readrxdata(rx_buffer, frame_len, 0);

            rx_event.type = RX_EVENT_VALID;// May not be necessary to store or log this in the struct, but leaving for now
            dwt_readfromdevice(RX_FQUAL_ID, 0x0, 8, (uint8_t *)&rx_event.rx_qual);
            rx_event.rx_info = rx_finfo;  // store RX_FINFO for diagnostics
            rx_event.timestamp = dwt_readrxtimestamphi32();
            rx_event.payload_len = frame_len;

            rx_event.maxNoise = rx_diag.maxNoise;
            rx_event.firstPathAmp1 = rx_diag.firstPathAmp1;
            rx_event.stdNoise = rx_diag.stdNoise;
            rx_event.firstPathAmp2 = rx_diag.firstPathAmp2;
            rx_event.firstPathAmp3 = rx_diag.firstPathAmp3;
            rx_event.maxGrowthCIR = rx_diag.maxGrowthCIR;
            rx_event.rxPreamCount = rx_diag.rxPreamCount;
            rx_event.firstPath = rx_diag.firstPath;



            /** @todo Implement safety here to ensure that if the FRAM_LEN_MAX is exceeded, the payload is truncated safely (i.e. the PHR_Mode is changed to standard) */
            memcpy(rx_event.payload, rx_buffer, frame_len);  // Copy the received payload into the event struct
            
            // Queue RX Structure and increment RX_FCG Counter.
            // Non-blocking: print_from_buffer_task is slower than the RX rate can be, so
            // if the queue is full, drop this frame instead of blocking the receiver
            // (portMAX_DELAY here previously let a full queue stall RX indefinitely).
            if (xQueueSend(xRxEventQueue, &rx_event, 0) != pdTRUE)
            {
                atomic_fetch_add_explicit(&RX_DROPPED, 1, memory_order_relaxed);
            }
            atomic_fetch_add_explicit(&RX_FCG, 1, memory_order_relaxed); // Increments counter

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

            atomic_fetch_add_explicit(&RX_PHE, 1, memory_order_relaxed); // Increments counter

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

            atomic_fetch_add_explicit(&RX_FCE, 1, memory_order_relaxed); // Increments counter
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

            atomic_fetch_add_explicit(&RX_RFSL, 1, memory_order_relaxed); // Increments counter
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

            atomic_fetch_add_explicit(&RX_RFTO, 1, memory_order_relaxed); // Increments counter
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFTO);
            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            continue;
        }
        // Receiver Overrun / Preamble Detection Timeout / SFD Timeout.
        // These are latched status bits this loop previously never checked or cleared.
        // Left unhandled, they stay set forever and the DW1000's receive state machine
        // stops generating any further RXFCG/error events until an RX reset is done -
        // this is what caused the "all counters frozen, only a full reboot recovers"
        // symptom (a plain loss of RF signal would instead still show the error
        // counters above climbing, not everything frozen).
        if (status_reg & (SYS_STATUS_RXOVRR | SYS_STATUS_RXPTO | SYS_STATUS_RXSFDTO))
        {
            if (status_reg & SYS_STATUS_RXOVRR)  atomic_fetch_add_explicit(&RX_OVRR, 1, memory_order_relaxed);
            if (status_reg & SYS_STATUS_RXPTO)   atomic_fetch_add_explicit(&RX_PTO, 1, memory_order_relaxed);
            if (status_reg & SYS_STATUS_RXSFDTO) atomic_fetch_add_explicit(&RX_SFDTO, 1, memory_order_relaxed);
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXOVRR | SYS_STATUS_RXPTO | SYS_STATUS_RXSFDTO);
            dwt_forcetrxoff();
            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            continue;
        }

        /** Uncomment for Debugging @todo Implement Heartbeat feature utilizing onboard TFT */
        // ESP_LOGD(TAG, "Badump Badump"); // Temp Heart beat

        /** @todo Replace this polling loop with the interrupt-driven path already
         * implemented in components/deca_gpio/deca_gpio.c (dw1000_setup_isr() +
         * dwt_setcallbacks()), which blocks on ulTaskNotifyTake instead of busy-polling
         * SYS_STATUS_ID. That removes CPU spin entirely instead of just yielding. */
        // No RX event this iteration - yield so lower-priority tasks (incl. IDLE0)
        // can run; prevents task watchdog starvation on CPU0.
        vTaskDelay(1); // Adjust this value if CPU0 is still being starved.
    }
}

void print_from_buffer_task(void *pvParameters)
{
    
    /** @todo Update this task to print faster, currently bottlenecking incoming data here */
    // This task will run on the second core and print the received data from the buffer.

    rx_event_t rx_event; // Struct to hold RX event data from queue
    int64_t prevTime = 0;

    // Wait until Receiver is initialized

    printf("timestamp,payload_len,rx_qual,rx_info,payload\n");

    while(1)
    {
        uint32_t FCG = atomic_load_explicit(&RX_FCG, memory_order_relaxed);
        // Receive struct from queue. Bounded timeout (not portMAX_DELAY) so the loop
        // still wakes up and reaches the 3-second counter print below even when no
        // frames are arriving - otherwise this call blocks forever on an empty queue
        // and the heartbeat print below never runs.
        if(xQueueReceive(xRxEventQueue, &rx_event, pdMS_TO_TICKS(500)) == pdTRUE) // Checks if Queue contains
        {
            //ESP_LOGI(TAG, "QUEUE RECEPTION");
            //ESP_LOGD(TAG, "Received RX Event from Queue: Type=%d, Timestamp=0x%08" PRIx32 ", Payload Length=%zu", rx_event.type, rx_event.timestamp, rx_event.payload_len);
            // CSV Print Format
            // Timestamp, Payload Length, RX Quality, RX Info, Payload Data
            printf("%" PRIu32 ",%zu,0x%016" PRIX64 ",0x%08" PRIX32 ",",
                   rx_event.timestamp, rx_event.payload_len,
                   rx_event.rx_qual, rx_event.rx_info);

            for(size_t i = 0; i < rx_event.payload_len; i++)
            {
                printf("%02X", rx_event.payload[i]);
            }
            printf("\n");
        };

        // Log the received packets into a "friendly format" for the user to read. This will include the timestamp, signal quality, and the actual payload data.
        /** @todo Determine faster logging format and method */
        /** @todo Add heartbeat feature utilizing onboard TFT for this task */

        if(esp_timer_get_time() - prevTime >= 10000000) // Every 10 second, print the RX counters
        {
            uint32_t dropped = atomic_load_explicit(&RX_DROPPED, memory_order_relaxed);
            uint32_t phe = atomic_load_explicit(&RX_PHE, memory_order_relaxed);
            uint32_t fce = atomic_load_explicit(&RX_FCE, memory_order_relaxed);
            uint32_t rfsl = atomic_load_explicit(&RX_RFSL, memory_order_relaxed);
            uint32_t rfto = atomic_load_explicit(&RX_RFTO, memory_order_relaxed);
            uint32_t ovrr = atomic_load_explicit(&RX_OVRR, memory_order_relaxed);
            uint32_t pto = atomic_load_explicit(&RX_PTO, memory_order_relaxed);
            uint32_t sfdto = atomic_load_explicit(&RX_SFDTO, memory_order_relaxed);
            printf("Counters: FCG=%" PRIu32 ", PHE=%" PRIu32 ", FCE=%" PRIu32 ", RFSL=%" PRIu32 ", RFTO=%" PRIu32
                   ", OVRR=%" PRIu32 ", PTO=%" PRIu32 ", SFDTO=%" PRIu32 ", DROPPED=%" PRIu32 "\n",
                   FCG, phe, fce, rfsl, rfto, ovrr, pto, sfdto, dropped);
            prevTime = esp_timer_get_time();
        }
    }
    
}

void display_update_task(void *pvParameters)
{
    // This task will run on the second core and updaate the display with the received data.
    /** @todo Implement display update functionality */

    // Wait for 
}

/**
 * @brief Main Application
 */
void app_main(void)
{   
    xRxEventQueue = xQueueCreate(20, sizeof(rx_event_t)); /** @todo Determine proper queue size */
    ESP_LOGI(TAG, "Queue Created: %p", xRxEventQueue);

    xTaskCreatePinnedToCore(DW1000_Receiver_Task, "Receiver Task", 5120, NULL, 1, NULL, 0); /** @todo Need to determine optimal stack size and priority for these tasks */
    xTaskCreatePinnedToCore(print_from_buffer_task, "Print Buffer Task", 5120, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(display_update_task, "Display Update Task", 5120, NULL, 1, NULL, 1);
    
}
