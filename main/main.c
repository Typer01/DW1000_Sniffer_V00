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
static const char *TAG = "MAIN";

/* Buffer to store received frame. Sized for DWT_PHRMODE_EXT (up to 1023 bytes). */
#define FRAME_LEN_MAX 1023

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

// CH3, 64M PRF, PHR_EXT, PLEN1024/PAC32/850K — confirmed working against the
// transmitter/ validation rig (see ../transmitter/main/main.c).
static const dwt_config_t rx_config = {3, DWT_PRF_64M, DWT_PLEN_1024, DWT_PAC32, 9, 9, 0, DWT_BR_850K, DWT_PHRMODE_EXT, (1024 + 1 + 8 - 32)};

// Heartbeat: periodic liveness/rate log, independent of the radio state.
#define HEARTBEAT_INTERVAL_US (10 * 1000000)

/**
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

    dwt_forcetrxoff();
    dwt_configure((dwt_config_t *)&rx_config);

    ESP_LOGI(TAG, "DW1000 Ready, chan=%u prf=%u plen=%u pac=%u rate=%u phr=%u nsSFD=%u",
             rx_config.chan, rx_config.prf, rx_config.txPreambLength, rx_config.rxPAC,
             rx_config.dataRate, rx_config.phrMode, rx_config.nsSFD);

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
        while (1);
    }

    int64_t last_heartbeat = esp_timer_get_time();

    while (1)
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

            const char *rate_str = (rx_finfo & RX_FINFO_RXBR_MASK) == RX_FINFO_RXBR_6M ? "6.8M" :
                                    (rx_finfo & RX_FINFO_RXBR_MASK) == RX_FINFO_RXBR_850k ? "850k" : "110k";
            const char *prf_str = (rx_finfo & RX_FINFO_RXPRF_MASK) == RX_FINFO_RXPRF_64M ? "64M" : "16M";
            uint32_t pacc = (rx_finfo & RX_FINFO_RXPACC_MASK) >> RX_FINFO_RXPACC_SHIFT;

            ESP_LOGI(TAG, "RX_FINFO decode: rate=%s prf=%s preambleAcc=%lu", rate_str, prf_str, pacc);

            log_rx_diagnostics("GOOD", status_reg);

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

            vTaskDelay(pdMS_TO_TICKS(1));

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

        int64_t now = esp_timer_get_time();
        if ((now - last_heartbeat) >= HEARTBEAT_INTERVAL_US)
        {
            ESP_LOGI(TAG, "heartbeat: preamble=%lu sfd=%lu good=%lu phe=%lu fce=%lu rfsl=%lu hw_to=%lu",
                     preamble_count, sfd_count, good_count, phe_count, fce_count, rfsl_count, hw_to_count);
            last_heartbeat = now;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
