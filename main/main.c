/**
 * @file main.c
 * @brief Event-driven DW1000 RX handler (ESP32-S3 / ESP-IDF)
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_spi.h"
#include "deca_gpio.h"

#include "hardware_defs.h"

static const char *TAG = "UWB";

/* ---------------- CONFIG ---------------- */

#define FRAME_LEN_MAX 127

#define UWB_EVENT_IRQ     (1 << 0)

/* ---------------- RTOS OBJECTS ---------------- */

static EventGroupHandle_t uwb_event_group;

/* ---------------- RX BUFFER ---------------- */

static uint8_t rx_buffer[FRAME_LEN_MAX];

/* ---------------- IRQ HANDLER ---------------- */
/*
 * IMPORTANT:
 * Do NOT do SPI here. Only signal event.
 */
static void IRAM_ATTR dw1000_irq_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xEventGroupSetBitsFromISR(
        uwb_event_group,
        UWB_EVENT_IRQ,
        &xHigherPriorityTaskWoken
    );

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ---------------- RX TASK ---------------- */

static void uwb_rx_task(void *arg)
{
    ESP_LOGI(TAG, "UWB RX task started");
    
    while (1)
    {
        /* Wait for DW1000 interrupt event */
        xEventGroupWaitBits(
            uwb_event_group,
            UWB_EVENT_IRQ,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY
        );

        /* Read status ONCE per interrupt */
        uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);

        /* ---------------- RX SUCCESS ---------------- */
        if (status & SYS_STATUS_RXFCG)
        {
            uint32_t frame_len =
                dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;

            ESP_LOGI(TAG, "RXFCG OK, len=%lu", frame_len);

            if (frame_len > 0 && frame_len <= FRAME_LEN_MAX)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);

                printf("DATA: ");
                for (uint32_t i = 0; i < frame_len; i++)
                {
                    printf("%02X ", rx_buffer[i]);
                }
                printf("\n");
            }

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        /* ---------------- PHY ERROR ---------------- */
        if (status & SYS_STATUS_RXPHE)
        {
            ESP_LOGW(TAG, "PHY HEADER ERROR 0x%08lx", status);

            dwt_write32bitreg(SYS_STATUS_ID,
                SYS_STATUS_RXPHE |
                SYS_STATUS_RXPRD |
                SYS_STATUS_RXSFDD);

            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        /* ---------------- FCS ERROR ---------------- */
        if (status & SYS_STATUS_RXFCE)
        {
            ESP_LOGW(TAG, "FCS ERROR 0x%08lx", status);

            dwt_write32bitreg(SYS_STATUS_ID,
                SYS_STATUS_RXFCE |
                SYS_STATUS_RXPRD |
                SYS_STATUS_RXSFDD);

            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        /* ---------------- REED SOLOMON ERROR ---------------- */
        if (status & SYS_STATUS_RXRFSL)
        {
            ESP_LOGW(TAG, "RS ERROR 0x%08lx", status);

            dwt_write32bitreg(SYS_STATUS_ID,
                SYS_STATUS_RXRFSL |
                SYS_STATUS_RXPRD |
                SYS_STATUS_RXSFDD);

            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        /* ---------------- TIMEOUT ---------------- */
        if (status & SYS_STATUS_RXRFTO)
        {
            ESP_LOGW(TAG, "RX TIMEOUT");

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFTO);

            dwt_rxreset();
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
    }
}

/* ---------------- APP MAIN ---------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "Starting DW1000 event-driven RX");

    uwb_event_group = xEventGroupCreate();

    /* ---------------- SPI INIT ---------------- */

    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1024
    };

    if (dw1000_spi_init(SPI2_HOST, UWB_CS_PIN, &spi_bus_cfg) != 0)
    {
        ESP_LOGE(TAG, "SPI init failed");
        return;
    }

    /* ---------------- GPIO INIT ---------------- */

    if (dw1000_gpio_init(UWB_RST_PIN, UWB_IRQ_PIN, GPIO_NUM_NC) != 0)
    {
        ESP_LOGE(TAG, "GPIO init failed");
        return;
    }

    /* Install ISR */
    gpio_set_intr_type(UWB_IRQ_PIN, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(UWB_IRQ_PIN, dw1000_irq_handler, NULL);

    /* ---------------- DW1000 INIT ---------------- */

    dw1000_hard_reset();
    dw1000_spi_fix_bug();

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        ESP_LOGE(TAG, "DW1000 init failed");
        return;
    }

    spi_set_rate_high();

    ESP_LOGI(TAG, "DW1000 ready");

    /* ---------------- CONFIG ---------------- */

    dwt_config_t config = {
        3,                  // channel (you said fixed CH3)
        DWT_PRF_16M,
        DWT_PLEN_2048,
        DWT_PAC64,
        5,
        5,
        0,
        DWT_BR_110K,
        DWT_PHRMODE_STD,
        (2048 + 1 + 8 - 64)
    };

    dwt_configure(&config);

    /* ---------------- START RX ---------------- */

    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* ---------------- START TASK ---------------- */

    xTaskCreate(uwb_rx_task, "uwb_rx_task", 8192, NULL, 5, NULL);
}