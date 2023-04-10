#include <stdio.h>
#include "driver/gpio.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include "driver/uart.h"

#define BUF_SIZE (1024)

#define BLINK_GPIO GPIO_NUM_2

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

TaskHandle_t sendHandle = NULL;
TaskHandle_t recvHandle = NULL;
TaskHandle_t blinkHandle = NULL;
TaskHandle_t uartHandle = NULL;

static void configure_led(void)
{
    gpio_set_pull_mode(BLINK_GPIO, GPIO_PULLDOWN_ONLY);
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

void sendTask(void *arg)
{
    uint32_t value = 0;
    while (1)
    {
        // Configure message to transmit
        twai_message_t message;
        message.identifier = 0x24F;
        message.extd = 0;
        message.data_length_code = 4;
        message.data[0] = (value >> 24) & 0xFF;
        message.data[1] = (value >> 16) & 0xFF;
        message.data[2] = (value >> 8) & 0xFF;
        message.data[3] = value++ & 0xFF;
        esp_err_t ret = twai_transmit(&message, pdMS_TO_TICKS(1000));
        // Queue message for transmission
        if (ret != ESP_OK)
        {
            ESP_LOGE("send", "Failed to queue message for transmission: %s", esp_err_to_name(ret));
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void recvTask(void *arg)
{
    char data[32];
    twai_message_t message;
    while (1)
    {
        // Queue message for transmission
        esp_err_t ret = twai_receive(&message, pdMS_TO_TICKS(1000));
        if (ret == ESP_ERR_TIMEOUT)
        {
            continue;
        }
        if (ret != ESP_OK)
        {
            ESP_LOGE("recv", "failed to receive message: %s", esp_err_to_name(ret));
            continue;
        }
        sprintf(data, "%04lx:%X:", message.identifier, message.data_length_code);
        uart_write_bytes(UART_NUM_1, data, strnlen(data, 32));
        for (size_t i = 0; i < message.data_length_code; i++)
        {
            sprintf(data, "%02X", message.data[i]);
            uart_write_bytes(UART_NUM_1, data, 2);
        }
        uart_write_bytes(UART_NUM_1, "\n", 1);
    }
}

void blinkTask(void *arg)
{
    uint8_t s_led_state = 0;
    while (1)
    {
        gpio_set_level(BLINK_GPIO, s_led_state);
        s_led_state = !s_led_state;
        // uart_write_bytes(UART_NUM_1, "b", 1);
        vTaskDelay(900 / portTICK_PERIOD_MS);
    }
}

void sendMSG()
{
    // Configure message to transmit
    twai_message_t rxMsg = {
        .identifier = 0x666,
        .rtr = 0,
        .extd = 0,
        .data_length_code = 4,
        .data = {0xDE, 0xAD, 0xBE, 0xEF},
    };
    esp_err_t ret = twai_transmit(&rxMsg, pdMS_TO_TICKS(100));
    // Queue message for transmission
    if (ret != ESP_OK)
    {
        ESP_LOGE("send", "Failed to queue message for transmission: %s", esp_err_to_name(ret));
    }
}

void uartTask(void *arg)
{
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    while (1)
    {
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, pdMS_TO_TICKS(10));
        if (len > 0)
        {
            ESP_LOGI("UART", "Read %d bytes: '%.*s'\n", len, len, data);
            if (strncmp((char *)data, "s", 1) == 0)
            {
                sendMSG();
            }
        }
    }
    free(data);
}

int configure_can()
{
    // Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_NORMAL);
    // twai_timing_config_t t_config = {.brp = 104, .tseg_1 = 14, .tseg_2 = 8, .sjw = 4, .triple_sampling = false}; // 33.3 kbps (NG9-3 I-BUS (GMLAN))
    // twai_timing_config_t t_config = {.brp = 80, .tseg_1 = 12, .tseg_2 = 8, .sjw = 4, .triple_sampling = false}; // 47.619 kbps (OG9-3 & OG9-5 IBUS)
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    // ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        ESP_LOGI("CAN", "Driver installed");
    }
    else
    {
        ESP_LOGE("CAN", "Failed to install driver");
        return 1;
    }

    // Start TWAI driver
    // ESP_ERROR_CHECK(twai_start());

    if (twai_start() == ESP_OK)
    {
        ESP_LOGI("CAN", "Driver started");
    }
    else
    {
        ESP_LOGE("CAN", "Failed to start driver");
        return 1;
    }
    return 0;
}

void app_main(void)
{

    configure_led();

    if (configure_can() != 0)
    {
        return;
    }

    uart_config_t uart_config = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 256, 0, NULL, 0));

    xTaskCreate(sendTask, "Send Task", 4096, NULL, 10, &sendHandle);
    xTaskCreate(recvTask, "Recv Task", 4096, NULL, 10, &recvHandle);
    xTaskCreate(blinkTask, "blink Task", 4096, NULL, 10, &blinkHandle);
    xTaskCreate(uartTask, "uart Task", 4096, NULL, 10, &uartHandle);
}