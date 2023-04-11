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

#define EX_UART_NUM UART_NUM_1
#define PATTERN_CHR_NUM (3) /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

static const char *TAG = "uart_events";

static QueueHandle_t uart1_queue;
TaskHandle_t recvHandle = NULL;

// Initialize configuration structures using macro initializers
// twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_NORMAL);
twai_general_config_t g_config = {.mode = TWAI_MODE_NORMAL, .tx_io = GPIO_NUM_21, .rx_io = GPIO_NUM_22, .clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED, .tx_queue_len = 5, .rx_queue_len = 20, .alerts_enabled = TWAI_ALERT_NONE, .clkout_divider = 0, .intr_flags = ESP_INTR_FLAG_LEVEL3};
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
// twai_filter_config_t f_config = {.acceptance_code = (0x7E8 << 21), .acceptance_mask = ~(0x7FF << 21), .single_filter = true};

void start_can()
{
    // Install TWAI driver
    // ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        ESP_LOGI("can", "Driver installed");
    }
    else
    {
        ESP_LOGE("can", "Failed to install driver");
        return;
    }

    // Start TWAI driver
    // ESP_ERROR_CHECK(twai_start());

    if (twai_start() == ESP_OK)
    {
        ESP_LOGI("can", "Driver started");
    }
    else
    {
        ESP_LOGE("can", "Failed to start driver");
        return;
    }
}

void stop_can()
{
    esp_err_t ret = twai_stop();
    if (ret != ESP_OK)
    {
        ESP_LOGE("stop", "Failed to stop driver: %s", esp_err_to_name(ret));
        return;
    }

    ret = twai_driver_uninstall();
    if (ret != ESP_OK)
    {
        ESP_LOGE("stop", "Failed to uninstall driver: %s", esp_err_to_name(ret));
        return;
    }
}

void recvTask(void *arg)
{
    char data[22];
    twai_message_t rxMsg;
    while (1)
    {
        // Queue message for transmission
        esp_err_t ret = twai_receive(&rxMsg, pdMS_TO_TICKS(1000));
        if (ret == ESP_ERR_TIMEOUT)
        {
            continue;
        }
        if (ret != ESP_OK)
        {
            ESP_LOGE("recv", "Failed to receive message: %s", esp_err_to_name(ret));
            continue;
        }

        // Convert identifier and data length code to string
        int id_len = snprintf(data, sizeof(data), "t%03lX%X", rxMsg.identifier, rxMsg.data_length_code);

        // Convert data bytes to hex string
        char *hex_str = &data[5];
        for (size_t i = 0; i < rxMsg.data_length_code; i++)
        {
            *hex_str++ = "0123456789ABCDEF"[rxMsg.data[i] >> 4];
            *hex_str++ = "0123456789ABCDEF"[rxMsg.data[i] & 0x0F];
        }

        // Add newline character
        *hex_str++ = '\n';

        // Send message to UART
        uart_write_bytes(EX_UART_NUM, data, id_len + 1 + rxMsg.data_length_code * 2);
    }
}

static char hexBuff[9];
void transmitPacket(char *cmd, char *ptr, twai_message_t *txMsg)
{
    memcpy(hexBuff, ptr++, 1);
    hexBuff[1] = '\0';
    txMsg->data_length_code = strtoul(hexBuff, NULL, 16);

    // printf("pid: %03lX, pdata_len: %d\n", pid, pdata_len);

    for (int i = 0; i < txMsg->data_length_code; i++)
    {
        memcpy(hexBuff, ptr, 2);
        hexBuff[2] = '\0';
        ptr += 2;
        txMsg->data[i] = strtoul(hexBuff, NULL, 16);
    }

    // Transmit CAN packet
    esp_err_t ret = twai_transmit(txMsg, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK)
    {
        ESP_LOGE("send", "Failed to queue message for transmission: %s", esp_err_to_name(ret));
    }
}

char hex_char(uint8_t nibble)
{
    return "0123456789ABCDEF"[nibble & 0xf];
}

void transmit11Packet(char *cmd)
{
    static twai_message_t txMsg = {
        .rtr = 0,
        .extd = 0,
    };

    // Extract id, data length, and data from cmd
    char *ptr = cmd;
    ptr++; // skip over the first character

    memcpy(hexBuff, ptr, 3);
    hexBuff[3] = '\0';
    txMsg.identifier = strtoul(hexBuff, NULL, 16);
    ptr += 3;

    transmitPacket(cmd, ptr, &txMsg);
}

void transmit29Packet(char *cmd)
{
    static twai_message_t txMsg = {
        .rtr = 0,
        .extd = 1,
    };

    // Extract id, data length, and data from cmd
    char *ptr = cmd;
    ptr++; // skip over the first character

    memcpy(hexBuff, ptr, 8);
    hexBuff[8] = '\0';
    txMsg.identifier = strtoul(hexBuff, NULL, 16);
    ptr += 8;

    transmitPacket(cmd, ptr, &txMsg);
}

void parseCMD(char *cmd)
{
    switch (cmd[0])
    {
    case 'S':
        switch (cmd[1])
        {

        case '0':
            t_config.brp = 104;
            t_config.tseg_1 = 14;
            t_config.tseg_2 = 8;
            t_config.sjw = 4;
            t_config.triple_sampling = false;
            ESP_LOGI("can", "Setting 33.3 kbps (NG9-3 I-BUS (GMLAN))");
            break;
        case '1':
            t_config.brp = 80;
            t_config.tseg_1 = 12;
            t_config.tseg_2 = 8;
            t_config.sjw = 4;
            t_config.triple_sampling = false;
            ESP_LOGI("can", "Setting 47.619 kbps (OG9-3 & OG9-5 IBUS)");
            break;
        case '2':
            t_config.brp = 8;
            t_config.tseg_1 = 15;
            t_config.tseg_2 = 4;
            t_config.sjw = 3;
            t_config.triple_sampling = false;
            ESP_LOGI("can", "Setting 500 kbps (T7/T8 PBUS)");
            break;
        case '3':
            t_config.brp = 10;
            t_config.tseg_1 = 7;
            t_config.tseg_2 = 5;
            t_config.sjw = 4;
            t_config.triple_sampling = false;
            ESP_LOGI("can", "Setting 615.384 kbps (T5 PBUS)");
            break;
        default:
            ESP_LOGE("can", "Invalid speed");
            break;
        }
        break;
    case 'C':
        ESP_LOGI("can", "Stopping recv Task");
        vTaskSuspend(recvHandle);
        stop_can();
        gpio_set_level(BLINK_GPIO, false);
        return;
    case 'O':
        start_can();
        if (recvHandle == NULL)
        {
            ESP_LOGI("can", "Create recv Task");
            xTaskCreate(&recvTask, "recvTask", 4096, NULL, 5, &recvHandle);
        }
        else
        {
            ESP_LOGI("can", "Resume recv Task");
            vTaskResume(recvHandle);
        }
        gpio_set_level(BLINK_GPIO, true);
        return;
    case 'V':
        uart_write_bytes(EX_UART_NUM, "V1013\r", 6);
        return;
    case 'N':
        uart_write_bytes(EX_UART_NUM, "N1337\r", 7);
        return;
    case 't':
        transmit11Packet(cmd);
        return;
    case 'T':
        transmit29Packet(cmd);
        return;
    case 0x00:
        return;
    default:
        ESP_LOGE("parse_cmd", "Unknown command: %s", cmd);
        uart_write_bytes(EX_UART_NUM, "\x7", 1);
        return;
    }
}
static char command[22];
static int command_index = 0;

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t *dtmp = (uint8_t *)malloc(RD_BUF_SIZE);
    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(uart1_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            bzero(dtmp, RD_BUF_SIZE);
            // ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch (event.type)
            {
            // Event of UART receving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                // ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                // ESP_LOGI(TAG, "[DATA EVT]:");
                // uart_write_bytes(EX_UART_NUM, (const char *)dtmp, event.size);

                for (int i = 0; i < event.size; i++)
                {
                    switch (dtmp[i])
                    {
                    case 0x0D:
                        command[command_index] = '\0'; // add null terminator to command string
                        parseCMD(command);
                        command_index = 0;
                        break;
                    case 0x8:
                        if (command_index > 0)
                        {
                            command_index--;
                        }
                        break;
                    default:
                        command[command_index++] = dtmp[i];
                        if (command_index >= sizeof(command)) // handle buffer overflow
                        {
                            ESP_LOGE(TAG, "command buffer overflow");
                            command_index = 0;
                        }
                        break;
                    }
                }

                break;
            // Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGE(TAG, "hw fifo overflow");
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart1_queue);
                break;
            // Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGE(TAG, "ring buffer full");
                // If buffer full happened, you should consider encreasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(EX_UART_NUM);
                xQueueReset(uart1_queue);
                break;
            // Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            // Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGE(TAG, "uart parity error");
                break;
            // Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGE(TAG, "uart frame error");
                break;
            // UART_PATTERN_DET
            case UART_PATTERN_DET:
                uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                int pos = uart_pattern_pop_pos(EX_UART_NUM);
                ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                if (pos == -1)
                {
                    // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                    // record the position. We should set a larger queue size.
                    // As an example, we directly flush the rx buffer here.
                    uart_flush_input(EX_UART_NUM);
                }
                else
                {
                    uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                    uint8_t pat[PATTERN_CHR_NUM + 1];
                    memset(pat, 0, sizeof(pat));
                    uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                    ESP_LOGI(TAG, "read data: %s", dtmp);
                    ESP_LOGI(TAG, "read pat : %s", pat);
                }
                break;
            // Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

static void configure_led(void)
{
    gpio_set_pull_mode(BLINK_GPIO, GPIO_PULLDOWN_ONLY);
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void configure_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = 2000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, 0, 20, &uart1_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(EX_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(EX_UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Set uart pattern detect function.
    // ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0));
    // Reset the pattern queue length to record at most 20 pattern positions.
    // ESP_ERROR_CHECK(uart_pattern_queue_reset(EX_UART_NUM, 20));
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}

void app_main(void)
{
    configure_led();
    configure_uart();
    // Create a task to handler UART event from ISR
    ESP_LOGI("startup", "startup complete");
}