/* UART Echo Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

#define ECHO_TEST_UART_NUM_0_TXD (1)
#define ECHO_TEST_UART_NUM_0_RXD (3)
#define ECHO_TEST_UART_NUM_0_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_UART_NUM_0_CTS (UART_PIN_NO_CHANGE)

#define ECHO_TEST_UART_NUM_1_TXD (10)
#define ECHO_TEST_UART_NUM_1_RXD (9)
#define ECHO_TEST_UART_NUM_1_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_UART_NUM_1_CTS (UART_PIN_NO_CHANGE)

#define ECHO_TEST_UART_NUM_2_TXD (UART_PIN_NO_CHANGE)	//FIXME
#define ECHO_TEST_UART_NUM_2_RXD (4)
#define ECHO_TEST_UART_NUM_2_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_UART_NUM_2_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_BAUD_RATE     (115200)
#define ECHO_TASK_STACK_SIZE    (2048)

#define BUF_SIZE (1024)

uint8_t data[BUF_SIZE];

static void echo_task(void *arg)
{
	int len;

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config =
    {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    int intr_alloc_flags = 0;
	#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
	#endif

    //Install driver for UART0
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, ECHO_TEST_UART_NUM_0_TXD, ECHO_TEST_UART_NUM_0_RXD, ECHO_TEST_UART_NUM_0_RTS, ECHO_TEST_UART_NUM_0_CTS));

    //Install driver for UART1
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, ECHO_TEST_UART_NUM_1_TXD, ECHO_TEST_UART_NUM_1_RXD, ECHO_TEST_UART_NUM_1_RTS, ECHO_TEST_UART_NUM_1_CTS));

    //Install driver for UART2
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, ECHO_TEST_UART_NUM_2_TXD, ECHO_TEST_UART_NUM_2_RXD, ECHO_TEST_UART_NUM_2_RTS, ECHO_TEST_UART_NUM_2_CTS));

    while (1)
    {
        // Read data from the UART0
        len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        if(len > 0)
        {
            // Write data back to the UART
            uart_write_bytes(UART_NUM_0, (const char *) data, len);
        }

        // Read data from the UART1
        len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        if(len > 0)
        {
            // Write data back to the UART
            uart_write_bytes(UART_NUM_1, (const char *) data, len);
        }

        // Read data from the UART2
        len = uart_read_bytes(UART_NUM_2, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        if(len > 0)
        {
            // Write data back to the UART
            uart_write_bytes(UART_NUM_2, (const char *) data, len);
        }
    }
}

void app_main(void)
{
    xTaskCreate(echo_task, "test_remap_uarts", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);
}
