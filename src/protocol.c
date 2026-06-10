/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "protocol.h"
#include "gpio.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include <stdlib.h>


#define UART_ID   uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define BUFFER_SIZE 64

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN gpioGetPinNumber(GPIO_ESP32_UART_TX)
#define UART_RX_PIN gpioGetPinNumber(GPIO_ESP32_UART_RX)

static StreamBufferHandle_t xCommsStreamBuffer = NULL;
static TaskHandle_t protocolTaskHandle = NULL;

// RX interrupt handler
static void on_uart_rx() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    while (uart_is_readable(UART_ID)) {
        char ch = uart_getc(UART_ID);
        xStreamBufferSendFromISR(xCommsStreamBuffer, &ch, 1, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void uartInit() {
    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, 2400);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    uart_set_fifo_enabled(UART_ID, true);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
}

static void parceCommand(const char* command) {
    printf("Received command: %s\n", command);

    switch (command[0]) {
    case 'P':
        float kP = strtof(&command[1], NULL);
        printf("kP: %f\n", kP);
        break;
    case 'D':
        float kD = strtof(&command[1], NULL);
        printf("kD: %f\n", kD);
        break;
    case 'I':
        float kI = strtof(&command[1], NULL);
        printf("kI: %f\n", kI);
        break;
    default:
        break;
    }
    // TODO: O.T Implement command parsing and handling logic here
}

static void protocolTask(void *params) {
    printf("Protocol Task Started\n");

    char rx_buff[BUFFER_SIZE] = "\0";
    int idx = 0;
    char ch = '\0';

    for (;;) {
        if (xStreamBufferReceive(xCommsStreamBuffer, &ch, 1, portMAX_DELAY)) {
            if (idx < BUFFER_SIZE - 1) {
                rx_buff[idx++] = ch;
            }

            if (ch == '\n' || ch == '\r') {
                rx_buff[idx] = '\0';
                if (idx > 1) {
                    parceCommand(rx_buff);
                }
                idx = 0;
            }
        }
    }
}

void protocolInit(void) {
    // Make sure the stream buffer is created before we init the UART
    xCommsStreamBuffer = xStreamBufferCreate(BUFFER_SIZE, 1);

    BaseType_t status = xTaskCreate(
        protocolTask,
        "protocolTask",
        1024,
        NULL,
        2,
        &protocolTaskHandle
    );

    configASSERT(status == pdPASS);

    uartInit();
}
