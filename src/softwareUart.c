/*
 * Softuart
 *
 * Copyright (C) 2017 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (C) 2016 Bernhard Guillon <Bernhard.Guillon@web.de>
 *
 * This code is based on Softuart from here [1] and reworked to
 * fit into esp-open-rtos.
 *
 * it fits my needs to read the GY-GPS6MV2 module with 9600 8n1
 *
 * Original Copyright:
 * Copyright (c) 2015 plieningerweb
 *
 * MIT Licensed as described in the file LICENSE
 *
 * 1 https://github.com/plieningerweb/esp8266-software-uart
 */

#include "softwareUart.h"
#include <stdint.h>
#include <esp/gpio.h>
#include <espressif/esp_common.h>
#include "portmacro.h"
#include <stdio.h>
#include <stdlib.h>


static void irq_handle_rx(uint8_t gpio_num);


static softwareUart_t* uarts[10] = { NULL };


softwareUart_t* softwareUart_open(uint32_t baudrate, uint8_t rx_pin, uint8_t tx_pin)
{
	softwareUart_t* uart = NULL;

	if(rx_pin >= (sizeof(uarts)/sizeof(softwareUart_t*)))
		return NULL;

	if(uarts[rx_pin] != NULL)
		return NULL;

	uart = calloc(1, sizeof(softwareUart_t));
	uarts[rx_pin] = uart;

    uart->baudrate = baudrate;
    uart->rx_pin = rx_pin;
    uart->tx_pin = tx_pin;

    // Calculate bit_time
    uart->bit_time = (1000000 / baudrate);
    if(((100000000 / baudrate) - (100 * uart->bit_time)) > 50)
    	uart->bit_time++;

    // Setup Rx
    gpio_enable(rx_pin, GPIO_INPUT);
    gpio_set_pullup(rx_pin, true, false);

    // Setup Tx
    gpio_enable(tx_pin, GPIO_OUTPUT);
    gpio_set_pullup(tx_pin, true, false);
    gpio_write(tx_pin, 1);

    gpio_set_interrupt(rx_pin, GPIO_INTTYPE_EDGE_NEG, irq_handle_rx);

    return uart;
}

void softwareUart_disableRx(softwareUart_t* uart)
{
	gpio_set_interrupt(uart->rx_pin, GPIO_INTTYPE_NONE, NULL);
}

void softwareUart_enableRx(softwareUart_t* uart)
{
	gpio_set_interrupt(uart->rx_pin, GPIO_INTTYPE_EDGE_NEG, irq_handle_rx);
}

void softwareUart_close(softwareUart_t* uart)
{
    gpio_set_interrupt(uart->rx_pin, GPIO_INTTYPE_NONE, NULL);

    if(uarts[uart->rx_pin])
    {
    	free(uarts[uart->rx_pin]);
    	uarts[uart->rx_pin] = NULL;
    }
}

void softwareUart_put(softwareUart_t* uart, char byte)
{
	portDISABLE_INTERRUPTS();

    uint32_t start_time = sdk_system_get_time();
    gpio_write(uart->tx_pin, 0);

    for (uint8_t i = 0; i <= 8; i++)
    {
        while (sdk_system_get_time() < (start_time + (uart->bit_time * (i + 1))))
        {
            if (sdk_system_get_time() < start_time)
                break;
        }
        gpio_write(uart->tx_pin, byte & (1 << i));
    }

    while (sdk_system_get_time() < (start_time + (uart->bit_time * 9)))
    {
        if (sdk_system_get_time() < start_time)
            break;
    }

    gpio_write(uart->tx_pin, 1);

    sdk_os_delay_us(uart->bit_time * 6);

    portENABLE_INTERRUPTS();
}

uint8_t softwareUart_bytesAvailable(softwareUart_t* uart)
{
    return (uart->buffer.tail + UART_RX_BUFFER_SIZE - uart->buffer.head) % UART_RX_BUFFER_SIZE;
}

uint8_t softwareUart_read(softwareUart_t* uart)
{
    // Empty buffer?
    if (uart->buffer.head == uart->buffer.tail)
    	return 0;

    // Read from "head"
    uint8_t byte = uart->buffer.data[uart->buffer.head]; // grab next byte
    uart->buffer.head = (uart->buffer.head + 1) % UART_RX_BUFFER_SIZE;

    return byte;
}

void softwareUart_dumpRxBuffer(softwareUart_t* uart)
{
	uart->buffer.head = uart->buffer.tail = 0;
}


// GPIO interrupt handler
static void irq_handle_rx(uint8_t gpio_num)
{
    if(uarts[gpio_num] == NULL)
    	return;

    softwareUart_t* uart = uarts[gpio_num];

    // Disable interrupt
    gpio_set_interrupt(gpio_num, GPIO_INTTYPE_NONE, irq_handle_rx);

    // Wait till start bit is half over so we can sample the next one in the center
    sdk_os_delay_us(uart->bit_time / 2);

    // Now sample bits
    uint8_t byte = 0;
    uint32_t start_time = sdk_system_get_time();

    for (uint8_t i = 0; i < 8; i++)
    {
        while (sdk_system_get_time() < (start_time + (uart->bit_time * (i + 1))))
        {
            // If system timer overflow, escape from while loop
            if (sdk_system_get_time() < start_time)
                break;
        }
        // Shift d to the right
        byte >>= 1;

        // Read bit
        if (gpio_read(uart->rx_pin))
        {
            // If high, set msb of 8bit to 1
            byte |= 0x80;
        }
    }

    // Store byte in buffer
    // If buffer full, set the overflow flag and return
    uint8_t next = (uart->buffer.tail + 1) % UART_RX_BUFFER_SIZE;

    if (next != uart->buffer.head)
    {
        // save new data in buffer: tail points to where byte goes
        uart->buffer.data[uart->buffer.tail] = byte; // save new byte
        uart->buffer.tail = next;
    }
    else
    {
        uart->buffer.overflow = 1;
    }

    // Wait for stop bit
    sdk_os_delay_us(uart->bit_time);

    // Done, reenable interrupt
    gpio_set_interrupt(uart->rx_pin, GPIO_INTTYPE_EDGE_NEG, irq_handle_rx);
}


