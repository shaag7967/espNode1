/*
 * Softuart for esp-open-rtos
 *
 * Copyright (C) 2017 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (C) 2016 Bernhard Guillon <Bernhard.Guillon@web.de>
 *
 * This code is based on Softuart from here [1] and reworked to
 * fit into esp-open-rtos. For now only the RX part is ported.
 * Also the configuration of the pin is for now hardcoded.
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
#ifndef SOFTWAREUART_H_
#define SOFTWAREUART_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif


#define UART_RX_BUFFER_SIZE     64 // has to be power of 2!

typedef struct softwareUart_s
{
	uint32_t baudrate;
	uint8_t rx_pin;
	uint8_t tx_pin;
	uint16_t bit_time;

	volatile struct rxBuffer
	{
	    char data[UART_RX_BUFFER_SIZE];
	    uint8_t tail;
	    uint8_t head;
	    uint8_t overflow;
	} buffer;
} softwareUart_t;


softwareUart_t* softwareUart_open(uint32_t baudrate, uint8_t rx_pin, uint8_t tx_pin);
void softwareUart_close(softwareUart_t* uart);
void softwareUart_put(softwareUart_t* uart, char c);
uint8_t softwareUart_bytesAvailable(softwareUart_t* uart);
uint8_t softwareUart_read(softwareUart_t* uart);
void softwareUart_dumpRxBuffer(softwareUart_t* uart);

void softwareUart_disableRx(softwareUart_t* uart);
void softwareUart_enableRx(softwareUart_t* uart);


#ifdef __cplusplus
}
#endif

#endif /* SOFTWAREUART_H_ */
