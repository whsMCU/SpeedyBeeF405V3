/*
 * uart.h
 *
 *  Created on: 2020. 12. 8.
 *      Author: baram
 */

#ifndef SRC_COMMON_HW_INCLUDE_UART_H_
#define SRC_COMMON_HW_INCLUDE_UART_H_

#include "hw.h"

#define UART_MAX_CH         HW_UART_MAX_CH

typedef enum {
    BAUD_AUTO = 0,
    BAUD_9600,
    BAUD_19200,
    BAUD_38400,
    BAUD_57600,
    BAUD_115200,
    BAUD_230400,
    BAUD_250000,
    BAUD_400000,
    BAUD_460800,
    BAUD_500000,
    BAUD_921600,
    BAUD_1000000,
    BAUD_1500000,
    BAUD_2000000,
    BAUD_2470000,
    BAUD_COUNT
} baudRate_e;

extern const uint32_t baudRates[];

bool     uartInit(void);
bool     uartOpen(uint8_t ch, uint32_t baud);
uint32_t uartAvailable(uint8_t ch);
uint8_t  uartRead(uint8_t ch);
uint32_t uartWrite(uint8_t ch, uint8_t *p_data, uint32_t length);
uint32_t uartWriteIT(uint8_t ch, uint8_t *p_data, uint32_t length);
void serialPrint(uint8_t channel, const char *str);
uint32_t uartPrintf(uint8_t ch, char *fmt, ...);
uint32_t uartPrintf_IT(uint8_t ch, char *fmt, ...);
uint32_t uartGetBaud(uint8_t ch);
bool uartSetBaud(uint8_t ch, uint32_t baud);
baudRate_e lookupBaudRateIndex(uint32_t baudRate);


#endif /* SRC_COMMON_HW_INCLUDE_UART_H_ */
