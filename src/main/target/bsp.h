/*
 * bsp.h
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */

#ifndef SRC_BSP_BSP_H_
#define SRC_BSP_BSP_H_


#include "stm32f4xx_hal.h"

#define _USE_LOG_PRINT    1

#if _USE_LOG_PRINT
#define logPrintf(fmt, ...)     printf(fmt, ##__VA_ARGS__)
#else
#define logPrintf(fmt, ...)
#endif

void cycleCounterInit(void);
uint32_t getCycleCounter(void);

int32_t clockCyclesToMicros(int32_t clockCycles);
int32_t clockCyclesTo10thMicros(int32_t clockCycles);
uint32_t clockMicrosToCycles(uint32_t micros);

void delay(uint32_t ms);
uint32_t millis(void);
uint32_t micros(void);
void delayMicroseconds(uint32_t us);

#endif /* SRC_BSP_BSP_H_ */
