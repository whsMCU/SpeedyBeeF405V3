/*
 * led.h
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */

#ifndef SRC_COMMON_HW_INCLUDE_LED_H_
#define SRC_COMMON_HW_INCLUDE_LED_H_


#include "hw.h"


#ifdef _USE_HW_LED

#define LED_MAX_CH          HW_LED_MAX_CH

typedef enum
{
  ST1 = 0u,
  ST2,
} LED_Type;

#define LED0_TOGGLE              ledToggle(0)
#define LED0_OFF                 ledOff(0)
#define LED0_ON                  ledOn(0)

#define LED1_TOGGLE              ledToggle(1)
#define LED1_OFF                 ledOff(1)
#define LED1_ON                  ledOn(1)

#define LED2_TOGGLE              ledToggle(2)
#define LED2_OFF                 ledOff(2)
#define LED2_ON                  ledOn(2)


bool ledInit(void);
void ledOn(uint8_t ch);
void ledOff(uint8_t ch);
void ledToggle(uint8_t ch);

#endif

#endif /* SRC_COMMON_HW_INCLUDE_LED_H_ */
