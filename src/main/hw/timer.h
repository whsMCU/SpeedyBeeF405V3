/*
 * timer.h
 *
 *  Created on: 2020. 12. 26.
 *      Author: WANG
 */

#ifndef SRC_COMMON_HW_INCLUDE_TIMER_H_
#define SRC_COMMON_HW_INCLUDE_TIMER_H_

#include "hw.h"

#ifdef _USE_HW_TIMER

#include "drivers/pwm_output.h"

extern TIM_HandleTypeDef htim4;

typedef enum {
    TIMER_OUTPUT_NONE      = 0,
    TIMER_OUTPUT_INVERTED  = (1 << 0),
    TIMER_OUTPUT_N_CHANNEL = (1 << 1),
} timerFlag_e;

bool timerInit(pwmOutputPort_t *motors, uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion);

#endif

#endif /* SRC_COMMON_HW_INCLUDE_TIMER_H_ */
