/*
 * def.h
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */

#ifndef SRC_COMMON_DEF_H_
#define SRC_COMMON_DEF_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#define _DEF_LED1           0
#define _DEF_LED2           1
#define _DEF_LED3           2
#define _DEF_LED4           3

#define _DEF_USB            0
#define _DEF_UART1          1
#define _DEF_UART2          2
#define _DEF_UART3          3
#define _DEF_UART4          4
#define _DEF_UART5          5
#define _DEF_UART6          6

#define _DEF_SPI1             0
#define _DEF_SPI2             1
#define _DEF_SPI3             2
#define _DEF_SPI4             3

#define _DEF_LOW              0
#define _DEF_HIGH             1

#define _DEF_INPUT            0
#define _DEF_INPUT_PULLUP     1
#define _DEF_INPUT_PULLDOWN   2
#define _DEF_INPUT_IT_RISING  3
#define _DEF_OUTPUT           4
#define _DEF_OUTPUT_PULLUP    5
#define _DEF_OUTPUT_PULLDOWN  6
#define _DEF_INPUT_AF_PP      7


#define MAX_SUPPORTED_MOTORS 8


#define USE_HAL_DRIVER


#define USE_GYRO_OVERFLOW_CHECK
#define USE_YAW_SPIN_RECOVERY
#define USE_ACC
#define USE_BARO
#define USE_MAG
#define USE_DYN_LPF
#define USE_D_MIN
#define USE_LATE_TASK_STATISTICS
#define USE_DYN_NOTCH_FILTER
#define USE_SERIAL_RX
#define USE_ADC
#define CONFIG_IN_FLASH
//#define USE_TELEMETRY
#define USE_MSP_OVER_TELEMETRY
//#define USE_TELEMETRY_CRSF
//#define USE_CRSF_V3
//#define USE_CRSF_LINK_STATISTICS
//#define USE_DYN_IDLE
//#define USE_GYRO_DLPF_EXPERIMENTAL
#define USE_ADC_INTERNAL
#define USE_RC_SMOOTHING_FILTER
#define USE_FEEDFORWARD
#define USE_OSD
#define USE_BOARD_INFO
#define USE_SIGNATURE
#define USE_MOTOR
#define USE_PWM_OUTPUT


#endif /* SRC_COMMON_DEF_H_ */
