/*
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */

#ifndef SRC_HW_HW_DEF_H_
#define SRC_HW_HW_DEF_H_


#define _USE_HW_SD
#define _USE_HW_FATFS
//#define _USE_HW_FILES
#define _USE_HW_FLASH
#define _USE_HW_RTC
//#define USE_SDCARD
//#define USE_SDCARD_SPI

#define USE_ACCGYRO_BMI270

//SPI-------
#define BMI270  0
#define SDCARD  1
#define MAX7456 2

#define _PIN_BMI270_CS 0
#define _PIN_SDCARD_CS 3
#define _PIN_MAX7456_CS 4
//----------

#define USE_BARO_DPS310
#define USE_MAG_QMC5883

#define _USE_HW_CDC
#define _USE_HW_USB
#define _USE_HW_I2C
#define _USE_HW_TIMER

#define _USE_HW_SPI
#define      HW_SPI_MAX_CH          3

#define _USE_HW_GPIO
#define      HW_GPIO_MAX_CH         5


#define _USE_HW_LED
#define      HW_LED_MAX_CH          1

#define _USE_HW_UART
#define      HW_UART_MAX_CH         7


#define _USE_HW_CLI
#define      HW_CLI_CMD_NAME_MAX    16
#define      HW_CLI_CMD_LIST_MAX    16
#define      HW_CLI_LINE_HIS_MAX    4
#define      HW_CLI_LINE_BUF_MAX    32

#define _PIN_GPIO_SDCARD_DETECT     0



#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

#define D2R (3.141592653f / 180.0f)
#define R2D (180.0f / 3.141592653f)

#define TRUE 1
#define FALSE 0

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
//#define abs(x) ((x) > 0 ? (x) : -(x))
#define zofs(x, y, z) ((x) > (y+z) ? (x) : ((x) < (y-z) ? (x) : (y)))
#define map(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

#define ROUND(x, dig)  ( floor((x) * pow((float)(10), dig) + 0.5f) / pow((float)(10), dig) )

#define CONVERT_PARAMETER_TO_FLOAT(param) (0.001f * param)
#define CONVERT_PARAMETER_TO_PERCENT(param) (0.01f * param)


#endif /* SRC_HW_HW_DEF_H_ */
