#ifndef SRC_COMMON_HW_INCLUDE_BMI270_H_
#define SRC_COMMON_HW_INCLUDE_BMI270_H_

#include "hw.h"
#include "sensors.h"
#include "acceleration.h"
#include "gyro.h"

//typedef struct gyroDev_s gyroDev_t;

bool bmi270_Init(void);
bool bmi270Detect(uint8_t ch);
bool bmi270SpiAccDetect(accDev_t *acc);
bool bmi270SpiGyroDetect(gyroDev_t *gyro);
void bmi270Config(void);
bool bmi270SpiAccRead(accDev_t *acc);
bool bmi270SpiGyroRead(gyroDev_t *gyro);
void bmi270SetCallBack(void (*p_func)(void));
uint8_t bmi270InterruptStatus(gyroDev_t *gyro);
void bmi270Intcallback(void);

#endif /* SRC_COMMON_HW_INCLUDE_BMI270_H_ */
