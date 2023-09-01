#pragma once

#include "common/axis.h"
#include "common/maths.h"
#include "common/time.h"

#include "drivers/sensor.h"


#define GYRO_SCALE_2000DPS (2000.0f / (1 << 15))   // 16.384 dps/lsb scalefactor for 2000dps sensors
#define GYRO_SCALE_4000DPS (4000.0f / (1 << 15))   //  8.192 dps/lsb scalefactor for 4000dps sensors

typedef enum {
    GYRO_NONE = 0,
    GYRO_DEFAULT,
    GYRO_MPU6050,
    GYRO_L3G4200D,
    GYRO_MPU3050,
    GYRO_L3GD20,
    GYRO_MPU6000,
    GYRO_MPU6500,
    GYRO_MPU9250,
    GYRO_ICM20601,
    GYRO_ICM20602,
    GYRO_ICM20608G,
    GYRO_ICM20649,
    GYRO_ICM20689,
    GYRO_ICM42605,
    GYRO_ICM42688P,
    GYRO_BMI160,
    GYRO_BMI270,
    GYRO_LSM6DSO,
    GYRO_FAKE
} gyroHardware_e;

typedef enum {
    GYRO_HARDWARE_LPF_NORMAL,
    GYRO_HARDWARE_LPF_OPTION_1,
    GYRO_HARDWARE_LPF_OPTION_2,
#ifdef USE_GYRO_DLPF_EXPERIMENTAL
    GYRO_HARDWARE_LPF_EXPERIMENTAL,
#endif
    GYRO_HARDWARE_LPF_COUNT
} gyroHardwareLpf_e;

typedef enum {
    GYRO_RATE_1_kHz,
    GYRO_RATE_1100_Hz,
    GYRO_RATE_3200_Hz,
    GYRO_RATE_6400_Hz,
    GYRO_RATE_6664_Hz,
    GYRO_RATE_8_kHz,
    GYRO_RATE_9_kHz,
    GYRO_RATE_32_kHz,
} gyroRateKHz_e;

typedef enum {
    GYRO_EXTI_INIT = 0,
    GYRO_EXTI_INT_DMA,
    GYRO_EXTI_INT,
    GYRO_EXTI_NO_INT
} gyroModeSPI_e;

typedef struct gyroDev_s {
    uint8_t  gyro_bus_ch;
    sensorGyroInitFuncPtr initFn;                             // initialize function
    sensorGyroReadFuncPtr readFn;                             // read 3 axis data function
    sensorGyroReadDataFuncPtr temperatureFn;                  // read temperature if available
    //extiCallbackRec_t exti;
    //extDevice_t dev;
    uint8_t *txBuf, *rxBuf;
    float scale;                                             // scalefactor
    float gyroZero[XYZ_AXIS_COUNT];
    float gyroADC[XYZ_AXIS_COUNT];                           // gyro data after calibration and alignment
    int32_t gyroADCRawPrevious[XYZ_AXIS_COUNT];
    int16_t gyroADCRaw[XYZ_AXIS_COUNT];                      // raw data from sensor
    int16_t temperature;
    //mpuDetectionResult_t mpuDetectionResult;
    //sensor_align_e gyroAlign;
    gyroRateKHz_e gyroRateKHz;
    gyroModeSPI_e gyroModeSPI;

    uint32_t detectedEXTI;
    uint32_t gyroLastEXTI;
    uint32_t gyroSyncEXTI;
    int32_t gyroShortPeriod;
    int32_t gyroDmaMaxDuration;
    uint32_t exit_callback_dt;
    uint32_t rx_callback_dt;
    //busSegment_t segments[2];

    volatile bool dataReady;
    bool gyro_high_fsr;
    uint8_t hardware_lpf;
    uint8_t hardware_32khz_lpf;
    //uint8_t mpuDividerDrops;
    //ioTag_t mpuIntExtiTag;
    uint8_t gyroHasOverflowProtection;
    gyroHardware_e gyroHardware;
    fp_rotationMatrix_t rotationMatrix;
    uint16_t gyroSampleRateHz;
    uint16_t accSampleRateHz;
    uint8_t accDataReg;
    uint8_t gyroDataReg;
} gyroDev_t;

typedef struct accDev_s {
    float acc_1G_rec;
    sensorAccInitFuncPtr initFn;                              // initialize function
    sensorAccReadFuncPtr readFn;                              // read 3 axis data function
    uint16_t acc_1G;
    int16_t ADCRaw[XYZ_AXIS_COUNT];
    //mpuDetectionResult_t mpuDetectionResult;
    //sensor_align_e accAlign;
    bool dataReady;
    gyroDev_t *gyro;
    bool acc_high_fsr;
    char revisionCode;                                      // a revision code for the sensor, if known
    uint8_t filler[2];
    //fp_rotationMatrix_t rotationMatrix;
} accDev_t;
