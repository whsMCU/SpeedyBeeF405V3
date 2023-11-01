/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "hw.h"

#include "drivers/barometer/barometer_dps310.h"

#include "hw/i2c.h"

#include "common/utils.h"

#ifdef USE_BARO_DPS310

// 10 MHz max SPI frequency
#define DPS310_MAX_SPI_CLK_HZ 10000000

#define DPS310_I2C_ADDR             0x76

#define DPS310_REG_PSR_B2           0x00
#define DPS310_REG_PSR_B1           0x01
#define DPS310_REG_PSR_B0           0x02
#define DPS310_REG_TMP_B2           0x03
#define DPS310_REG_TMP_B1           0x04
#define DPS310_REG_TMP_B0           0x05
#define DPS310_REG_PRS_CFG          0x06
#define DPS310_REG_TMP_CFG          0x07
#define DPS310_REG_MEAS_CFG         0x08
#define DPS310_REG_CFG_REG          0x09

#define DPS310_REG_RESET            0x0C
#define DPS310_REG_ID               0x0D

#define DPS310_REG_COEF             0x10
#define DPS310_REG_COEF_SRCE        0x28


#define DPS310_ID_REV_AND_PROD_ID       (0x10)

#define DPS310_RESET_BIT_SOFT_RST       (0x09)    // 0b1001

#define DPS310_MEAS_CFG_COEF_RDY        (1 << 7)
#define DPS310_MEAS_CFG_SENSOR_RDY      (1 << 6)
#define DPS310_MEAS_CFG_TMP_RDY         (1 << 5)
#define DPS310_MEAS_CFG_PRS_RDY         (1 << 4)
#define DPS310_MEAS_CFG_MEAS_CTRL_CONT  (0x7)

#define DPS310_PRS_CFG_BIT_PM_RATE_32HZ (0x50)      //  101 - 32 measurements pr. sec.
#define DPS310_PRS_CFG_BIT_PM_PRC_16    (0x04)      // 0100 - 16 times (Standard).

#define DPS310_TMP_CFG_BIT_TMP_EXT          (0x80)  //
#define DPS310_TMP_CFG_BIT_TMP_RATE_32HZ    (0x50)  //  101 - 32 measurements pr. sec.
#define DPS310_TMP_CFG_BIT_TMP_PRC_16       (0x04)  // 0100 - 16 times (Standard).

#define DPS310_CFG_REG_BIT_P_SHIFT          (0x04)
#define DPS310_CFG_REG_BIT_T_SHIFT          (0x08)

#define DPS310_COEF_SRCE_BIT_TMP_COEF_SRCE  (0x80)

typedef struct {
    int16_t c0;     // 12bit
    int16_t c1;     // 12bit
    int32_t c00;    // 20bit
    int32_t c10;    // 20bit
    int16_t c01;    // 16bit
    int16_t c11;    // 16bit
    int16_t c20;    // 16bit
    int16_t c21;    // 16bit
    int16_t c30;    // 16bit
} calibrationCoefficients_t;

typedef struct {
    calibrationCoefficients_t   calib;
    float                       pressure;       // Pa
    float                       temperature;    // DegC
} baroState_t;

static baroState_t  baroState;

static uint8_t buf[6];

// Helper functions
static uint8_t registerRead(const baroDev_t *dev, uint8_t reg)
{
    uint8_t buf_temp[1];
    I2C_ByteRead(dev->address, reg, 1, buf_temp, 1);
    return buf_temp[0];
}

static void registerWrite(const baroDev_t *dev, uint8_t reg, uint8_t value)
{
    I2C_ByteWrite_HAL(dev->address, reg, 1, &value, 1);
}

static void registerSetBits(const baroDev_t *dev, uint8_t reg, uint8_t setbits)
{
    uint8_t val = registerRead(dev, reg);

    if ((val & setbits) != setbits) {
        val |= setbits;
        registerWrite(dev, reg, val);
    }
}

static int32_t getTwosComplement(uint32_t raw, uint8_t length)
{
    if (raw & ((int)1 << (length - 1))) {
        return ((int32_t)raw) - ((int32_t)1 << length);
    }
    else {
        return raw;
    }
}

static bool deviceConfigure(const baroDev_t *dev)
{
    // Trigger a chip reset
    registerSetBits(dev, DPS310_REG_RESET, DPS310_RESET_BIT_SOFT_RST);

    // Sleep 40ms
    delay(40);

    uint8_t status = registerRead(dev, DPS310_REG_MEAS_CFG);

    // Check if coefficients are available
    if ((status & DPS310_MEAS_CFG_COEF_RDY) == 0) {
        return false;
    }

    // Check if sensor initialization is complete
    if ((status & DPS310_MEAS_CFG_SENSOR_RDY) == 0) {
        return false;
    }

    // 1. Read the pressure calibration coefficients (c00, c10, c20, c30, c01, c11, and c21) from the Calibration Coefficient register.
    //   Note: The coefficients read from the coefficient register are 2's complement numbers.
    // Do the read of the coefficients in multiple parts, as the chip will return a read failure when trying to read all at once over I2C.
#define COEFFICIENT_LENGTH 18
#define READ_LENGTH (COEFFICIENT_LENGTH / 2)

    uint8_t coef[COEFFICIENT_LENGTH];
    
    if (!I2C_ByteRead(baro.dev.address, DPS310_REG_COEF, 1, coef, READ_LENGTH)) {
        return false;
    }
    
     if (!I2C_ByteRead(baro.dev.address, DPS310_REG_COEF + READ_LENGTH, 1, coef + READ_LENGTH, COEFFICIENT_LENGTH - READ_LENGTH)) {
        return false;
    }

    // See section 8.11, Calibration Coefficients (COEF), of datasheet

    // 0x11 c0 [3:0] + 0x10 c0 [11:4]
    baroState.calib.c0 = getTwosComplement(((uint32_t)coef[0] << 4) | (((uint32_t)coef[1] >> 4) & 0x0F), 12);

    // 0x11 c1 [11:8] + 0x12 c1 [7:0]
    baroState.calib.c1 = getTwosComplement((((uint32_t)coef[1] & 0x0F) << 8) | (uint32_t)coef[2], 12);

    // 0x13 c00 [19:12] + 0x14 c00 [11:4] + 0x15 c00 [3:0]
    baroState.calib.c00 = getTwosComplement(((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | (((uint32_t)coef[5] >> 4) & 0x0F), 20);

    // 0x15 c10 [19:16] + 0x16 c10 [15:8] + 0x17 c10 [7:0]
    baroState.calib.c10 = getTwosComplement((((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | (uint32_t)coef[7], 20);

    // 0x18 c01 [15:8] + 0x19 c01 [7:0]
    baroState.calib.c01 = getTwosComplement(((uint32_t)coef[8] << 8) | (uint32_t)coef[9], 16);

    // 0x1A c11 [15:8] + 0x1B c11 [7:0]
    baroState.calib.c11 = getTwosComplement(((uint32_t)coef[10] << 8) | (uint32_t)coef[11], 16);

    // 0x1C c20 [15:8] + 0x1D c20 [7:0]
    baroState.calib.c20 = getTwosComplement(((uint32_t)coef[12] << 8) | (uint32_t)coef[13], 16);

    // 0x1E c21 [15:8] + 0x1F c21 [7:0]
    baroState.calib.c21 = getTwosComplement(((uint32_t)coef[14] << 8) | (uint32_t)coef[15], 16);

    // 0x20 c30 [15:8] + 0x21 c30 [7:0]
    baroState.calib.c30 = getTwosComplement(((uint32_t)coef[16] << 8) | (uint32_t)coef[17], 16);

    // PRS_CFG: pressure measurement rate (32 Hz) and oversampling (16 time standard)
    registerSetBits(dev, DPS310_REG_PRS_CFG, DPS310_PRS_CFG_BIT_PM_RATE_32HZ | DPS310_PRS_CFG_BIT_PM_PRC_16);

    // TMP_CFG: temperature measurement rate (32 Hz) and oversampling (16 times)
    const uint8_t TMP_COEF_SRCE = registerRead(dev, DPS310_REG_COEF_SRCE) & DPS310_COEF_SRCE_BIT_TMP_COEF_SRCE;
    registerSetBits(dev, DPS310_REG_TMP_CFG, DPS310_TMP_CFG_BIT_TMP_RATE_32HZ | DPS310_TMP_CFG_BIT_TMP_PRC_16 | TMP_COEF_SRCE);

    // CFG_REG: set pressure and temperature result bit-shift (required when the oversampling rate is >8 times)
    registerSetBits(dev, DPS310_REG_CFG_REG, DPS310_CFG_REG_BIT_T_SHIFT | DPS310_CFG_REG_BIT_P_SHIFT);

    // MEAS_CFG: Continuous pressure and temperature measurement
    registerSetBits(dev, DPS310_REG_MEAS_CFG, DPS310_MEAS_CFG_MEAS_CTRL_CONT);

    return true;
}

static bool dps310ReadUP(baroDev_t *baro)
{
    // if (busBusy(&baro->dev, NULL)) {
    //     return false;
    // }

    // 1. Kick off read
    // No need to poll for data ready as the conversion rate is 32Hz and this is sampling at 20Hz
    // Read PSR_B2, PSR_B1, PSR_B0, TMP_B2, TMP_B1, TMP_B0
     //busReadRegisterBufferStart(&baro->dev, DPS310_REG_PSR_B2, buf, 6);
     I2C_ByteRead(baro->address, DPS310_REG_PSR_B2, 1, buf, 6);

    return true;
}

static bool dps310GetUP(baroDev_t *baro)
{
    UNUSED(baro);

    // 2. Choose scaling factors kT (for temperature) and kP (for pressure) based on the chosen precision rate.
    // The scaling factors are listed in Table 9.
    static float kT = 253952; // 16 times (Standard)
    static float kP = 253952; // 16 times (Standard)

    // 3. Read the pressure and temperature result from the registers

    const int32_t Praw = getTwosComplement((buf[0] << 16) + (buf[1] << 8) + buf[2], 24);
    const int32_t Traw = getTwosComplement((buf[3] << 16) + (buf[4] << 8) + buf[5], 24);

    // 4. Calculate scaled measurement results.
    const float Praw_sc = Praw / kP;
    const float Traw_sc = Traw / kT;

    // 5. Calculate compensated measurement results.
    const float c00 = baroState.calib.c00;
    const float c01 = baroState.calib.c01;
    const float c10 = baroState.calib.c10;
    const float c11 = baroState.calib.c11;
    const float c20 = baroState.calib.c20;
    const float c21 = baroState.calib.c21;
    const float c30 = baroState.calib.c30;

    // See section 4.9.1, How to Calculate Compensated Pressure Values, of datasheet
    baroState.pressure = c00 + Praw_sc * (c10 + Praw_sc * (c20 + Praw_sc * c30)) + Traw_sc * c01 + Traw_sc * Praw_sc * (c11 + Praw_sc * c21);

    const float c0 = baroState.calib.c0;
    const float c1 = baroState.calib.c1;

    // See section 4.9.2, How to Calculate Compensated Temperature Values, of datasheet
    baroState.temperature = c0 * 0.5f + c1 * Traw_sc;

    return true;
}

static void deviceCalculate(int32_t *pressure, int32_t *temperature)
{
    if (pressure) {
        *pressure = baroState.pressure; 
    }

    if (temperature) {
        *temperature = (baroState.temperature * 100);   // to centidegrees
    }
}



#define DETECTION_MAX_RETRY_COUNT   5
static bool deviceDetect(const baroDev_t *dev)
{
    for (int retry = 0; retry < DETECTION_MAX_RETRY_COUNT; retry++) {
        uint8_t chipId[1];

        delay(100);

        bool ack = I2C_ByteRead(dev->address, DPS310_REG_ID, 1, chipId, 1);

        if (ack && chipId[0] == DPS310_ID_REV_AND_PROD_ID) {
            return true;
        }
    };

    return false;
}

static void dps310StartUT(baroDev_t *baro)
{
    UNUSED(baro);
}

static bool dps310ReadUT(baroDev_t *baro)
{
    UNUSED(baro);

    return true;
}

static bool dps310GetUT(baroDev_t *baro)
{
    UNUSED(baro);

    return true;
}

static void dps310StartUP(baroDev_t *baro)
{
    UNUSED(baro);
}

bool dps310Detect(baroDev_t *baro)
{
    baro->address = DPS310_I2C_ADDR;

    if (!deviceDetect(baro)) {
        return false;
    }

    if (!deviceConfigure(baro)) {
        return false;
    }

    baro->ut_delay = 0;
    baro->start_ut = dps310StartUT;
    baro->read_ut = dps310ReadUT;
    baro->get_ut = dps310GetUT;

    baro->up_delay = 45000; // 45ms delay plus 5 1ms cycles 50ms
    baro->start_up = dps310StartUP;
    baro->read_up = dps310ReadUP;
    baro->get_up = dps310GetUP;

    baro->calculate = deviceCalculate;

    return true;
}

#endif // USE_ACCGYRO_DSP310
