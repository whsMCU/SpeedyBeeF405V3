/*
 * i2c.h
 *
 *  Created on: 2020. 12. 20.
 *      Author: WANG
 */

#ifndef SRC_COMMON_HW_INCLUDE_SPI_H_
#define SRC_COMMON_HW_INCLUDE_SPI_H_

#include "hw.h"


#ifdef _USE_HW_SPI

#define SPI_MAX_CH          HW_SPI_MAX_CH


#define SPI_MODE0           0
#define SPI_MODE1           1
#define SPI_MODE2           2
#define SPI_MODE3           3

#define SPI_SPEED_CLOCK_DIV2_MHZ    ((uint32_t)2)
#define SPI_SPEED_CLOCK_DIV4_MHZ    ((uint32_t)4)
#define SPI_SPEED_CLOCK_DIV8_MHZ    ((uint32_t)8)
#define SPI_SPEED_CLOCK_DIV16_MHZ   ((uint32_t)16)
#define SPI_SPEED_CLOCK_DIV32_MHZ   ((uint32_t)32)
#define SPI_SPEED_CLOCK_DIV64_MHZ   ((uint32_t)64)
#define SPI_SPEED_CLOCK_DIV128_MHZ  ((uint32_t)128)
#define SPI_SPEED_CLOCK_DIV256_MHZ  ((uint32_t)256)

typedef enum {
    BUS_READY,
    BUS_BUSY,
    BUS_ABORT
} busStatus_e;

bool spiInit(void);
bool spiDev_Init(void);
bool spiBegin(uint8_t dev);
bool spiIsBegin(uint8_t dev);
void spiSetDataMode(uint8_t dev, uint8_t dataMode);
void spiSetBitWidth(uint8_t dev, uint8_t bit_width);

bool SPI_Set_Speed_hz(uint8_t dev, uint32_t speed);

uint32_t SPI_Get_Speed(uint8_t dev);
bool SPI_Set_Speed(uint8_t dev, uint32_t prescaler);
void spiSetClkDivisor(uint8_t dev, uint32_t prescaler);
uint16_t spiCalculateDivider(uint32_t freq);

uint8_t spiReadReg(uint8_t dev, uint8_t reg);
uint8_t spiReadRegMsk(uint8_t dev, uint8_t reg);
void spiWriteReg(uint8_t dev, uint8_t reg, uint8_t data);
void spiWrite(uint8_t dev, uint8_t data);
void spiWait(uint8_t dev);
bool spiRx_flag(uint8_t dev);
bool spiIsBusy(uint8_t dev);
void spiReadWriteBuf(uint8_t dev, uint8_t *txData, uint8_t *rxData, int len);
void spiWriteRegBuf(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
bool spiReadRegMskBufRB(uint8_t dev, uint8_t reg, uint8_t *data, uint8_t length);

HAL_StatusTypeDef SPI_ByteRead(uint8_t dev, uint8_t MemAddress, uint8_t *data, uint8_t length);
HAL_StatusTypeDef SPI_ByteWrite(uint8_t ch, uint8_t MemAddress, uint8_t *data, uint32_t length);
HAL_StatusTypeDef SPI_ByteRead_DMA(uint8_t dev, uint8_t *MemAddress, uint8_t *data, uint8_t length);
HAL_StatusTypeDef SPI_ByteReadWrite_DMA(uint8_t dev, uint8_t *MemAddress, uint8_t *data, uint8_t length);
HAL_StatusTypeDef SPI_ByteWrite_DMA(uint8_t dev, uint8_t *data, uint8_t length);


bool spiTransfer(uint8_t dev, uint8_t *tx_buf, uint8_t *rx_buf, uint32_t length, uint32_t timeout);

#endif

#endif /* SRC_COMMON_HW_INCLUDE_I2C_H_ */
