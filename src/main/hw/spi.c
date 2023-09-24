/*
 * i2c.c
 *
 *  Created on: 2020. 12. 20.
 *      Author: WANG
 */


#include "spi.h"
#include "drivers/accgyro/bmi270.h"


typedef struct
{
  bool is_open;
  bool is_tx_done;
  bool is_rx_done;
  bool is_error;

  void (*func_tx)(void);
  void (*func_rx)(void);

  uint8_t ch;

  SPI_HandleTypeDef *h_spi;
  DMA_HandleTypeDef *h_dma_tx;
  DMA_HandleTypeDef *h_dma_rx;
} spi_t;

typedef struct
{
	spi_t dev;
	uint8_t mode;
	uint8_t csTag;
} spi_dev_t;

spi_dev_t spi_dev_tbl[SPI_MAX_CH];

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

static void cliSPI(cli_args_t *args);

bool spiDev_Init(void)
{
	bool ret = true;
	spi_dev_tbl[BMI270].dev.ch  = _DEF_SPI1;
	spi_dev_tbl[BMI270].mode    = SPI_MODE3;
	spi_dev_tbl[BMI270].csTag   = _PIN_BMI270_CS;

	spi_dev_tbl[SDCARD].dev.ch  = _DEF_SPI2;
	spi_dev_tbl[SDCARD].mode    = SPI_MODE0;
	spi_dev_tbl[SDCARD].csTag   = _PIN_SDCARD_CS;

	spi_dev_tbl[MAX7456].dev.ch = _DEF_SPI2;
	spi_dev_tbl[MAX7456].mode   = SPI_MODE0;
	spi_dev_tbl[MAX7456].csTag  = _PIN_MAX7456_CS;
	return ret;
}

bool spiInit(void)
{
  bool ret = true;

  spiDev_Init();

  for (int i=0; i<SPI_MAX_CH; i++)
  {
	  spi_dev_tbl[i].dev.is_open = false;
	  spi_dev_tbl[i].dev.is_tx_done = true;
	  spi_dev_tbl[i].dev.is_rx_done = true;
	  spi_dev_tbl[i].dev.is_error = false;
	  spi_dev_tbl[i].dev.func_tx = NULL;
	  spi_dev_tbl[i].dev.func_rx = NULL;
	  spi_dev_tbl[i].dev.h_dma_rx = NULL;
	  spi_dev_tbl[i].dev.h_dma_tx = NULL;
  }
  spiBegin(BMI270);
  spiBegin(MAX7456);
  cliAdd("spi", cliSPI);
  return ret;
}

bool spiBegin(uint8_t dev)
{
  bool ret = false;

  switch(dev)
  {
    case BMI270:
      spi_dev_tbl[BMI270].dev.h_spi = &hspi1;
      spi_dev_tbl[BMI270].dev.h_dma_tx = &hdma_spi1_tx;
      spi_dev_tbl[BMI270].dev.h_dma_rx = &hdma_spi1_rx;
      hspi1.Instance = SPI1;
      hspi1.Init.Mode = SPI_MODE_MASTER;
      hspi1.Init.Direction = SPI_DIRECTION_2LINES;
      hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
      hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
      hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
      hspi1.Init.NSS = SPI_NSS_SOFT;
      hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
      hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
      hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
      hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
      hspi1.Init.CRCPolynomial = 10;

      //HAL_SPI_DeInit(&hspi1);

      if (HAL_SPI_Init(&hspi1) == HAL_OK)
      {
    	spi_dev_tbl[BMI270].dev.is_open = true;
        ret = true;
      }
      break;

    case SDCARD:
	  spi_dev_tbl[SDCARD].dev.h_spi = &hspi2;
	  spi_dev_tbl[SDCARD].dev.h_dma_tx = &hdma_spi2_tx;
	  spi_dev_tbl[SDCARD].dev.h_dma_rx = &hdma_spi2_rx;
      hspi2.Instance = SPI2;
      hspi2.Init.Mode = SPI_MODE_MASTER;
      hspi2.Init.Direction = SPI_DIRECTION_2LINES;
      hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
      hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
      hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
      hspi2.Init.NSS = SPI_NSS_SOFT;
      hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
      hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
      hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
      hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
      hspi2.Init.CRCPolynomial = 10;
      //HAL_SPI_DeInit(&hspi2);
      if (HAL_SPI_Init(&hspi2) == HAL_OK)
      {
    	spi_dev_tbl[SDCARD].dev.is_open = true;
        ret = true;
      }
      break;


    case MAX7456:
  	  spi_dev_tbl[MAX7456].dev.h_spi = &hspi2;
  	  spi_dev_tbl[MAX7456].dev.h_dma_tx = &hdma_spi2_tx;
  	  spi_dev_tbl[MAX7456].dev.h_dma_rx = &hdma_spi2_rx;
      hspi2.Instance = SPI2;
      hspi2.Init.Mode = SPI_MODE_MASTER;
      hspi2.Init.Direction = SPI_DIRECTION_2LINES;
      hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
      hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
      hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
      hspi2.Init.NSS = SPI_NSS_SOFT;
      hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
      hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
      hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
      hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
      hspi2.Init.CRCPolynomial = 10;
      //HAL_SPI_DeInit(&hspi2);
      if (HAL_SPI_Init(&hspi2) == HAL_OK)
      {
    	spi_dev_tbl[MAX7456].dev.is_open = true;
        ret = true;
      }
      break;
  }

  return ret;
}

bool spiIsBegin(uint8_t dev)
{
  return spi_dev_tbl[dev].dev.is_open;
}

void spiSetDataMode(uint8_t dev, uint8_t dataMode)
{
	spi_t  *p_spi = &spi_dev_tbl[dev].dev;


  if (p_spi->is_open == false) return;


  switch( dataMode )
  {
    // CPOL=0, CPHA=0
    case SPI_MODE0:
      p_spi->h_spi->Init.CLKPolarity = SPI_POLARITY_LOW;
      p_spi->h_spi->Init.CLKPhase    = SPI_PHASE_1EDGE;
      HAL_SPI_Init(p_spi->h_spi);
      break;

    // CPOL=0, CPHA=1
    case SPI_MODE1:
      p_spi->h_spi->Init.CLKPolarity = SPI_POLARITY_LOW;
      p_spi->h_spi->Init.CLKPhase    = SPI_PHASE_2EDGE;
      HAL_SPI_Init(p_spi->h_spi);
      break;

    // CPOL=1, CPHA=0
    case SPI_MODE2:
      p_spi->h_spi->Init.CLKPolarity = SPI_POLARITY_HIGH;
      p_spi->h_spi->Init.CLKPhase    = SPI_PHASE_1EDGE;
      HAL_SPI_Init(p_spi->h_spi);
      break;

    // CPOL=1, CPHA=1
    case SPI_MODE3:
      p_spi->h_spi->Init.CLKPolarity = SPI_POLARITY_HIGH;
      p_spi->h_spi->Init.CLKPhase    = SPI_PHASE_2EDGE;
      HAL_SPI_Init(p_spi->h_spi);
      break;
  }
}

uint32_t SPI_Get_Speed(uint8_t dev)
{
  spi_t  *p_spi = &spi_dev_tbl[dev].dev;
  return p_spi->h_spi->Init.BaudRatePrescaler;
}

static uint32_t spiDivisorToBRbits(uint8_t dev, uint16_t divisor)
{
#if !defined(STM32H7)
    // SPI2 and SPI3 are on APB1/AHB1 which PCLK is half that of APB2/AHB2.
	spi_t  *p_spi = &spi_dev_tbl[dev].dev;
    if (p_spi->h_spi->Instance == SPI2 || p_spi->h_spi->Instance == SPI3) {
        divisor /= 2; // Safe for divisor == 0 or 1
    }
#else
    UNUSED(dev);
#endif

    divisor = constrain(divisor, 2, 256);

#if defined(STM32H7)
    const uint32_t baudRatePrescaler[8] = {
        LL_SPI_BAUDRATEPRESCALER_DIV2,
        LL_SPI_BAUDRATEPRESCALER_DIV4,
        LL_SPI_BAUDRATEPRESCALER_DIV8,
        LL_SPI_BAUDRATEPRESCALER_DIV16,
        LL_SPI_BAUDRATEPRESCALER_DIV32,
        LL_SPI_BAUDRATEPRESCALER_DIV64,
        LL_SPI_BAUDRATEPRESCALER_DIV128,
        LL_SPI_BAUDRATEPRESCALER_DIV256,
    };
    int prescalerIndex = ffs(divisor) - 2; // prescaler begins at "/2"

    return baudRatePrescaler[prescalerIndex];
#else
    return (ffs(divisor) - 2) << SPI_CR1_BR_Pos;
#endif
}

bool SPI_Set_Speed(uint8_t dev, uint32_t prescaler)
{
  spi_t  *p_spi = &spi_dev_tbl[dev].dev;
  p_spi->h_spi->Init.BaudRatePrescaler = prescaler;
  HAL_SPI_Init(p_spi->h_spi);
  return true;
}

void spiSetClkDivisor(uint8_t dev, uint32_t divisor)
{
  uint32_t Prescaler;
  spi_t  *p_spi = &spi_dev_tbl[dev].dev;
  Prescaler = spiDivisorToBRbits(dev, divisor);
  p_spi->h_spi->Init.BaudRatePrescaler = Prescaler;
  HAL_SPI_Init(p_spi->h_spi);
}

 HAL_StatusTypeDef SPI_ByteRead(uint8_t dev, uint8_t MemAddress, uint8_t *data, uint8_t length)
{
  spi_t  *p_spi = &spi_dev_tbl[dev].dev;
  HAL_StatusTypeDef status;
    gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_LOW);
    HAL_SPI_Transmit(p_spi->h_spi, &MemAddress, 1, 10);
    status = HAL_SPI_Receive(p_spi->h_spi, data, length, 10);
    //status = HAL_SPI_TransmitReceive(p_spi->h_spi, &MemAddress, data, length, 10);
    gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_HIGH);
  return status;
}

HAL_StatusTypeDef SPI_ByteRead_DMA(uint8_t dev, uint8_t *MemAddress, uint8_t *data, uint8_t length)
{
    spi_t  *p_spi = &spi_dev_tbl[dev].dev;
    HAL_StatusTypeDef status;
    gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_LOW);
    HAL_SPI_Transmit_DMA(p_spi->h_spi, MemAddress, 1);
    status = HAL_SPI_Receive_DMA(p_spi->h_spi, data, length);
    gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_HIGH);
  return status;
}

HAL_StatusTypeDef SPI_ByteReadWrite_DMA(uint8_t dev, uint8_t *MemAddress, uint8_t *data, uint8_t length)
{
    spi_t  *p_spi = &spi_dev_tbl[dev].dev;
    HAL_StatusTypeDef status;
    gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_LOW);
    status = HAL_SPI_TransmitReceive_DMA(p_spi->h_spi, MemAddress, data, length);
    gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_HIGH);
  return status;
}

HAL_StatusTypeDef SPI_ByteWrite_DMA(uint8_t dev, uint8_t *data, uint8_t length)
{
	spi_t  *p_spi = &spi_dev_tbl[dev].dev;
	HAL_StatusTypeDef status;
	gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_LOW);
	status = HAL_SPI_Transmit_DMA(p_spi->h_spi, data, length);
	gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_HIGH);
  return status;
}

HAL_StatusTypeDef SPI_ByteWrite(uint8_t dev, uint8_t MemAddress, uint8_t *data, uint32_t length)
{
  spi_t  *p_spi = &spi_dev_tbl[dev].dev;
  HAL_StatusTypeDef status;
    gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_LOW);
    HAL_SPI_Transmit(p_spi->h_spi, &MemAddress, 1, 10);
    status = HAL_SPI_Transmit(p_spi->h_spi, data, length, 10);
    gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_HIGH);
  return status;
}

// Wait for bus to become free, then read a byte from a register
uint8_t spiReadReg(uint8_t dev, uint8_t reg)
{
	spi_t  *p_spi = &spi_dev_tbl[dev].dev;
	uint8_t data;
	gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_LOW);
	HAL_SPI_Transmit(p_spi->h_spi, &reg, sizeof(reg), 10);
	HAL_SPI_Receive(p_spi->h_spi, &data, sizeof(data), 10);
	gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_HIGH);

    // Wait for completion
	spiWait(dev);
	return data;
}
// Wait for bus to become free, then read a byte of data where the register is ORed with 0x80
uint8_t spiReadRegMsk(uint8_t dev, uint8_t reg)
{
    return spiReadReg(dev, reg | 0x80);
}

void spiWriteReg(uint8_t dev, uint8_t reg, uint8_t data)
{
	spi_t  *p_spi = &spi_dev_tbl[dev].dev;
    gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_LOW);
    HAL_SPI_Transmit(p_spi->h_spi, &reg, sizeof(reg), 10);
    HAL_SPI_Transmit(p_spi->h_spi, &data, sizeof(data), 10);
    gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_HIGH);
}

// Wait for bus to become free, then read/write block of data
void spiReadWriteBuf(uint8_t dev, uint8_t *txData, uint8_t *rxData, int len)
{
	spi_t  *p_spi = &spi_dev_tbl[dev].dev;
	gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_LOW);
	HAL_SPI_Transmit(p_spi->h_spi, txData, 1, 10);
	HAL_SPI_Receive(p_spi->h_spi, rxData, len, 10);
	gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_HIGH);
	spiWait(dev);
}

// Read a block of data from a register
void spiReadRegBuf(uint8_t dev, uint8_t reg, uint8_t *data, uint8_t length)
{
	spi_t  *p_spi = &spi_dev_tbl[dev].dev;
	gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_LOW);
	HAL_SPI_Transmit(p_spi->h_spi, &reg, sizeof(reg), 10);
	HAL_SPI_Receive(p_spi->h_spi, data, length, 10);
	gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_HIGH);

    spiWait(dev);
}

// Read a block of data from a register, returning false if the bus is busy
bool spiReadRegBufRB(uint8_t dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    // Ensure any prior DMA has completed before continuing
    if (spiIsBusy(dev)) {
        return false;
    }

    spiReadRegBuf(dev, reg, data, length);

    return true;
}

// Read a block of data where the register is ORed with 0x80, returning false if the bus is busy
bool spiReadRegMskBufRB(uint8_t dev, uint8_t reg, uint8_t *data, uint8_t length)
{
    return spiReadRegBufRB(dev, reg | 0x80, data, length);
}

void spiWrite(uint8_t dev, uint8_t data)
{
	spi_t  *p_spi = &spi_dev_tbl[dev].dev;
	gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_LOW);
	HAL_SPI_Transmit(p_spi->h_spi, &data, sizeof(data), 10);
	gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_HIGH);

	spiWait(dev);
}

// Wait for bus to become free, then write a block of data to a register
void spiWriteRegBuf(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)
{
	spi_t  *p_spi = &spi_dev_tbl[dev].dev;
	gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_LOW);
	HAL_SPI_Transmit(p_spi->h_spi, &reg, sizeof(reg), 10);
	HAL_SPI_Receive(p_spi->h_spi, data, length, 10);
	gpioPinWrite(spi_dev_tbl[dev].csTag, _DEF_HIGH);
    spiWait(dev);
}

// Wait for DMA completion
void spiWait(uint8_t dev)
{
	spi_t  *p_spi = &spi_dev_tbl[dev].dev;
	// Wait for completion
	while(HAL_SPI_GetState(p_spi->h_spi) != HAL_SPI_STATE_READY);
}

// Return true if DMA engine is busy
bool spiIsBusy(uint8_t dev)
{
	spi_t  *p_spi = &spi_dev_tbl[dev].dev;
    return (HAL_SPI_GetState(p_spi->h_spi) != HAL_SPI_STATE_READY);
}


void spiSetBitWidth(uint8_t dev, uint8_t bit_width)
{
  spi_t  *p_spi = &spi_dev_tbl[dev].dev;

  if (p_spi->is_open == false) return;

  p_spi->h_spi->Init.DataSize = SPI_DATASIZE_8BIT;

  if (bit_width == 16)
  {
    p_spi->h_spi->Init.DataSize = SPI_DATASIZE_16BIT;
  }
  HAL_SPI_Init(p_spi->h_spi);
}

uint16_t spiCalculateDivider(uint32_t freq)
{
#if defined(STM32F4) || defined(STM32G4) || defined(STM32F7)
    uint32_t spiClk = SystemCoreClock / 2;
#elif defined(STM32H7)
    uint32_t spiClk = 100000000;
#else
#error "Base SPI clock not defined for this architecture"
#endif

    uint16_t divisor = 2;

    spiClk >>= 1;

    for (; (spiClk > freq) && (divisor < 256); divisor <<= 1, spiClk >>= 1);

    return divisor;
}

bool spiTransfer(uint8_t dev, uint8_t *tx_buf, uint8_t *rx_buf, uint32_t length, uint32_t timeout)
{
  bool ret = true;
  HAL_StatusTypeDef status;
  spi_t  *p_spi = &spi_dev_tbl[dev].dev;

  if (p_spi->is_open == false) return false;

  if (rx_buf == NULL)
  {
    status =  HAL_SPI_Transmit(p_spi->h_spi, tx_buf, length, timeout);
  }
  else if (tx_buf == NULL)
  {
    status =  HAL_SPI_Receive(p_spi->h_spi, rx_buf, length, timeout);
  }
  else
  {
    status =  HAL_SPI_TransmitReceive(p_spi->h_spi, tx_buf, rx_buf, length, timeout);
  }

  if (status != HAL_OK)
  {
    return false;
  }

  return ret;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	for(uint8_t i = 0; i<SPI_MAX_CH; i++)
	{
		  if (hspi->Instance == spi_dev_tbl[i].dev.h_spi->Instance)
		  {
			  spi_dev_tbl[i].dev.is_error = true;
		  }
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	spi_t  *p_spi;
	for(uint8_t i = 0; i<SPI_MAX_CH; i++)
	{
		  if (hspi->Instance == spi_dev_tbl[i].dev.h_spi->Instance)
		  {
			  p_spi = &spi_dev_tbl[i].dev;
			  p_spi->is_tx_done = true;
			    if (p_spi->func_tx != NULL)
			    {
			      (*p_spi->func_tx)();
			    }
		  }
	}

}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    spi_t  *p_spi;
	for(uint8_t i = 0; i<SPI_MAX_CH; i++)
	{
		  if (hspi->Instance == spi_dev_tbl[i].dev.h_spi->Instance)
		  {
			  	p_spi = &spi_dev_tbl[i].dev;
			  	p_spi->is_rx_done = true;
				if (p_spi->func_rx != NULL)
				{
				  (*p_spi->func_rx)();
				}
		  }
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	spi_t  *p_spi;

	for(uint8_t i = 0; i<SPI_MAX_CH; i++)
	{
		  if (hspi->Instance == spi_dev_tbl[i].dev.h_spi->Instance)
		  {
			  if(i == BMI270) bmi270Intcallback();
			  p_spi = &spi_dev_tbl[i].dev;
			  p_spi->is_tx_done = true;

			  if (p_spi->func_tx != NULL)
			  {
			    (*p_spi->func_tx)();
			  }

			  p_spi->is_rx_done = true;

			  if (p_spi->func_rx != NULL)
			  {
			    (*p_spi->func_rx)();
			  }
		  }
	}
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 DMA Init */
    /* SPI1_RX Init */
    hdma_spi1_rx.Instance = DMA2_Stream2;
    hdma_spi1_rx.Init.Channel = DMA_CHANNEL_3;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi1_rx);

    /* SPI1_TX Init */
    hdma_spi1_tx.Instance = DMA2_Stream3;
    hdma_spi1_tx.Init.Channel = DMA_CHANNEL_3;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi1_tx);

    /* SPI1 interrupt Init */
    HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI2 DMA Init */
    /* SPI2_TX Init */
    hdma_spi2_tx.Instance = DMA1_Stream4;
    hdma_spi2_tx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi2_tx.Init.Mode = DMA_NORMAL;
    hdma_spi2_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi2_tx);

    /* SPI2_RX Init */
    hdma_spi2_rx.Instance = DMA1_Stream3;
    hdma_spi2_rx.Init.Channel = DMA_CHANNEL_0;
    hdma_spi2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi2_rx.Init.Mode = DMA_NORMAL;
    hdma_spi2_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi2_rx);

    /* SPI2 interrupt Init */
    HAL_NVIC_SetPriority(SPI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    /* SPI1 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);

    /* SPI1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();

    /**SPI2 GPIO Configuration
    PB13     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

    /* SPI2 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmatx);
    HAL_DMA_DeInit(spiHandle->hdmarx);

    /* SPI2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(SPI2_IRQn);
  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
}

void cliSPI(cli_args_t *args)
{
  bool ret = true;

  if (ret == false)
  {
    cliPrintf( "spi scan\n");
    cliPrintf( "spi read dev_addr reg_addr length\n");
    cliPrintf( "spi write dev_addr reg_addr data\n");
  }
}
