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

  SPI_HandleTypeDef *h_spi;
  DMA_HandleTypeDef *h_dma_tx;
  DMA_HandleTypeDef *h_dma_rx;
} spi_t;

spi_t spi_tbl[SPI_MAX_CH];


SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

static void cliSPI(cli_args_t *args);

bool spiInit(void)
{
  bool ret = true;

  for (int i=0; i<SPI_MAX_CH; i++)
  {
    spi_tbl[i].is_open = false;
    spi_tbl[i].is_tx_done = true;
    spi_tbl[i].is_rx_done = true;
    spi_tbl[i].is_error = false;
    spi_tbl[i].func_tx = NULL;
    spi_tbl[i].func_rx = NULL;
    spi_tbl[i].h_dma_rx = NULL;
    spi_tbl[i].h_dma_tx = NULL;
  }
  spiBegin(_DEF_SPI1);
  spiSetDataMode(_DEF_SPI1, SPI_MODE3);

  cliAdd("spi", cliSPI);
  return ret;
}

bool spiBegin(uint8_t ch)
{
  bool ret = false;
  spi_t *p_spi = &spi_tbl[ch];

  switch(ch)
  {
    case _DEF_SPI1:
      p_spi->h_spi = &hspi1;
      p_spi->h_dma_tx = &hdma_spi1_tx;
      p_spi->h_dma_rx = &hdma_spi1_rx;

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
        p_spi->is_open = true;
        ret = true;
      }
      break;

    case _DEF_SPI2:
      p_spi->h_spi = &hspi2;
      p_spi->h_dma_tx = &hdma_spi2_tx;
      p_spi->h_dma_rx = &hdma_spi2_rx;

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
        p_spi->is_open = true;
        ret = true;
      }
      break;
  }

  return ret;
}

bool spiIsBegin(uint8_t ch)
{
  return spi_tbl[ch].is_open;
}

void spiSetDataMode(uint8_t ch, uint8_t dataMode)
{
  spi_t  *p_spi = &spi_tbl[ch];


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

uint32_t SPI_Get_Speed(uint8_t ch)
{
  spi_t  *p_spi = &spi_tbl[ch];
  return p_spi->h_spi->Init.BaudRatePrescaler;
}

bool SPI_Set_Speed(uint8_t ch, uint32_t prescaler)
{
  spi_t  *p_spi = &spi_tbl[ch];
  p_spi->h_spi->Init.BaudRatePrescaler = prescaler;
  HAL_SPI_Init(p_spi->h_spi);
  return true;
}

 HAL_StatusTypeDef SPI_ByteRead(uint8_t ch, uint8_t MemAddress, uint8_t *data, uint8_t length)
{
  spi_t  *p_spi = &spi_tbl[ch];
  HAL_StatusTypeDef status;
    gpioPinWrite(_PIN_DEF_CS, _DEF_LOW);
    HAL_SPI_Transmit(p_spi->h_spi, &MemAddress, 1, 10);
    status = HAL_SPI_Receive(p_spi->h_spi, data, length, 10);
    //status = HAL_SPI_TransmitReceive(p_spi->h_spi, &MemAddress, data, length, 10);
    gpioPinWrite(_PIN_DEF_CS, _DEF_HIGH);
  return status;
}

HAL_StatusTypeDef SPI_ByteRead_Poll(uint8_t ch, uint8_t *MemAddress, uint8_t *data, uint8_t length)
{
	spi_t  *p_spi = &spi_tbl[ch];
	HAL_StatusTypeDef status;
	gpioPinWrite(_PIN_DEF_CS, _DEF_LOW);
	HAL_SPI_Transmit(p_spi->h_spi, MemAddress, 1, 10);
	status = HAL_SPI_Receive(p_spi->h_spi, data, length, 10);
	gpioPinWrite(_PIN_DEF_CS, _DEF_HIGH);

    // Wait for completion
	while(HAL_SPI_GetState(p_spi->h_spi) != HAL_SPI_STATE_READY);
	return status;
}

HAL_StatusTypeDef SPI_ByteRead_DMA(uint8_t ch, uint8_t *MemAddress, uint8_t *data, uint8_t length)
{
  spi_t  *p_spi = &spi_tbl[ch];
  HAL_StatusTypeDef status;
    gpioPinWrite(_PIN_DEF_CS, _DEF_LOW);
    HAL_SPI_Transmit_DMA(p_spi->h_spi, MemAddress, 1);
    status = HAL_SPI_Receive_DMA(p_spi->h_spi, data, length);
    gpioPinWrite(_PIN_DEF_CS, _DEF_HIGH);
  return status;
}

HAL_StatusTypeDef SPI_ByteReadWrite_DMA(uint8_t ch, uint8_t *MemAddress, uint8_t *data, uint8_t length)
{
  spi_t  *p_spi = &spi_tbl[ch];
  HAL_StatusTypeDef status;
    gpioPinWrite(_PIN_DEF_CS, _DEF_LOW);
    status = HAL_SPI_TransmitReceive_DMA(p_spi->h_spi, MemAddress, data, length);
    gpioPinWrite(_PIN_DEF_CS, _DEF_HIGH);
  return status;
}

HAL_StatusTypeDef SPI_ByteWrite(uint8_t ch, uint8_t MemAddress, uint8_t *data, uint32_t length)
{
  spi_t  *p_spi = &spi_tbl[ch];
  HAL_StatusTypeDef status;
    gpioPinWrite(_PIN_DEF_CS, _DEF_LOW);
    HAL_SPI_Transmit(p_spi->h_spi, &MemAddress, 1, 10);
    status = HAL_SPI_Transmit(p_spi->h_spi, data, length, 10);
    gpioPinWrite(_PIN_DEF_CS, _DEF_HIGH);
  return status;
}

void SPI_RegisterWrite(uint8_t ch, uint8_t MemAddress, uint8_t data, uint8_t delayMs)
{
    SPI_ByteWrite(ch, MemAddress, &data, 1);
    if (delayMs) {
        delay(delayMs);
    }
}

void spiSetBitWidth(uint8_t ch, uint8_t bit_width)
{
  spi_t  *p_spi = &spi_tbl[ch];

  if (p_spi->is_open == false) return;

  p_spi->h_spi->Init.DataSize = SPI_DATASIZE_8BIT;

  if (bit_width == 16)
  {
    p_spi->h_spi->Init.DataSize = SPI_DATASIZE_16BIT;
  }
  HAL_SPI_Init(p_spi->h_spi);
}

uint8_t spiTransfer8(uint8_t ch, uint8_t data)
{
  uint8_t ret;
  spi_t  *p_spi = &spi_tbl[ch];


  if (p_spi->is_open == false) return 0;

  HAL_SPI_TransmitReceive(p_spi->h_spi, &data, &ret, 1, 10);

  return ret;
}

uint16_t spiTransfer16(uint8_t ch, uint16_t data)
{
  uint8_t tBuf[2];
  uint8_t rBuf[2];
  uint16_t ret;
  spi_t  *p_spi = &spi_tbl[ch];


  if (p_spi->is_open == false) return 0;

  if (p_spi->h_spi->Init.DataSize == SPI_DATASIZE_8BIT)
  {
    tBuf[1] = (uint8_t)data;
    tBuf[0] = (uint8_t)(data>>8);
    HAL_SPI_TransmitReceive(p_spi->h_spi, (uint8_t *)&tBuf, (uint8_t *)&rBuf, 2, 10);

    ret = rBuf[0];
    ret <<= 8;
    ret += rBuf[1];
  }
  else
  {
    HAL_SPI_TransmitReceive(p_spi->h_spi, (uint8_t *)&data, (uint8_t *)&ret, 1, 10);
  }

  return ret;
}

bool spiTransfer(uint8_t ch, uint8_t *tx_buf, uint8_t *rx_buf, uint32_t length, uint32_t timeout)
{
  bool ret = true;
  HAL_StatusTypeDef status;
  spi_t  *p_spi = &spi_tbl[ch];

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

void spiDmaTxStart(uint8_t spi_ch, uint8_t *p_buf, uint32_t length)
{
  spi_t  *p_spi = &spi_tbl[spi_ch];

  if (p_spi->is_open == false) return;

  p_spi->is_tx_done = false;
  HAL_SPI_Transmit_DMA(p_spi->h_spi, p_buf, length);
}

bool spiDmaTxTransfer(uint8_t ch, void *buf, uint32_t length, uint32_t timeout)
{
  bool ret = true;
  uint32_t t_time;


  spiDmaTxStart(ch, (uint8_t *)buf, length);

  t_time = millis();

  if (timeout == 0) return true;

  while(1)
  {
    if(spiDmaTxIsDone(ch))
    {
      break;
    }
    if((millis()-t_time) > timeout)
    {
      ret = false;
      break;
    }
  }

  return ret;
}

bool spiDmaTxIsDone(uint8_t ch)
{
  spi_t  *p_spi = &spi_tbl[ch];

  if (p_spi->is_open == false)     return true;

  return p_spi->is_tx_done;
}

void spiAttachTxInterrupt(uint8_t ch, void (*func)())
{
  spi_t  *p_spi = &spi_tbl[ch];


  if (p_spi->is_open == false)     return;

  p_spi->func_tx = func;
}



void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == spi_tbl[_DEF_SPI1].h_spi->Instance)
  {
    spi_tbl[_DEF_SPI1].is_error = true;
  }

  if (hspi->Instance == spi_tbl[_DEF_SPI2].h_spi->Instance)
  {
    spi_tbl[_DEF_SPI2].is_error = true;
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  spi_t  *p_spi;

  if (hspi->Instance == spi_tbl[_DEF_SPI1].h_spi->Instance)
  {
    p_spi = &spi_tbl[_DEF_SPI1];

    p_spi->is_tx_done = true;

    if (p_spi->func_tx != NULL)
    {
      (*p_spi->func_tx)();
    }
  }

  if (hspi->Instance == spi_tbl[_DEF_SPI2].h_spi->Instance)
  {
    p_spi = &spi_tbl[_DEF_SPI2];

    p_spi->is_tx_done = true;

    if (p_spi->func_tx != NULL)
    {
      (*p_spi->func_tx)();
    }
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  spi_t  *p_spi;

  if (hspi->Instance == spi_tbl[_DEF_SPI1].h_spi->Instance)
  {
    p_spi = &spi_tbl[_DEF_SPI1];

    p_spi->is_rx_done = true;

    if (p_spi->func_rx != NULL)
    {
      (*p_spi->func_rx)();
    }
  }

  if (hspi->Instance == spi_tbl[_DEF_SPI2].h_spi->Instance)
  {
    p_spi = &spi_tbl[_DEF_SPI2];

    p_spi->is_rx_done = true;

    if (p_spi->func_rx != NULL)
    {
      (*p_spi->func_rx)();
    }
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  spi_t  *p_spi;

  if (hspi->Instance == spi_tbl[_DEF_SPI1].h_spi->Instance)
  {
    bmi270Intcallback();
    p_spi = &spi_tbl[_DEF_SPI1];

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

  if (hspi->Instance == spi_tbl[_DEF_SPI2].h_spi->Instance)
  {
    p_spi = &spi_tbl[_DEF_SPI2];

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
