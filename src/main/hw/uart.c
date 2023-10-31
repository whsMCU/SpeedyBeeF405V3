/*
 * uart.c
 *
 *  Created on: 2020. 12. 8.
 *      Author: baram
 */


#include "uart.h"
#include "ring_buffer.h"
#include "usbd_cdc_if.h"
#include "rx/rx.h"
#include "rx/crsf.h"

#include "fc/core.h"


static bool is_open[UART_MAX_CH];
#define MAX_SIZE 128

static qbuffer_t ring_buffer[UART_MAX_CH];
static volatile uint8_t rx_buf[UART_MAX_CH-1][MAX_SIZE];
static volatile uint8_t rx_buf2[MAX_SIZE];

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart6_rx;

const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000}; // see baudRate_e

#define BAUD_RATE_COUNT (sizeof(baudRates) / sizeof(baudRates[0]))

bool uartInit(void)
{
  for (int i=0; i<UART_MAX_CH; i++)
  {
    is_open[i] = false;
  }

  return true;
}

bool uartOpen(uint8_t ch, uint32_t baud)
{
  bool ret = false;

  switch(ch)
  {
    case _DEF_USB:
      is_open[ch] = true;
      ret = true;
      break;

    case _DEF_UART2:
    	huart2.Instance = USART2;
    	huart2.Init.BaudRate = baud;
    	huart2.Init.WordLength = UART_WORDLENGTH_8B;
      huart2.Init.StopBits = UART_STOPBITS_1;
    	huart2.Init.Parity = UART_PARITY_NONE;
    	huart2.Init.Mode = UART_MODE_TX_RX;
    	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    	huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    	qbufferCreate(&ring_buffer[ch], (uint8_t *)&rx_buf2[0], MAX_SIZE);

    	//HAL_UART_DeInit(&huart2);

    	if (HAL_UART_Init(&huart2) != HAL_OK)
    	{
    	  Error_Handler();
    	}
    	else
    	{
    		ret = true;
        is_open[ch] = true;
    	  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)&rx_buf[0], MAX_SIZE);
    	  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
//        if(HAL_UART_Receive_DMA(&huart2, (uint8_t *)&rx_buf2[0], MAX_SIZE) != HAL_OK)
//        {
//          ret = false;
//        }
        //ring_buffer[ch].in  = (ring_buffer[ch].len - hdma_usart2_rx.Instance->NDTR);
        //ring_buffer[ch].out = ring_buffer[ch].in;
    	}
      break;

    case _DEF_UART3:
    	huart3.Instance = USART3;
    	huart3.Init.BaudRate = baud;
    	huart3.Init.WordLength = UART_WORDLENGTH_8B;
      huart3.Init.StopBits = UART_STOPBITS_1;
    	huart3.Init.Parity = UART_PARITY_NONE;
    	huart3.Init.Mode = UART_MODE_TX_RX;
    	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    	huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    	qbufferCreate(&ring_buffer[ch], (uint8_t *)&rx_buf[ch-1][0], MAX_SIZE);

    	if (HAL_UART_Init(&huart3) != HAL_OK)
    	{
    	  Error_Handler();
    	}
    	else
    	{
    		ret = true;
        is_open[ch] = true;
        if(HAL_UART_Receive_DMA(&huart3, (uint8_t *)&rx_buf[ch-1][0], MAX_SIZE) != HAL_OK)
        {
          ret = false;
        }
        ring_buffer[ch].in  = ring_buffer[ch].len - hdma_usart3_rx.Instance->NDTR;
        ring_buffer[ch].out = ring_buffer[ch].in;
    	}
      break;

    case _DEF_UART4:
    	huart4.Instance = UART4;
    	huart4.Init.BaudRate = baud;
    	huart4.Init.WordLength = UART_WORDLENGTH_8B;
      huart4.Init.StopBits = UART_STOPBITS_1;
    	huart4.Init.Parity = UART_PARITY_NONE;
    	huart4.Init.Mode = UART_MODE_TX_RX;
    	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    	huart4.Init.OverSampling = UART_OVERSAMPLING_16;

    	qbufferCreate(&ring_buffer[ch], (uint8_t *)&rx_buf[ch-1][0], MAX_SIZE);

    	if (HAL_UART_Init(&huart4) != HAL_OK)
    	{
    	  Error_Handler();
    	}
    	else
    	{
    		ret = true;
        is_open[ch] = true;
        if(HAL_UART_Receive_DMA(&huart4, (uint8_t *)&rx_buf[ch-1][0], MAX_SIZE) != HAL_OK)
        {
          ret = false;
        }
        ring_buffer[ch].in  = ring_buffer[ch].len - hdma_uart4_rx.Instance->NDTR;
        ring_buffer[ch].out = ring_buffer[ch].in;
    	}
      break;
    case _DEF_UART5:
    	huart5.Instance = UART5;
    	huart5.Init.BaudRate = baud;
    	huart5.Init.WordLength = UART_WORDLENGTH_8B;
      huart5.Init.StopBits = UART_STOPBITS_1;
    	huart5.Init.Parity = UART_PARITY_NONE;
    	huart5.Init.Mode = UART_MODE_TX_RX;
    	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    	huart5.Init.OverSampling = UART_OVERSAMPLING_16;

    	qbufferCreate(&ring_buffer[ch], (uint8_t *)&rx_buf[ch-1][0], MAX_SIZE);

    	if (HAL_UART_Init(&huart5) != HAL_OK)
    	{
    	  Error_Handler();
    	}
    	else
    	{
    		ret = true;
        is_open[ch] = true;
        if(HAL_UART_Receive_DMA(&huart5, (uint8_t *)&rx_buf[ch-1][0], MAX_SIZE) != HAL_OK)
        {
          ret = false;
        }
        ring_buffer[ch].in  = ring_buffer[ch].len - hdma_uart5_rx.Instance->NDTR;
        ring_buffer[ch].out = ring_buffer[ch].in;
    	}
      break;

    case _DEF_UART6:
    	huart6.Instance = USART6;
    	huart6.Init.BaudRate = baud;
    	huart6.Init.WordLength = UART_WORDLENGTH_8B;
      huart6.Init.StopBits = UART_STOPBITS_1;
    	huart6.Init.Parity = UART_PARITY_NONE;
    	huart6.Init.Mode = UART_MODE_TX_RX;
    	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    	huart6.Init.OverSampling = UART_OVERSAMPLING_16;

    	qbufferCreate(&ring_buffer[ch], (uint8_t *)&rx_buf[ch-1][0], MAX_SIZE);

    	if (HAL_UART_Init(&huart6) != HAL_OK)
    	{
    	  Error_Handler();
    	}
    	else
    	{
    		ret = true;
        is_open[ch] = true;
        if(HAL_UART_Receive_DMA(&huart6, (uint8_t *)&rx_buf[ch-1][0], MAX_SIZE) != HAL_OK)
        {
          ret = false;
        }
        ring_buffer[ch].in  = ring_buffer[ch].len - hdma_usart6_rx.Instance->NDTR;
        ring_buffer[ch].out = ring_buffer[ch].in;
    	}
      break;
  }

  return ret;
}

uint32_t uartAvailable(uint8_t ch)
{
  uint32_t ret = 0;

  switch(ch)
  {

    case _DEF_USB:
      ret = cdcAvailable();
      break;

    case _DEF_UART2:
    	//ring_buffer[ch].in = (ring_buffer[ch].len - hdma_usart2_rx.Instance->NDTR);
      ret = qbufferAvailable(&ring_buffer[ch]);
      break;
    case _DEF_UART3:
    	ring_buffer[ch].in = (ring_buffer[ch].len - hdma_usart3_rx.Instance->NDTR);
      ret = qbufferAvailable(&ring_buffer[ch]);
      break;

    case _DEF_UART4:
    	ring_buffer[ch].in = (ring_buffer[ch].len - hdma_uart4_rx.Instance->NDTR);
      ret = qbufferAvailable(&ring_buffer[ch]);
      break;

    case _DEF_UART5:
    	ring_buffer[ch].in = (ring_buffer[ch].len - hdma_uart5_rx.Instance->NDTR);
      ret = qbufferAvailable(&ring_buffer[ch]);
      break;

    case _DEF_UART6:
    	ring_buffer[ch].in = (ring_buffer[ch].len - hdma_usart6_rx.Instance->NDTR);
      ret = qbufferAvailable(&ring_buffer[ch]);
      break;
  }
  return ret;
}

uint8_t uartRead(uint8_t ch)
{
  uint8_t ret = 0;

  switch(ch)
  {
    case _DEF_USB:
      ret = cdcRead();
      break;

    case _DEF_UART2:
    	qbufferRead(&ring_buffer[ch], &ret, 1);
      break;

    case _DEF_UART3:
    	qbufferRead(&ring_buffer[ch], &ret, 1);
      break;

    case _DEF_UART4:
    	qbufferRead(&ring_buffer[ch], &ret, 1);
      break;

    case _DEF_UART5:
    	qbufferRead(&ring_buffer[ch], &ret, 1);
      break;

    case _DEF_UART6:
    	qbufferRead(&ring_buffer[ch], &ret, 1);
      break;
  }

  return ret;
}

uint32_t uartWrite(uint8_t ch, uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;
  HAL_StatusTypeDef status;

  switch(ch)
  {

    case _DEF_USB:
      ret = cdcWrite(p_data, length);
      break; 

    case _DEF_UART2:
      status = HAL_UART_Transmit(&huart2, p_data, length, 100);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART3:
      status = HAL_UART_Transmit(&huart3, p_data, length, 100);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART4:
      status = HAL_UART_Transmit(&huart4, p_data, length, 100);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART5:
      status = HAL_UART_Transmit(&huart5, p_data, length, 100);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART6:
      status = HAL_UART_Transmit(&huart6, p_data, length, 100);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;
  }

  return ret;
}

uint32_t uartWriteIT(uint8_t ch, uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;
  HAL_StatusTypeDef status;

  switch(ch)
  {
    case _DEF_USB:
      status = HAL_UART_Transmit_IT(&huart2, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART2:
      status = HAL_UART_Transmit_IT(&huart2, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART3:
      status = HAL_UART_Transmit_IT(&huart3, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART4:
      status = HAL_UART_Transmit_IT(&huart4, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART5:
      status = HAL_UART_Transmit_IT(&huart5, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART6:
      status = HAL_UART_Transmit_IT(&huart6, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;
  }

  return ret;
}

void serialPrint(uint8_t channel, const char *str)
{
    uint8_t ch;
    while ((ch = *(str++)) != 0) {
      uartWrite(channel, &ch, 1);
    }
}

uint32_t uartPrintf(uint8_t ch, char *fmt, ...)
{
  char buf[MAX_SIZE];
  va_list args;
  int len;
  uint32_t ret;

  va_start(args, fmt);
  len = vsnprintf(buf, MAX_SIZE, fmt, args);

  ret = uartWrite(ch, (uint8_t *)buf, len);

  va_end(args);


  return ret;
}

uint32_t uartPrintf_IT(uint8_t ch, char *fmt, ...)
{
  char buf[MAX_SIZE];
  va_list args;
  int len;
  uint32_t ret;

  va_start(args, fmt);
  len = vsnprintf(buf, MAX_SIZE, fmt, args);

  ret = uartWriteIT(ch, (uint8_t *)buf, len);

  va_end(args);


  return ret;
}

uint32_t uartGetBaud(uint8_t ch)
{
  uint32_t ret = 0;


  switch(ch)
  {
    case _DEF_USB:
      ret = cdcGetBaud();
      break;

    case _DEF_UART2:
      ret = huart2.Init.BaudRate;
      break;

    case _DEF_UART3:
      ret = huart3.Init.BaudRate;
      break;

    case _DEF_UART4:
      ret = huart4.Init.BaudRate;
      break;

    case _DEF_UART5:
      ret = huart5.Init.BaudRate;
      break;

    case _DEF_UART6:
      ret = huart6.Init.BaudRate;
      break;
  }

  return ret;
}

bool uartSetBaud(uint8_t ch, uint32_t baud)
{
	bool ret = false;

	switch(ch)
	{
    case _DEF_USB:
			huart2.Init.BaudRate = baud;
    	if (HAL_UART_Init(&huart2) != HAL_OK)
    	{
    	  Error_Handler();
    	}else
    	{
    		ret = true;
    	}
			break;

		case _DEF_UART2:
			huart2.Init.BaudRate = baud;
    	if (HAL_UART_Init(&huart2) != HAL_OK)
    	{
    	  Error_Handler();
    	}else
    	{
    		ret = true;
    	}
			break;

    case _DEF_UART3:
			huart3.Init.BaudRate = baud;
    	if (HAL_UART_Init(&huart3) != HAL_OK)
    	{
    	  Error_Handler();
    	}else
    	{
    		ret = true;
    	}
			break;

    case _DEF_UART4:
			huart4.Init.BaudRate = baud;
    	if (HAL_UART_Init(&huart4) != HAL_OK)
    	{
    	  Error_Handler();
    	}else
    	{
    		ret = true;
    	}
			break;

    case _DEF_UART5:
			huart5.Init.BaudRate = baud;
    	if (HAL_UART_Init(&huart5) != HAL_OK)
    	{
    	  Error_Handler();
    	}else
    	{
    		ret = true;
    	}
			break;

    case _DEF_UART6:
			huart6.Init.BaudRate = baud;
    	if (HAL_UART_Init(&huart6) != HAL_OK)
    	{
    	  Error_Handler();
    	}else
    	{
    		ret = true;
    	}
			break;
	}

	return ret;
}

baudRate_e lookupBaudRateIndex(uint32_t baudRate)
{
    uint8_t index;

    for (index = 0; index < BAUD_RATE_COUNT; index++) {
        if (baudRates[index] == baudRate) {
            return index;
        }
    }
    return BAUD_AUTO;
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)&rx_buf2[0], MAX_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	static uint32_t pre_time = 0;
	static uint32_t pre_time1 = 0;

  if(huart->Instance == USART2)
  {
		rxRuntimeState.callbackTime = micros() - pre_time;
		pre_time = micros();
		pre_time1 = micros();
		qbufferWrite(&ring_buffer[_DEF_UART2], (uint8_t *)&rx_buf2[0], (uint32_t)Size);
		rxRuntimeState.uartAvailable = uartAvailable(_DEF_UART2);

		while(uartAvailable(_DEF_UART2) > 0){
		      excute_temp = micros();
				crsfDataReceive(uartRead(_DEF_UART2), (void*) &rxRuntimeState);
		        {excute_time = (micros()-excute_temp);
		        if(excute_time >= excute_max)
		        {
		       	 excute_max = excute_time;
		        }
		        if(excute_count > 10000)
		        {
		       	 excute_count = 0;
		       	 excute_max = 0;
		        }
		        excute_count++;}
				rxRuntimeState.rx_count++;
		}
		rxRuntimeState.callbackExeTime = micros() - pre_time1;
		rxRuntimeState.rx_count = 0;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)&rx_buf2[0], MAX_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
 GPIO_InitTypeDef GPIO_InitStruct = {0};
 if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream5;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_MEDIUM;
    hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_usart2_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;
    hdma_usart2_rx.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_usart2_rx.Init.PeriphBurst = DMA_PBURST_SINGLE;

    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }else if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PA0-WKUP     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* UART4 DMA Init */
    /* UART4_RX Init */
    hdma_uart4_rx.Instance = DMA1_Stream2;
    hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart4_rx);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */
    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* UART5 DMA Init */
    /* UART5_RX Init */
    hdma_uart5_rx.Instance = DMA1_Stream0;
    hdma_uart5_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart5_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart5_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_uart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_uart5_rx);

    /* UART5 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspInit 1 */

  /* USER CODE END UART5_MspInit 1 */
  }else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART6 DMA Init */
    /* USART6_RX Init */
    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart6_rx);

    /* USART6 interrupt Init */
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PA0-WKUP     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

    /* UART4 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    /* UART5 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* UART5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PC6     ------> USART6_TX
    PC7     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);

    /* USART6 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART6 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART6_IRQn);
  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}
