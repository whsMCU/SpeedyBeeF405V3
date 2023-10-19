/*
 * rtc.c
 *
 *  Created on: 2020. 12. 9.
 *      Author: baram
 */


#include "rtc.h"

#ifdef _USE_HW_RTC

#define PERSISTENT_OBJECT_MAGIC_VALUE (('B' << 24)|('e' << 16)|('f' << 8)|('1' << 0))

static RTC_HandleTypeDef hrtc;

bool rtcInit(void)
{
  bool ret = true;


  __HAL_RCC_GPIOC_CLK_ENABLE();


  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  uint32_t wasSoftReset;

#ifdef STM32H7
  wasSoftReset = RCC->RSR & RCC_RSR_SFTRSTF;
#else
  wasSoftReset = RCC->CSR & RCC_CSR_SFTRSTF;
#endif

  if (!wasSoftReset || (rtcBackupRegRead(PERSISTENT_OBJECT_MAGIC) != PERSISTENT_OBJECT_MAGIC_VALUE)) {
      for (int i = 1; i < PERSISTENT_OBJECT_COUNT; i++) {
    	  rtcBackupRegWrite(i, 0);
      }
      rtcBackupRegWrite(PERSISTENT_OBJECT_MAGIC, PERSISTENT_OBJECT_MAGIC_VALUE);
  }

  return ret;
}

uint32_t rtcBackupRegRead(persistentObjectId_e id)
{
  return HAL_RTCEx_BKUPRead(&hrtc, id);
}

void rtcBackupRegWrite(persistentObjectId_e id, uint32_t data)
{
  HAL_RTCEx_BKUPWrite(&hrtc, id, data);
}

void RTC_WriteProtectionCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
    
  if (NewState != DISABLE)
  {
    /* Enable the write protection for RTC registers */
    RTC->WPR = 0xFF;   
  }
  else
  {
    /* Disable the write protection for RTC registers */
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;    
  }
}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
    }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

#endif
