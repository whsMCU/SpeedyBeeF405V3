/*
 * timer.c
 *
 *  Created on: 2020. 12. 26.
 *      Author: WANG
 */


#include "timer.h"

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

static void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

bool timerInit(pwmOutputPort_t *motors, uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion)
{
	bool ret = true;

	//time4 init

  TIM_HandleTypeDef* handle = motors->channel.tim;
  if (handle == NULL) return false;

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = ((SystemCoreClock / 2) / hz) - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = (period - 1) & 0xffff; // AKA TIMx_ARR


//	htim4.Init.Prescaler = 72-1;
//	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
//	htim4.Init.Period = 2000-1; //500hz
#ifdef MOTOR_ESC
	htim4.Init.Period = 2041-1;//490hz
#endif
#ifdef MOTOR_ESC_OneShot125
	htim4.Init.Prescaler = 3; //4khz
	htim4.Init.Period = 4500-1;
#endif
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.RepetitionCounter = 0x0000;
	//htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}

	//sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.Pulse = value;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, motors->channel.channel) != HAL_OK)
	{
		Error_Handler();
	}

	/* USER CODE BEGIN TIM4_Init 2 */
	HAL_TIM_PWM_Start(&htim4, motors->channel.channel);

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

	////////////////////////////////////////////////////////////////
	TIM_ClockConfigTypeDef sClockSourceConfig1 = {0};
	TIM_MasterConfigTypeDef sMasterConfig1 = {0};

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 83;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	{
	Error_Handler();
	}
	sClockSourceConfig1.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig1) != HAL_OK)
	{
	Error_Handler();
	}
	sMasterConfig1.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig1.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig1) != HAL_OK)
	{
	Error_Handler();
	}
	HAL_TIM_Base_Init(&htim5);
	HAL_TIM_Base_Start(&htim5);


	return ret;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

	if(tim_baseHandle->Instance==TIM4)
	{
		/* USER CODE BEGIN TIM4_MspInit 0 */

		/* USER CODE END TIM4_MspInit 0 */
		/* TIM4 clock enable */
		__HAL_RCC_TIM4_CLK_ENABLE();

		/* TIM4 interrupt Init */
		HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM4_IRQn);
		/* USER CODE BEGIN TIM4_MspInit 1 */

		/* USER CODE END TIM4_MspInit 1 */
	}
	else if(tim_baseHandle->Instance==TIM5)
	{
		/* USER CODE BEGIN TIM5_MspInit 0 */

		/* USER CODE END TIM5_MspInit 0 */
		/* TIM5 clock enable */
		__HAL_RCC_TIM5_CLK_ENABLE();

		/* TIM5 interrupt Init */
		HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM5_IRQn);
		/* USER CODE BEGIN TIM5_MspInit 1 */

		/* USER CODE END TIM5_MspInit 1 */
	}
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspPostInit 0 */

  /* USER CODE END TIM4_MspPostInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2
    PB8     ------> TIM4_CH3
    PB9     ------> TIM4_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM4_MspPostInit 1 */

  /* USER CODE END TIM4_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

	if(tim_baseHandle->Instance==TIM4)
	{
		/* USER CODE BEGIN TIM4_MspDeInit 0 */

		/* USER CODE END TIM4_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM4_CLK_DISABLE();

		/* TIM4 interrupt Deinit */
		HAL_NVIC_DisableIRQ(TIM4_IRQn);
		/* USER CODE BEGIN TIM4_MspDeInit 1 */

		/* USER CODE END TIM4_MspDeInit 1 */
	}
	else if(tim_baseHandle->Instance==TIM5)
	{
		/* USER CODE BEGIN TIM5_MspDeInit 0 */

		/* USER CODE END TIM5_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM5_CLK_DISABLE();

		/* TIM5 interrupt Deinit */
		HAL_NVIC_DisableIRQ(TIM5_IRQn);
		/* USER CODE BEGIN TIM5_MspDeInit 1 */

		/* USER CODE END TIM5_MspDeInit 1 */
	}
}
