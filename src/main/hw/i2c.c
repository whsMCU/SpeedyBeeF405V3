/*
 * i2c.c
 *
 *  Created on: 2020. 12. 20.
 *      Author: WANG
 */


#include "i2c.h"
#include "cli.h"

I2C_HandleTypeDef hi2c2;

static void cliI2C(cli_args_t *args);

bool i2cInit(void)
{
  bool ret = true;

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  cliAdd("i2c", cliI2C);
  return ret;
}


bool I2C_ByteWrite(uint8_t DevAddress, uint8_t MemAddress, uint8_t bitStart, uint8_t length, uint8_t data)
{
	 // 010 value to write
	// 76543210 bit numbers
	// xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	uint8_t tmp;
	HAL_I2C_Mem_Read(&hi2c2, DevAddress, MemAddress, 1, &tmp, 1, 1000);

	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	data <<= (bitStart - length + 1); // shift data into correct position
	data &= mask; // zero all non-important bits in data
	tmp &= ~(mask); // zero all important bits in existing byte
	tmp |= data; // combine data with existing byte
	HAL_I2C_Mem_Write(&hi2c2, DevAddress, MemAddress, 1, &tmp, 1, 1000);
  return true;
}

void I2C_BitWrite(uint8_t DevAddress, uint8_t MemAddress, uint8_t bitNum, uint8_t data)
{
	uint8_t tmp;
	HAL_I2C_Mem_Write(&hi2c2, DevAddress, MemAddress, 1, &tmp, 1, 1000);
	tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
	HAL_I2C_Mem_Write(&hi2c2, DevAddress, MemAddress, 1, &tmp, 1, 1000);
}

//void I2C_ByteRead(uint8_t DevAddress, uint8_t MemAddress, uint8_t bitStart, uint8_t length, uint8_t *data)
//{
//	// 01101001 read byte
//	// 76543210 bit numbers
//	//    xxx   args: bitStart=4, length=3
//	//    010   masked
//	//   -> 010 shifted
//	uint8_t tmp;
//	HAL_I2C_Mem_Read(&hi2c2, DevAddress, MemAddress, 1, &tmp, 1, 1);
//	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
//	tmp &= mask;
//	tmp >>= (bitStart - length + 1);
//	*data = tmp;
//}

bool I2C_ByteRead(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  HAL_I2C_Mem_Read(&hi2c2, DevAddress<<1, MemAddress, MemAddSize, pData, Size, 1);
  return true;
}

bool I2C_ByteWrite_HAL(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
  HAL_I2C_Mem_Write(&hi2c2, DevAddress<<1, MemAddress, MemAddSize, pData, Size, 1);
  return true;
}

void I2C_BitRead(uint8_t DevAddress, uint8_t MemAddress, uint8_t bitNum, uint8_t *data)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(&hi2c2, DevAddress, MemAddress, 1, &tmp, 1, 1000);
	*data = tmp & (1 << bitNum);
}

void I2C_Write(uint16_t DevAddress, uint8_t data, uint16_t Size)
{
  if(HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY){
    HAL_I2C_Master_Transmit(&hi2c2, DevAddress, &data, Size, 1000);
  }
}

void I2C_Read(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
  if(HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY){
    HAL_I2C_Master_Receive(&hi2c2, DevAddress, pData, Size, 1000);
  }
}

bool i2cIsDeviceReady(uint8_t dev_addr)
{

  if (HAL_I2C_IsDeviceReady(&hi2c2, dev_addr << 1, 10, 10) == HAL_OK)
  {
    __enable_irq();
    return true;
  }

  return false;
}

// Returns true if bus is still busy
bool busBusy(void)
{
  bool ret = false;
  volatile HAL_I2C_StateTypeDef *status = &hi2c2.State;

  if(*status == HAL_I2C_STATE_BUSY)
  {
    ret = true;
  }
  return ret;
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();

    /* I2C2 interrupt Init */
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

    /* I2C2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
}

void cliI2C(cli_args_t *args)
{
  bool ret = true;
  bool i2c_ret;

  uint16_t dev_addr;
  uint16_t reg_addr;
  uint16_t length;
  uint8_t data;

  uint32_t i;
  uint8_t i2c_data[128];
  uint32_t pre_time;


  if (args->argc == 1)
  {

    if(args->isStr(0, "scan") == true)
    {
      for (i=0x00; i<= 0x7F; i++)
      {
        if (i2cIsDeviceReady(i) == true)
        {
          cliPrintf("I2C CH%d Addr 0x%X : OK\n\r", 2, i);
        }
      }
    }
  }
  else if (args->argc == 4)
  {
    dev_addr = (uint16_t) args->getData(1);
    reg_addr = (uint16_t) args->getData(2);
    length   = (uint16_t) args->getData(3);

    if(args->isStr(0, "read") == true)
    {
      for (i=0; i<length; i++)
      {
        i2c_ret = I2C_ByteRead(dev_addr<<1, reg_addr+i, I2C_MEMADD_SIZE_8BIT, i2c_data, 1);
        if (i2c_ret == true)
        {
          cliPrintf("%d I2C - 0x%02X : 0x%02X\n\r", reg_addr+i, i2c_data[0]);
        }
        else
        {
          cliPrintf("%d I2C - Fail \n\r", 2);
          break;
        }
      }
    }
    else if(args->isStr(0, "write") == true)
    {
      pre_time = millis();
      data = (uint8_t) length;
      i2c_ret = I2C_ByteWrite_HAL(dev_addr<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1);
      if (i2c_ret == true)
      {
        cliPrintf("%d I2C - 0x%02X : 0x%02X, %d ms\n\r", 2, reg_addr, data, millis()-pre_time);
      }
      else
      {
        cliPrintf("%d I2C - Fail \n\r", 2);
      }
    }
    else
    {
      ret = false;
    }
  }
  else
  {
    ret = false;
  }

  if (ret == false)
  {
    cliPrintf( "i2c scan\n\r");
    cliPrintf( "i2c read dev_addr reg_addr length\n\r");
    cliPrintf( "i2c write dev_addr reg_addr data\n\r");
  }
}
