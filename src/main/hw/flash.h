/*
 * flash.h
 *
 *  Created on: 2020. 12. 14.
 *      Author: baram
 */

#ifndef SRC_COMMON_HW_INCLUDE_FLASH_H_
#define SRC_COMMON_HW_INCLUDE_FLASH_H_

#include "hw.h"


#ifdef _USE_HW_FLASH


bool flashInit(void);
HAL_StatusTypeDef flashErase(int16_t sector_num, unsigned int FLASH_VOLTAGE_RANGE);
bool flashWrite(uint32_t addr, uint8_t *p_data, uint32_t length);
HAL_StatusTypeDef FLASH_ProgramWord(uint32_t Address, uint32_t Data);
bool flashRead(uint32_t addr, uint8_t *p_data, uint32_t length);


#endif


#endif /* SRC_COMMON_HW_INCLUDE_FLASH_H_ */
