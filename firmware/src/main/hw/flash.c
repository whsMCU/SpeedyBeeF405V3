/*
 * flash.c
 *
 *  Created on: 2020. 12. 14.
 *      Author: baram
 */


#include "flash.h"
#include "cli.h"

#ifdef _USE_HW_FLASH


#define FLASH_SECTOR_MAX          12



typedef struct
{
  uint32_t addr;
  uint32_t length;
} flash_tbl_t;


static flash_tbl_t flash_tbl[FLASH_SECTOR_MAX] =
    {
        {0x8000000,  16*1024},
        {0x8004000,  16*1024},
        {0x8008000,  16*1024},
        {0x800C000,  16*1024},
        {0x8010000,  64*1024},
        {0x8020000, 128*1024},
        {0x8040000, 128*1024},
        {0x8060000, 128*1024},
				{0x8080000, 128*1024},
				{0x80A0000, 128*1024},
				{0x80C0000, 128*1024},
				{0x80E0000, 128*1024},
    };



static bool flashInSector(uint16_t sector_num, uint32_t addr, uint32_t length);


#ifdef _USE_HW_CLI
static void cliFlash(cli_args_t *args);
#endif


bool flashInit(void)
{

#ifdef _USE_HW_CLI
  cliAdd("flash", cliFlash);
#endif

  return true;
}

HAL_StatusTypeDef flashErase(int16_t sector_num, unsigned int FLASH_VOLTAGE_RANGE)
{
  bool ret = false;
  HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef init;
  uint32_t page_error;
  uint32_t sector_count = 1;


  if (sector_count > 0)
  {

    init.TypeErase   = FLASH_TYPEERASE_SECTORS;
    init.Banks       = FLASH_BANK_1;
    init.Sector      = sector_num;
    init.NbSectors   = sector_count;
    init.VoltageRange = FLASH_VOLTAGE_RANGE;

    status = HAL_FLASHEx_Erase(&init, &page_error);
    if (status == HAL_OK)
    {
      ret = true;
    }

  }

  return status;
}

bool flashWrite(uint32_t addr, uint8_t *p_data, uint32_t length)
{
  bool ret = true;
  HAL_StatusTypeDef status;


  HAL_FLASH_Unlock();

  for (int i=0; i<length; i+=1)
  {
    uint16_t data;

    data  = p_data[i+0] << 0;

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, addr + i, (uint64_t)data);
    if (status != HAL_OK)
    {
      ret = false;
      break;
    }
  }

  HAL_FLASH_Lock();

  return ret;
}

HAL_StatusTypeDef FLASH_ProgramWord(uint32_t Address, uint32_t Data)
{
  HAL_StatusTypeDef status;

  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, (uint64_t)Data);

  return status;
}

bool flashRead(uint32_t addr, uint8_t *p_data, uint32_t length)
{
  bool ret = true;
  uint8_t *p_byte = (uint8_t *)addr;


  for (int i=0; i<length; i++)
  {
    p_data[i] = p_byte[i];
  }

  return ret;
}

bool flashInSector(uint16_t sector_num, uint32_t addr, uint32_t length)
{
  bool ret = false;

  uint32_t sector_start;
  uint32_t sector_end;
  uint32_t flash_start;
  uint32_t flash_end;


  sector_start = flash_tbl[sector_num].addr;
  sector_end   = flash_tbl[sector_num].addr + flash_tbl[sector_num].length - 1;
  flash_start  = addr;
  flash_end    = addr + length - 1;


  if (sector_start >= flash_start && sector_start <= flash_end)
  {
    ret = true;
  }

  if (sector_end >= flash_start && sector_end <= flash_end)
  {
    ret = true;
  }

  if (flash_start >= sector_start && flash_start <= sector_end)
  {
    ret = true;
  }

  if (flash_end >= sector_start && flash_end <= sector_end)
  {
    ret = true;
  }

  return ret;
}

#ifdef _USE_HW_CLI
void cliFlash(cli_args_t *args)
{
  bool ret = false;



  if (args->argc == 1 && args->isStr(0, "info") == true)
  {
    for (int i=0; i<FLASH_SECTOR_MAX; i++)
    {
      cliPrintf("0x%X : %dKB\n", flash_tbl[i].addr, flash_tbl[i].length/1024);
    }

    ret = true;
  }

  if (args->argc == 3 && args->isStr(0, "read") == true)
  {
    uint32_t addr;
    uint32_t length;

    addr   = (uint32_t)args->getData(1);
    length = (uint32_t)args->getData(2);

    for (int i=0; i<length; i++)
    {
      cliPrintf("0x%X : 0x%X\n", addr+i, *((uint8_t *)(addr+i)));
    }

    ret = true;
  }

  if (args->argc == 3 && args->isStr(0, "erase") == true)
  {
    uint32_t addr;
    uint32_t length;

    addr   = (uint32_t)args->getData(1);
    length = (uint32_t)args->getData(2);

    if (flashErase(addr, length) == true)
    {
      cliPrintf("Erase OK\n");
    }
    else
    {
      cliPrintf("Erase Fail\n");
    }
  }

  if (args->argc == 3 && args->isStr(0, "write") == true)
  {
    uint32_t addr;
    uint32_t data;

    addr   = (uint32_t)args->getData(1);
    data   = (uint32_t)args->getData(2);

    if (flashWrite(addr, (uint8_t *)&data, 4) == true)
    {
      cliPrintf("Write OK\n");
    }
    else
    {
      cliPrintf("Write Fail\n");
    }

    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("flash info\n");
    cliPrintf("flash read  addr length\n");
    cliPrintf("flash erase addr length\n");
    cliPrintf("flash write addr data\n");
  }
}
#endif

#endif
