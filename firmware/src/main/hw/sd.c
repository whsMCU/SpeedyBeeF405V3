#include "sd.h"
#include "gpio.h"
#include "spi.h"
#include "cli.h"



#ifdef _USE_HW_SD


/* Definitions for MMC/SDC command */
#define CMD0     (0x40+0)     /* GO_IDLE_STATE */
#define CMD1     (0x40+1)     /* SEND_OP_COND */
#define CMD8     (0x40+8)     /* SEND_IF_COND */
#define CMD9     (0x40+9)     /* SEND_CSD */
#define CMD10    (0x40+10)    /* SEND_CID */
#define CMD12    (0x40+12)    /* STOP_TRANSMISSION */
#define CMD16    (0x40+16)    /* SET_BLOCKLEN */
#define CMD17    (0x40+17)    /* READ_SINGLE_BLOCK */
#define CMD18    (0x40+18)    /* READ_MULTIPLE_BLOCK */
#define CMD23    (0x40+23)    /* SET_BLOCK_COUNT */
#define CMD24    (0x40+24)    /* WRITE_BLOCK */
#define CMD25    (0x40+25)    /* WRITE_MULTIPLE_BLOCK */
#define CMD41    (0x40+41)    /* SEND_OP_COND (ACMD) */
#define CMD55    (0x40+55)    /* APP_CMD */
#define CMD58    (0x40+58)    /* READ_OCR */




static bool is_init = false;
static bool is_detected = false;
static bool is_cli_init = false;
static uint8_t dev = SDCARD;
static volatile bool is_rx_done = false;
static volatile bool is_tx_done = false;
static sd_info_t sd_info;

static uint8_t CardType;                                /* SD 타입 0:MMC, 1:SDC, 2:Block addressing */
static uint8_t PowerFlag = 0;                           /* Power 상태 Flag */


static void sdSpiCS(bool enable);
static bool sdSpiTxByte(uint8_t data);
static uint8_t sdSpiRxByte(void);
static uint8_t sdSpiReadyWait(void);
static void sdSpiPowerOn(void);
static void sdSpiPowerOff(void);
//static uint8_t sdSpiCheckPower(void);
static bool sdSpiRxDataBlock(uint8_t *buff, uint32_t btr);
static bool sdSpiTxDataBlock(const uint8_t *buff, uint8_t token);
static uint8_t sdSpiSendCmd(uint8_t cmd, uint32_t arg);
static bool sdSpiInitialize(void);


#ifdef _USE_HW_CLI
static void cliSd(cli_args_t *args);
#endif


bool sdInit(void)
{
  bool ret = false;

  if (spiIsBegin(SDCARD) != true)
  {
	  spiBegin(SDCARD);
  }

  is_detected = true;
  if (gpioPinRead(_PIN_GPIO_SDCARD_DETECT) == true)
  {
    is_detected = true;
  }

  sd_info.card_type         = 0;
  sd_info.card_version      = 0;
  sd_info.card_class        = 0;
  sd_info.rel_card_Add      = 0;
  sd_info.block_numbers     = 0;
  sd_info.block_size        = 0;
  sd_info.log_block_numbers = 0;
  sd_info.log_block_size    = 512;
  sd_info.card_size         = 0;

  if (is_detected == true)
  {
    ret = sdSpiInitialize();
  }

  is_init = ret;


#ifdef _USE_HW_CLI
  if (is_cli_init != true)
  {
    cliAdd("sd", cliSd);
    is_cli_init = true;
  }
#endif

  return ret;
}

bool sdDeInit(void)
{
  bool ret = false;

  if (is_init == true)
  {
    ret = true;
  }

  return ret;
}

bool sdIsInit(void)
{
  return is_init;
}

bool sdIsDetected(void)
{
  if (gpioPinRead(_PIN_GPIO_SDCARD_DETECT) == true)
  {
    is_detected = true;
  }
  else
  {
    is_detected = false;
  }

  return is_detected;
}

bool sdGetInfo(sd_info_t *p_info)
{
  bool ret = false;


  if (is_init == true)
  {
    *p_info = sd_info;
    ret = true;
  }

  return ret;
}

bool sdIsBusy(void)
{
  bool is_busy = true;

  if (sdSpiRxByte() == 0xFF)
  {
    is_busy = false;
  }

  return is_busy;
}

bool sdIsReady(uint32_t timeout)
{
  uint32_t pre_time;

  pre_time = millis();

  while(millis() - pre_time < timeout)
  {
    if (sdIsBusy() == false)
    {
      return true;
    }
  }

  return false;
}

bool sdReadBlocks(uint32_t block_addr, uint8_t *p_data, uint32_t num_of_blocks, uint32_t timeout_ms)
{
  bool ret = false;

  if (!(CardType & 4))
  {
    block_addr *= 512;      /* 지정 sector를 Byte addressing 단위로 변경 */
  }

  sdSpiCS(false);

  if (num_of_blocks == 1)
  {
    /* 싱글 블록 읽기 */
    if ((sdSpiSendCmd(CMD17, block_addr) == 0) && sdSpiRxDataBlock(p_data, 512))
    {
      num_of_blocks = 0;
    }
  }
  else
  {
    /* 다중 블록 읽기 */
    if (sdSpiSendCmd(CMD18, block_addr) == 0)
    {
      do
      {
        if (!sdSpiRxDataBlock(p_data, 512))
        {
          break;
        }

        p_data += 512;
      } while (--num_of_blocks);

      /* STOP_TRANSMISSION, 모든 블럭을 다 읽은 후, 전송 중지 요청 */
      sdSpiSendCmd(CMD12, 0);
    }
  }

  sdSpiCS(true);
  sdSpiRxByte(); /* Idle 상태(Release DO) */

  if (num_of_blocks == 0)
  {
    ret = true;
  }

  return ret;
}

bool sdWriteBlocks(uint32_t block_addr, uint8_t *p_data, uint32_t num_of_blocks, uint32_t timeout_ms)
{
  bool ret = false;

  if (!(CardType & 4))
  {
    block_addr *= 512; /* 지정 sector를 Byte addressing 단위로 변경 */
  }


  sdSpiCS(false);

  if (num_of_blocks == 1)
  {
    /* 싱글 블록 쓰기 */
    if ((sdSpiSendCmd(CMD24, block_addr) == 0) && sdSpiTxDataBlock(p_data, 0xFE))
    {
      num_of_blocks = 0;
    }
  }
  else
  {
    /* 다중 블록 쓰기 */
    if (CardType & 2)
    {
      sdSpiSendCmd(CMD55, 0);
      sdSpiSendCmd(CMD23, num_of_blocks); /* ACMD23 */
    }

    if (sdSpiSendCmd(CMD25, block_addr) == 0)
    {
      do {
        if(!sdSpiTxDataBlock(p_data, 0xFC))
          break;

        p_data += 512;
      } while (--num_of_blocks);

      if(!sdSpiTxDataBlock(0, 0xFD))
      {
        num_of_blocks = 1;
      }
    }
  }

  sdSpiCS(true);
  sdSpiRxByte();

  if (num_of_blocks == 0)
  {
    ret = true;
  }
  else
  {
    ret = false;
  }

  return ret;
}

bool sdEraseBlocks(uint32_t start_addr, uint32_t end_addr)
{
  bool ret = true;


  return ret;
}



void sdSpiCS(bool enable)
{
  gpioPinWrite(_PIN_SDCARD_CS, enable);
}

bool sdSpiTxByte(uint8_t data)
{
  bool ret;

  ret = spiTransfer(SDCARD, &data, NULL, 1, 100);

  return ret;
}

uint8_t sdSpiRxByte(void)
{
  uint8_t data = 0xFF;


  spiTransfer(SDCARD, NULL, &data, 1, 100);

  return data;
}

uint8_t sdSpiReadyWait(void)
{
  uint8_t res;
  uint32_t pre_time;

  sdSpiRxByte();
  pre_time = millis();
  do
  {
    /* 0xFF 값이 수신될 때 까지 SPI 통신 */
    res = sdSpiRxByte();
  } while ((res != 0xFF) && (millis()-pre_time < 500));

  return res;
}

/* 전원 켜기 */
void sdSpiPowerOn(void)
{
  uint8_t cmd_arg[6];
  uint32_t Count = 0x1FFF;
  uint8_t rx_data;

  /* Deselect 상태에서 SPI 메시지를 전송하여 대기상태로 만든다. */
  sdSpiCS(true);

  for(int i = 0; i < 10; i++)
  {
    sdSpiTxByte(0xFF);
  }

  /* SPI Chips Select */
  sdSpiCS(false);

  /* 초기 GO_IDLE_STATE 상태 전환 */
  cmd_arg[0] = (CMD0 | 0x40);
  cmd_arg[1] = 0;
  cmd_arg[2] = 0;
  cmd_arg[3] = 0;
  cmd_arg[4] = 0;
  cmd_arg[5] = 0x95;

  /* 명령 전송 */
  for (int i = 0; i < 6; i++)
  {
    sdSpiTxByte(cmd_arg[i]);
  }

  /* 응답 대기 */
  while (((rx_data=sdSpiRxByte()) != 0x01) && Count)
  {
    Count--;
  }

  sdSpiCS(true);
  sdSpiTxByte(0XFF);

  PowerFlag = 1;
}

/* 전원 끄기 */
void sdSpiPowerOff(void)
{
  PowerFlag = 0;
}

/* 전원 상태 확인 */
uint8_t sdSpiCheckPower(void)
{
  /*  0=off, 1=on */
  return PowerFlag;
}

/* 데이터 패킷 수신 */
bool sdSpiRxDataBlock(uint8_t *buff, uint32_t btr)
{
  uint8_t token;
  uint32_t pre_time;


  /* 응답 대기 */
  pre_time = millis();
  do
  {
    token = sdSpiRxByte();
  } while((token == 0xFF) && (millis()-pre_time < 100));

  /* 0xFE 이외 Token 수신 시 에러 처리 */
  if(token != 0xFE)
    return false;

  /* Receive the data block into buffer */
#if 1
  do
  {
    *buff = sdSpiRxByte();
    buff++;
    *buff = sdSpiRxByte();
    buff++;
  } while (btr -= 2);
#else
  spiTransfer(spi_ch, NULL, buff, btr, 100);
#endif

  sdSpiRxByte(); /* CRC 무시 */
  sdSpiRxByte();

  return true;
}

/* 데이터 전송 패킷 */
bool sdSpiTxDataBlock(const uint8_t *buff, uint8_t token)
{
  uint8_t resp = 0x00;
  uint8_t i = 0;
  uint8_t wc;


  /* SD카드 준비 대기 */
  if (sdSpiReadyWait() != 0xFF)
    return false;

  /* 토큰 전송 */
  sdSpiTxByte(token);

  /* 데이터 토큰인 경우 */
  if (token != 0xFD)
  {
    /* 512 바이트 데이터 전송 */
    //spiTransfer(spi_ch, (uint8_t *)buff, NULL, 512, 100);
    wc = 0;
    do
    {
      sdSpiTxByte(*buff++);
      sdSpiTxByte(*buff++);
    } while (--wc);

    sdSpiRxByte();       /* CRC 무시 */
    sdSpiRxByte();

    /* 데이트 응답 수신 */
    while (i <= 64)
    {
      resp = sdSpiRxByte();

      /* 에러 응답 처리 */
      if ((resp & 0x1F) == 0x05)
        break;

      i++;
    }

    /* SPI 수신 버퍼 Clear */
    while (sdSpiRxByte() == 0);
  }

  if ((resp & 0x1F) == 0x05)
    return true;
  else
    return false;
}

/* CMD 패킷 전송 */
uint8_t sdSpiSendCmd(uint8_t cmd, uint32_t arg)
{
  uint8_t crc, res;

  /* SD카드 대기 */
  if (sdSpiReadyWait() != 0xFF)
    return 0xFF;


  /* 명령 패킷 전송 */
  sdSpiTxByte(cmd);                   /* Command */
  sdSpiTxByte((uint8_t) (arg >> 24)); /* Argument[31..24] */
  sdSpiTxByte((uint8_t) (arg >> 16)); /* Argument[23..16] */
  sdSpiTxByte((uint8_t) (arg >> 8));  /* Argument[15..8] */
  sdSpiTxByte((uint8_t) arg);         /* Argument[7..0] */

  /* 명령별 CRC 준비 */
  crc = 0;
  if (cmd == CMD0)
    crc = 0x95; /* CRC for CMD0(0) */

  if (cmd == CMD8)
    crc = 0x87; /* CRC for CMD8(0x1AA) */

  /* CRC 전송 */
  sdSpiTxByte(crc);

  /* CMD12 Stop Reading 명령인 경우에는 응답 바이트 하나를 버린다 */
  if (cmd == CMD12)
    sdSpiRxByte();

  /* 10회 내에 정상 데이터를 수신한다. */
  uint8_t n = 10;
  do
  {
    res = sdSpiRxByte();
  } while ((res & 0x80) && --n);

  return res;
}


/* SD카드 초기화 */
bool sdSpiInitialize(void)
{
  bool ret = false;
  uint8_t n, type, ocr[4];
  uint32_t pre_time;
  uint8_t csd[16];
  uint32_t csize;

  //spiSetClockDivider(spi_ch, SPI_CLOCK_DIV_16);
  //spiSetClkDivisor(dev, SPI_BAUDRATEPRESCALER_16);
  SPI_Set_Speed(dev, SPI_BAUDRATEPRESCALER_16);

  /* SD카드 Power On */
  sdSpiPowerOn();

  /* SPI 통신을 위해 Chip Select */
  sdSpiCS(false);

  /* SD카드 타입변수 초기화 */
  type = 0;

  /* Idle 상태 진입 */
  if (sdSpiSendCmd(CMD0, 0) == 1)
  {
    pre_time = millis();

    /* SD 인터페이스 동작 조건 확인 */
    if (sdSpiSendCmd(CMD8, 0x1AA) == 1)
    {
      /* SDC Ver2+ */
      for (n = 0; n < 4; n++)
      {
        ocr[n] = sdSpiRxByte();
      }

      if (ocr[2] == 0x01 && ocr[3] == 0xAA)
      {
        /* 2.7-3.6V 전압범위 동작 */
        do {
          if (sdSpiSendCmd(CMD55, 0) <= 1 && sdSpiSendCmd(CMD41, 1UL << 30) == 0)
            break; /* ACMD41 with HCS bit */
        } while (millis()-pre_time < 1000);

        if ((millis()-pre_time < 1000) && sdSpiSendCmd(CMD58, 0) == 0)
        {
          /* Check CCS bit */
          for (n = 0; n < 4; n++)
          {
            ocr[n] = sdSpiRxByte();
          }

          type = (ocr[0] & 0x40) ? 6 : 2;
        }
      }
    }
    else
    {
      /* SDC Ver1 or MMC */
      type = (sdSpiSendCmd(CMD55, 0) <= 1 && sdSpiSendCmd(CMD41, 0) <= 1) ? 2 : 1; /* SDC : MMC */

      do {
        if (type == 2)
        {
          if (sdSpiSendCmd(CMD55, 0) <= 1 && sdSpiSendCmd(CMD41, 0) == 0)
            break; /* ACMD41 */
        }
        else
        {
          if (sdSpiSendCmd(CMD1, 0) == 0)
            break; /* CMD1 */
        }
      } while (millis()-pre_time < 1000);

      if (!(millis()-pre_time < 1000) || sdSpiSendCmd(CMD16, 512) != 0)
      {
        /* 블럭 길이 선택 */
        type = 0;
      }
    }
  }

  CardType = type;
  sd_info.card_type = type;

  sdSpiCS(true);
  sdSpiRxByte(); /* Idle 상태 전환 (Release DO) */


#if 1
  if (type)
  {
    sdSpiCS(false);

    /* SD카드 내 Sector의 개수 (DWORD) */
    if ((sdSpiSendCmd(CMD9, 0) == 0) && sdSpiRxDataBlock(csd, 16))
    {
      if ((csd[0] >> 6) == 1)
      {
        /* SDC ver 2.00 */
        csize = csd[9] + ((uint32_t) csd[8] << 8) + 1;
        sd_info.log_block_numbers = (uint32_t) csize << 10;
      }
      else
      {
        /* MMC or SDC ver 1.XX */
        n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
        csize = (csd[8] >> 6) + ((uint32_t) csd[7] << 2) + ((uint32_t) (csd[6] & 3) << 10) + 1;
        sd_info.log_block_numbers = (uint32_t) csize << (n - 9);
      }

      sd_info.block_numbers = sd_info.log_block_numbers;
      sd_info.block_size    = sd_info.log_block_size;

      sd_info.card_size = (uint32_t)((uint64_t)sd_info.block_numbers * (uint64_t)sd_info.block_size / (uint64_t)1024 / (uint64_t)1024);
    }


    sdSpiCS(true);
    sdSpiRxByte(); /* Idle 상태 전환 (Release DO) */
  }
#endif


  if (type)
  {
    ret = true;
  }
  else
  {
    /* Initialization failed */
    sdSpiPowerOff();
  }

  //spiSetClkDivisor(dev, SPI_BAUDRATEPRESCALER_2);
  SPI_Set_Speed(dev, SPI_BAUDRATEPRESCALER_2);

  return ret;
}

#ifdef _USE_HW_CLI
void cliSd(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "info") == true)
  {
    sd_info_t sd_info;

    cliPrintf("sd init      : %d\n\r", is_init);
    cliPrintf("sd connected : %d\n\r", is_detected);

    if (is_init == true)
    {
      if (sdGetInfo(&sd_info) == true)
      {
        cliPrintf("   card_type            : %d\n\r", sd_info.card_type);
        cliPrintf("   card_version         : %d\n\r", sd_info.card_version);
        cliPrintf("   card_class           : %d\n\r", sd_info.card_class);
        cliPrintf("   rel_card_Add         : %d\n\r", sd_info.rel_card_Add);
        cliPrintf("   block_numbers        : %d\n\r", sd_info.block_numbers);
        cliPrintf("   block_size           : %d\n\r", sd_info.block_size);
        cliPrintf("   log_block_numbers    : %d\n\r", sd_info.log_block_numbers);
        cliPrintf("   log_block_size       : %d\n\r", sd_info.log_block_size);
        cliPrintf("   card_size            : %d MB, %d.%d GB\n\r", sd_info.card_size, sd_info.card_size/1024, ((sd_info.card_size * 10)/1024) % 10);
      }
    }
    ret = true;
  }

  if (args->argc == 2 && args->isStr(0, "read") == true)
  {
    uint32_t number;
    uint32_t buf[512/4];

    number = args->getData(1);

    if (sdReadBlocks(number, (uint8_t *)buf, 1, 100) == true)
    {
      for (int i=0; i<512/4; i++)
      {
        cliPrintf("%d:%04d : 0x%08X\n", number, i*4, buf[i]);
      }
    }
    else
    {
      cliPrintf("sdRead Fail\n\r");
    }

    ret = true;
  }


  if (ret != true)
  {
    cliPrintf("sd info\n\r");

    if (is_init == true)
    {
      cliPrintf("sd read block_number\n\r");
    }
  }
}
#endif
#endif
