/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    user_diskio.c
  * @brief   SD card SPI diskio driver for FatFs
  ******************************************************************************
  */
/* USER CODE END Header */

#include <string.h>
#include "ff_gen_drv.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
static BYTE CardType = 0;

#define SD_CS_GPIO_Port GPIOA
#define SD_CS_Pin       GPIO_PIN_4

#define CMD0    0
#define CMD1    1
#define CMD8    8
#define CMD9    9
#define CMD12   12
#define CMD16   16
#define CMD17   17
#define CMD24   24
#define CMD55   55
#define CMD58   58
#define ACMD41  (0x80 + 41)

#define CT_MMC    0x01
#define CT_SD1    0x02
#define CT_SD2    0x04
#define CT_SDC    (CT_SD1 | CT_SD2)
#define CT_BLOCK  0x08

DSTATUS USER_initialize(BYTE pdrv);
DSTATUS USER_status(BYTE pdrv);
DRESULT USER_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
DRESULT USER_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif
#if _USE_IOCTL == 1
DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void *buff);
#endif

Diskio_drvTypeDef USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if _USE_WRITE == 1
  USER_write,
#endif
#if _USE_IOCTL == 1
  USER_ioctl,
#endif
};

static uint8_t sd_spi_txrx(uint8_t data)
{
  uint8_t rx = 0xFF;
  HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, 100);
  return rx;
}

static void sd_spi_rx_multi(BYTE *buff, UINT len)
{
  while (len--) {
    *buff++ = sd_spi_txrx(0xFF);
  }
}

static void sd_spi_tx_multi(const BYTE *buff, UINT len)
{
  while (len--) {
    sd_spi_txrx(*buff++);
  }
}

static void sd_deselect(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
  sd_spi_txrx(0xFF);
}

static int sd_wait_ready(uint32_t timeout_ms)
{
  uint8_t d;
  uint32_t start = HAL_GetTick();

  do {
    d = sd_spi_txrx(0xFF);
    if (d == 0xFF) {
      return 1;
    }
  } while ((HAL_GetTick() - start) < timeout_ms);

  return 0;
}

static int sd_select(void)
{
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
  sd_spi_txrx(0xFF);
  return sd_wait_ready(500);
}

static BYTE sd_send_cmd(BYTE cmd, DWORD arg)
{
  BYTE crc, res, n;

  if (cmd & 0x80) {
    cmd &= 0x7F;
    res = sd_send_cmd(CMD55, 0);
    if (res > 1) {
      return res;
    }
  }

  sd_deselect();
  if (!sd_select()) {
    return 0xFF;
  }

  sd_spi_txrx(0x40 | cmd);
  sd_spi_txrx((BYTE)(arg >> 24));
  sd_spi_txrx((BYTE)(arg >> 16));
  sd_spi_txrx((BYTE)(arg >> 8));
  sd_spi_txrx((BYTE)arg);

  crc = 0x01;
  if (cmd == CMD0) crc = 0x95;
  if (cmd == CMD8) crc = 0x87;
  sd_spi_txrx(crc);

  if (cmd == CMD12) {
    sd_spi_txrx(0xFF);
  }

  n = 10;
  do {
    res = sd_spi_txrx(0xFF);
  } while ((res & 0x80) && --n);

  return res;
}

static int sd_rx_datablock(BYTE *buff, UINT len)
{
  BYTE token;
  uint32_t start = HAL_GetTick();

  do {
    token = sd_spi_txrx(0xFF);
    if (token != 0xFF) {
      break;
    }
  } while ((HAL_GetTick() - start) < 200);

  if (token != 0xFE) {
    return 0;
  }

  sd_spi_rx_multi(buff, len);
  sd_spi_txrx(0xFF);
  sd_spi_txrx(0xFF);

  return 1;
}

#if _USE_WRITE == 1
static int sd_tx_datablock(const BYTE *buff, BYTE token)
{
  BYTE resp;

  if (!sd_wait_ready(500)) {
    return 0;
  }

  sd_spi_txrx(token);

  if (token == 0xFD) {
    return 1;
  }

  sd_spi_tx_multi(buff, 512);
  sd_spi_txrx(0xFF);
  sd_spi_txrx(0xFF);

  resp = sd_spi_txrx(0xFF);
  if ((resp & 0x1F) != 0x05) {
    return 0;
  }

  return 1;
}
#endif

DSTATUS USER_initialize(BYTE pdrv)
{
  BYTE n, cmd, ty, ocr[4];
  uint32_t start;

  if (pdrv != 0) {
    return STA_NOINIT;
  }

  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
  for (n = 0; n < 10; n++) {
    sd_spi_txrx(0xFF);
  }

  ty = 0;

  if (sd_send_cmd(CMD0, 0) == 1) {
    if (sd_send_cmd(CMD8, 0x1AA) == 1) {
      for (n = 0; n < 4; n++) {
        ocr[n] = sd_spi_txrx(0xFF);
      }

      if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
        start = HAL_GetTick();
        while ((HAL_GetTick() - start) < 1000 && sd_send_cmd(ACMD41, 1UL << 30)) {
        }

        if ((HAL_GetTick() - start) < 1000 && sd_send_cmd(CMD58, 0) == 0) {
          for (n = 0; n < 4; n++) {
            ocr[n] = sd_spi_txrx(0xFF);
          }
          ty = (ocr[0] & 0x40) ? (CT_SD2 | CT_BLOCK) : CT_SD2;
        }
      }
    } else {
      if (sd_send_cmd(ACMD41, 0) <= 1) {
        ty = CT_SD1;
        cmd = ACMD41;
      } else {
        ty = CT_MMC;
        cmd = CMD1;
      }

      start = HAL_GetTick();
      while ((HAL_GetTick() - start) < 1000 && sd_send_cmd(cmd, 0)) {
      }

      if ((HAL_GetTick() - start) >= 1000 || sd_send_cmd(CMD16, 512) != 0) {
        ty = 0;
      }
    }
  }

  CardType = ty;
  sd_deselect();

  if (ty) {
    Stat = 0;
  } else {
    Stat = STA_NOINIT;
  }

  return Stat;
}

DSTATUS USER_status(BYTE pdrv)
{
  if (pdrv != 0) {
    return STA_NOINIT;
  }

  return Stat;
}

DRESULT USER_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
  if (pdrv != 0 || count == 0) {
    return RES_PARERR;
  }

  if (Stat & STA_NOINIT) {
    return RES_NOTRDY;
  }

  if (!(CardType & CT_BLOCK)) {
    sector *= 512U;
  }

  do {
    if (sd_send_cmd(CMD17, sector) != 0 || !sd_rx_datablock(buff, 512)) {
      break;
    }

    buff += 512;
    if (CardType & CT_BLOCK) {
      sector += 1U;
    } else {
      sector += 512U;
    }
  } while (--count);

  sd_deselect();
  return count ? RES_ERROR : RES_OK;
}

#if _USE_WRITE == 1
DRESULT USER_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  if (pdrv != 0 || count == 0) {
    return RES_PARERR;
  }

  if (Stat & STA_NOINIT) {
    return RES_NOTRDY;
  }

  if (!(CardType & CT_BLOCK)) {
    sector *= 512U;
  }

  do {
    if (sd_send_cmd(CMD24, sector) != 0 || !sd_tx_datablock(buff, 0xFE)) {
      break;
    }

    buff += 512;
    if (CardType & CT_BLOCK) {
      sector += 1U;
    } else {
      sector += 512U;
    }
  } while (--count);

  sd_deselect();
  return count ? RES_ERROR : RES_OK;
}
#endif

#if _USE_IOCTL == 1
DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  BYTE csd[16];
  DWORD csize;
  BYTE n;

  if (pdrv != 0) {
    return RES_PARERR;
  }

  if (Stat & STA_NOINIT) {
    return RES_NOTRDY;
  }

  switch (cmd) {
    case CTRL_SYNC:
      if (sd_select()) {
        res = RES_OK;
      }
      sd_deselect();
      break;

    case GET_SECTOR_SIZE:
      *(WORD *)buff = 512;
      res = RES_OK;
      break;

    case GET_BLOCK_SIZE:
      *(DWORD *)buff = 1;
      res = RES_OK;
      break;

    case GET_SECTOR_COUNT:
      if (sd_send_cmd(CMD9, 0) == 0 && sd_rx_datablock(csd, 16)) {
        if ((csd[0] & 0xC0) == 0x40) {
          csize = ((DWORD)(csd[7] & 0x3F) << 16) | ((DWORD)csd[8] << 8) | csd[9];
          *(DWORD *)buff = (csize + 1U) << 10;
        } else {
          n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
          csize = ((DWORD)(csd[6] & 3) << 10) | ((DWORD)csd[7] << 2) | (csd[8] >> 6);
          *(DWORD *)buff = (csize + 1U) << (n - 9);
        }
        res = RES_OK;
      }
      sd_deselect();
      break;

    case MMC_GET_TYPE:
      *(BYTE *)buff = CardType;
      res = RES_OK;
      break;

    default:
      res = RES_PARERR;
      break;
  }

  return res;
}
#endif
