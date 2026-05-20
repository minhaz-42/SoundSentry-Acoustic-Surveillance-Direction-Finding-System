/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "fatfs.h"

uint8_t retUSER;    /* Return value for USER */
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */

/* USER CODE BEGIN Variables */
extern I2C_HandleTypeDef hi2c1;

static uint8_t rtc_bcd_to_dec(uint8_t val)
{
  return (uint8_t)(((val >> 4) * 10U) + (val & 0x0FU));
}
/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  uint8_t data[7];
  uint16_t year;
  uint8_t month, date, hour, min, sec;

  if (HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x00, I2C_MEMADD_SIZE_8BIT, data, 7, 100) != HAL_OK) {
    return ((DWORD)(2026 - 1980U) << 25)
         | ((DWORD)5 << 21)
         | ((DWORD)15 << 16);
  }

  year  = 2000U + rtc_bcd_to_dec(data[6]);
  month = rtc_bcd_to_dec(data[5] & 0x1F);
  date  = rtc_bcd_to_dec(data[4] & 0x3F);
  hour  = rtc_bcd_to_dec(data[2] & 0x3F);
  min   = rtc_bcd_to_dec(data[1] & 0x7F);
  sec   = rtc_bcd_to_dec(data[0] & 0x7F);

  if (year < 1980U) year = 1980U;
  if (month == 0 || month > 12) month = 1;
  if (date == 0 || date > 31) date = 1;
  if (hour > 23) hour = 0;
  if (min > 59) min = 0;
  if (sec > 59) sec = 0;

  return ((DWORD)(year - 1980U) << 25)
       | ((DWORD)month << 21)
       | ((DWORD)date << 16)
       | ((DWORD)hour << 11)
       | ((DWORD)min << 5)
       | ((DWORD)(sec / 2U));
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */
