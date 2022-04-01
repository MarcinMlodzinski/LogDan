/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lcd.h
  * @brief   This file contains all the function prototypes for
  *          the lcd.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_H__
#define __LCD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern LCD_HandleTypeDef hlcd;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_LCD_Init(void);

/* USER CODE BEGIN Prototypes */
void LCDBarDemo();
void LCDCharDemo();
void LCDStringDemo();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __LCD_H__ */

