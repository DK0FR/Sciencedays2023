/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
  typedef enum
  {
    eLeft,
    eRight,
  } Direction;

  typedef enum
  {
    eBlack = 0,
    eRed,
    eGreen,
    eBlue,
    eYellow,
  } LedColor;

  typedef enum
  {
    eNone = 0,
    eLed1 = 0x1,
    eLed2 = 0x2,
    eLed3 = 0x4
  } Led;

  typedef enum
  {
    eStop = 0x1,
    eRunning = 0x2,
    eAccel = 0x4,
    eDecel = 0x08,
	eStopping = 0x10
  } SystemState;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void setLed(Led led);
void setLedColor(LedColor color);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RIGHT_Pin GPIO_PIN_13
#define RIGHT_GPIO_Port GPIOC
#define STOP_Pin GPIO_PIN_0
#define STOP_GPIO_Port GPIOA
#define LEFT_Pin GPIO_PIN_2
#define LEFT_GPIO_Port GPIOA
#define HALL_U_Pin GPIO_PIN_3
#define HALL_U_GPIO_Port GPIOA
#define HALL_V_Pin GPIO_PIN_4
#define HALL_V_GPIO_Port GPIOA
#define HALL_W_Pin GPIO_PIN_5
#define HALL_W_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_0
#define LED3_GPIO_Port GPIOB
#define CFG2_Pin GPIO_PIN_3
#define CFG2_GPIO_Port GPIOB
#define CFG3_Pin GPIO_PIN_4
#define CFG3_GPIO_Port GPIOB
#define CFG1_Pin GPIO_PIN_5
#define CFG1_GPIO_Port GPIOB
#define PG_Pin GPIO_PIN_6
#define PG_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_7
#define LED_R_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_8
#define LED_G_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_9
#define LED_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
