/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
  const uint32_t fullDuty = 1001;


  extern uint8_t hal_sensors[3];

  extern uint8_t commutationState;
  extern Direction dir;
  extern LedColor color;
  extern SystemState currentState;
  extern uint32_t powerDuty;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
  void Tim1Commutation();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim1;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */

  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */
  Tim1Commutation();
  /* USER CODE END TIM14_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */

  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */
  hal_sensors[0] = HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin);
  hal_sensors[1] = HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin);
  hal_sensors[2] = HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin);
  /* USER CODE END TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM17 global interrupt.
  */
void TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM17_IRQn 0 */
  static uint8_t ledState = 0;
  /* USER CODE END TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM17_IRQn 1 */
  if (dir == eRight)
    {
      ledState++;
      if (ledState > 5)
      {
        ledState = 0;
      }
    }
  else
    {
      ledState--;
      if (ledState > 5)
      {
        ledState = 5;
      }
    }



  setLedColor(color);


  switch (ledState) {
    case 0:
      setLed(eLed1);
      break;
    case 1:
      setLed(eLed1 | eLed2);
      break;
    case 2:
      setLed(eLed2);
      break;
    case 3:
      setLed(eLed2 | eLed3);
      break;
    case 4:
      setLed(eLed3);
      break;
    case 5:
      setLed(eLed3 | eLed1);
      break;
    default:
      break;
  }


  /* USER CODE END TIM17_IRQn 1 */
}

/* USER CODE BEGIN 1 */




void allLedsBreathe(LedColor color)
{

}






void Tim1Commutation()
{
  commutationState++;
    if(commutationState == 7)
      commutationState = 1;


  if (dir == eRight)
    {
    switch (commutationState) {
	  case 0:
		  htim1.Instance->CCR1 = 0;
		  htim1.Instance->CCR2 = fullDuty;
		  htim1.Instance->CCR3 = powerDuty;
		  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		  break;
	  case 1:
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		  htim1.Instance->CCR1 = powerDuty;
		  htim1.Instance->CCR2 = fullDuty;
		  htim1.Instance->CCR3 = 0;
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		  break;
	  case 2:
		  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		  htim1.Instance->CCR1 = powerDuty;
		  htim1.Instance->CCR2 = 0;
		  htim1.Instance->CCR3 = fullDuty;
		  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
		  break;
	  case 3:
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		  htim1.Instance->CCR1 = 0;
		  htim1.Instance->CCR2 = powerDuty;
		  htim1.Instance->CCR3 = fullDuty;
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		  break;
	  case 4:
		  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
		  htim1.Instance->CCR1 = fullDuty;
		  htim1.Instance->CCR2 = powerDuty;
		  htim1.Instance->CCR3 = 0;
		  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		  break;
	  case 5:
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		  htim1.Instance->CCR1 = fullDuty;
		  htim1.Instance->CCR2 = 0;
		  htim1.Instance->CCR3 = powerDuty;
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		  break;
	  case 6:
		  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		  htim1.Instance->CCR1 = 0;
		  htim1.Instance->CCR2 = fullDuty;
		  htim1.Instance->CCR3 = powerDuty;
		  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		  break;
	  default:
		  break;

    }
  } else {
      switch (commutationState) {
	  case 0:
		  htim1.Instance->CCR1 = 0;
		  htim1.Instance->CCR2 = fullDuty;
		  htim1.Instance->CCR3 = powerDuty;
		  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	  case 1:
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
		  htim1.Instance->CCR1 = 0;
		  htim1.Instance->CCR2 = fullDuty;
		  htim1.Instance->CCR3 = powerDuty;
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		  break;
	  case 2:
		  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
		  htim1.Instance->CCR1 = fullDuty;
		  htim1.Instance->CCR2 = 0;
		  htim1.Instance->CCR3 = powerDuty ;
		  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		  break;
	  case 3:
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
		  htim1.Instance->CCR1 = fullDuty;
		  htim1.Instance->CCR2 = powerDuty;
		  htim1.Instance->CCR3 = 0;
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		  break;
	  case 4:
		  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
		  htim1.Instance->CCR1 = 0;
		  htim1.Instance->CCR2 = powerDuty;
		  htim1.Instance->CCR3 = fullDuty;
		  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
		  break;
	  case 5:
		  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
		  htim1.Instance->CCR1 = powerDuty;
		  htim1.Instance->CCR2 = 0;
		  htim1.Instance->CCR3 = fullDuty ;
		  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		  break;
	  case 6:
		  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
		  htim1.Instance->CCR1 = powerDuty;
		  htim1.Instance->CCR2 = fullDuty;
		  htim1.Instance->CCR3 = 0;
		  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		  break;
	  default:
		  break;

      }
  }
}
/* USER CODE END 1 */
