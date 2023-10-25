/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMEBASE 5 // waiting timebase for speed in/decrease
#define ACCEL_RATE 0.001
#define DECEL_RATE 0.005

#define BASE_PWM 600
#define MAX_PWM 1000

#define MIN_SPEED 1
#define MAX_SPEED 10
#define SPEED_STEP 0.2
#define INITAL_SPEED 0.1

#define BLINK_RATE 1000


#define MOTOR_POLES 4
#define CLOCKSPEED 64000000
#define SPEED_TIM htim14

#define LED_TIM htim17

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t hal_sensors[3];

Direction dir = eRight;
double currentSpeed = 0;
LedColor color = eRed;

SystemState currentState = eStop;

uint8_t commutationState = 0;

uint32_t powerDuty = 700;



typedef enum
{
	eButtonNone,
	eButtonRight,
	eButtonLeft,
	eButtonStop
} ButtonState;


float currentScaling[] = {0, 1, 0.85, 0.7, 0.55};

typedef enum
{
	e5V = 1,
	e9V = 2,
	e12V = 3,
	e15V = 4
} Voltage;

Voltage currentVoltage = e12V;


uint8_t pg_state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
SystemState mainStateMachine(SystemState inState, ButtonState buttonState);
uint32_t calcPwm(SystemState state, Voltage volt, float speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void alignMotor(uint32_t time, uint32_t power)
{
	commutationState = 0;
	HAL_TIM_Base_Start(&htim1);
	htim1.Instance->CCR1 = power;
	htim1.Instance->CCR2 = power;
	htim1.Instance->CCR3 = 1001;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	HAL_Delay(time);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
}


uint32_t calcPwm(SystemState state, Voltage volt, float speed)
{
	uint32_t temp = BASE_PWM;

	temp = temp * currentScaling[volt];
	if(state == eAccel)
	{
		temp *= 1.3;
	}
	else if( state == eDecel)
	{
		temp *= 0.8;
	}

	temp *= 1 + 0.2 * (speed/MAX_SPEED);


	if (temp > MAX_PWM)
		return MAX_PWM;

	return temp;
}

bool setVoltage (Voltage volt)
{
	switch (volt) {
	case e5V:
		GPIOB->MODER &= ~(0x3C0);
		HAL_GPIO_WritePin(CFG1_GPIO_Port, CFG1_Pin, 1);
		break;
	case e9V:
		HAL_GPIO_WritePin(CFG1_GPIO_Port, CFG1_Pin, 0);
		HAL_GPIO_WritePin(CFG2_GPIO_Port, CFG2_Pin, 0);
		HAL_GPIO_WritePin(CFG3_GPIO_Port, CFG3_Pin, 0);
		break;
	case e12V:
		GPIOB->MODER |= (0x140);
		HAL_GPIO_WritePin(CFG1_GPIO_Port, CFG1_Pin, 0);
		HAL_GPIO_WritePin(CFG2_GPIO_Port, CFG2_Pin, 0);
		HAL_GPIO_WritePin(CFG3_GPIO_Port, CFG3_Pin, 1);
		break;
	case e15V:
		GPIOB->MODER |= (0x140);
		HAL_GPIO_WritePin(CFG1_GPIO_Port, CFG1_Pin, 0);
		HAL_GPIO_WritePin(CFG2_GPIO_Port, CFG2_Pin, 1);
		HAL_GPIO_WritePin(CFG3_GPIO_Port, CFG3_Pin, 1);
		break;
	default:
		break;
	}

	HAL_Delay(600);
	return  ~HAL_GPIO_ReadPin(PG_GPIO_Port, PG_Pin);

}


uint32_t reduce_speed(uint32_t in_speed)
{
	if (in_speed > 10000)
	{
		in_speed -= 500;
	}
	else if (in_speed > 5000)
	{
		in_speed -= 300;
	}
	else if (in_speed > 2000)
	{
		in_speed -= 150;
	}
	else
	{
		in_speed -= 100;
	}
	return in_speed;
}

uint32_t speedToLedTimReload(double speed)
{

	double temp = CLOCKSPEED / ( ( (double) LED_TIM.Instance->PSC + 1.0) * speed * 6);
	return ((uint32_t) (temp + 0.5) ) - 1;

}


uint32_t speedToTimReload(double speed)
{

	double temp = CLOCKSPEED / ( ( (double) SPEED_TIM.Instance->PSC + 1.0) * speed * MOTOR_POLES * 6);
	return ((uint32_t) (temp + 0.5) ) - 1;

}

ButtonState updateButtonState()
{
	uint8_t left = HAL_GPIO_ReadPin(LEFT_GPIO_Port, LEFT_Pin) ^1;
	uint8_t right =  HAL_GPIO_ReadPin(RIGHT_GPIO_Port, RIGHT_Pin)^1;
	uint8_t stop =  HAL_GPIO_ReadPin(STOP_GPIO_Port, STOP_Pin)^1;

	if (left + right + stop > 1 || left + right + stop == 0)
	{
		return eButtonNone;
	}
	else if (left)
	{
		return eButtonLeft;
	}
	else if(right)
	{
		return eButtonRight;
	}
	else
	{
		return eButtonStop;
	}

}


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
 int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	 /* MCU Configuration--------------------------------------------------------*/

	 /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	 HAL_Init();

	 /* USER CODE BEGIN Init */

	 /* USER CODE END Init */

	 /* Configure the system clock */
	 SystemClock_Config();

	 /* USER CODE BEGIN SysInit */

	 /* USER CODE END SysInit */

	 /* Initialize all configured peripherals */
	 MX_GPIO_Init();
	 MX_TIM1_Init();
	 MX_TIM16_Init();
	 MX_TIM17_Init();
	 MX_TIM14_Init();
	 /* USER CODE BEGIN 2 */

	 setLed(eLed1 | eLed2 | eLed3);
	 setLedColor(eRed);
	 HAL_Delay(300);
	 setLedColor(eGreen);
	 HAL_Delay(300);
	 setLedColor(eBlue);
	 HAL_Delay(300);
	 setLed(eNone);


	 while (currentVoltage >= e5V)
	 {
		 if (setVoltage(currentVoltage))
		 {
			 break;
		 }
		 currentVoltage--;
	 }



	 setLed(eLed1 | eLed2 | eLed3);
	 if(currentVoltage == e15V)
	 {
		 setLedColor(eGreen);
	 }
	 else
	 {
		 setLedColor(eBlue);
	 }

	 HAL_Delay(500);
	 setLed(eNone);


	 ButtonState buttonState = eButtonNone;


	 setLedColor(eBlack);
	 setLed(eNone);

	 /* USER CODE END 2 */

	 /* Infinite loop */
	 /* USER CODE BEGIN WHILE */
	 while (1)
	 {
		 /* USER CODE END WHILE */

		 /* USER CODE BEGIN 3 */

		 buttonState = updateButtonState();
		 currentState = mainStateMachine(currentState, buttonState);
		 ledControl(currentState);


		 HAL_Delay(5);


	 }
	 /* USER CODE END 3 */
}

 /**
  * @brief System Clock Configuration
  * @retval None
  */
 void SystemClock_Config(void)
 {
	 RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	 RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	 /** Configure the main internal regulator output voltage
	  */
	 HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	 /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	 RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	 RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	 RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	 RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	 RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	 RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	 RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	 RCC_OscInitStruct.PLL.PLLN = 8;
	 RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	 RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	 if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	 {
		 Error_Handler();
	 }

	 /** Initializes the CPU, AHB and APB buses clocks
	  */
	 RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			 |RCC_CLOCKTYPE_PCLK1;
	 RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	 RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	 RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	 if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	 {
		 Error_Handler();
	 }
 }

 /* USER CODE BEGIN 4 */


 void ledControl(SystemState state)
 {
	 switch(state){
	 case eStop:
		 setLed(eNone);
		 break;
	 case eAccel:
		 setLedColor(eYellow);
		 break;
	 case eDecel:
		 setLedColor(eYellow);
		 break;
	 case eRunning:
		 setLedColor(eGreen);
		 break;
	 };
 }



 void setLedColor(LedColor color)
 {
	 static last_color = eBlack;

	 if (last_color != color)
	 {

		 switch (color) {
		 case eBlack:
			 HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 1);
			 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 1);
			 HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 1);
			 break;
		 case eRed:
			 HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 0);
			 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 1);
			 HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 1);
			 break;
		 case eGreen:
			 HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 1);
			 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 0);
			 HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 1);
			 break;
		 case eBlue:
			 HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 1);
			 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 1);
			 HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 0);
			 break;
		 case eYellow:
			 HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 0);
			 HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 0);
			 HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, 1);
			 break;
		 default:
			 break;
		 }
	 }
	 last_color = color;
 }

 void setLed(Led led)
 {

	 if (led & eLed1)
	 {
		 HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	 }

	 if (led & eLed2)
	 {
		 HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);
	 }

	 if (led & eLed3)
	 {
		 HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
	 }
 }




 SystemState mainStateMachine(SystemState inState, ButtonState buttonState)
 {

	 static SystemState lastState = eStop;
	 static double targetSpeed = 0;
	 SystemState nextState = inState;
	 static uint32_t waitUntil = 0;

	 if(buttonState == eButtonStop )
	 {
		 nextState= eStopping;
		 targetSpeed = 0;
	 }

	 if ( waitUntil > HAL_GetTick() && (inState == eAccel || inState == eDecel))
	 {
		 return nextState;
	 }


	 switch (inState) {
	 case eStop:
		 if (inState != lastState )
		 {
			 alignMotor(currentSpeed * 800, 1000);
			 currentSpeed = 0;
			 HAL_TIM_Base_Stop_IT(&htim14);
			 HAL_TIM_Base_Stop_IT(&LED_TIM);
			 setLed(eNone);
			 HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			 HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			 HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
			 HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
			 HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
			 HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

			 waitUntil = HAL_GetTick() + BLINK_RATE;
		 }

		 if(buttonState == eButtonLeft ||  buttonState == eButtonRight)
		 {
			 nextState = eAccel;
			 targetSpeed = MIN_SPEED;

			 dir = eRight;
			 if(buttonState == eButtonLeft)
			 {
				 dir = eLeft;
			 }

		 }

		 if ( HAL_GetTick() > waitUntil)
		 {
			 setLedColor(eBlue);
			 waitUntil += BLINK_RATE;
			 setLed(eLed3);
			 HAL_Delay(50);
			 setLed(eNone);
		 }

		 break;
	 case eRunning:
		 if(buttonState == eButtonLeft )
		 {
			 if (dir == eRight)
			 {
				 nextState = eDecel;
				 targetSpeed -= SPEED_STEP;
			 }
			 else
			 {
				 nextState = eAccel;
				 targetSpeed += SPEED_STEP;
			 }
		 }
		 else if(buttonState == eButtonRight)
		 {
			 if (dir == eRight)
			 {
				 nextState = eAccel;
				 targetSpeed += SPEED_STEP;
			 }
			 else
			 {
				 nextState = eDecel;
				 targetSpeed -= SPEED_STEP;
			 }
		 }

		 powerDuty = calcPwm(inState, currentVoltage, currentSpeed);


		 break;

	 case eAccel:
		 powerDuty = calcPwm(inState, currentVoltage, currentSpeed);

		 // abort if we are at max speed
		 if (targetSpeed > MAX_SPEED)
		 {
			 targetSpeed = MAX_SPEED;
			 break;
		 }

		 // start pwm if we were stopped
		 if (lastState == eStop)
		 {
			 alignMotor(300, 300);
			 __HAL_TIM_SET_AUTORELOAD(&htim14, speedToTimReload(INITAL_SPEED));
			 currentSpeed = INITAL_SPEED;
			 HAL_TIM_Base_Start_IT(&htim14);
			 HAL_TIM_Base_Start(&htim1);
			 HAL_TIM_Base_Start_IT(&htim16);

			 __HAL_TIM_SET_AUTORELOAD(&LED_TIM, speedToLedTimReload(INITAL_SPEED));
			 HAL_TIM_Base_Start_IT(&LED_TIM);
		 }


		 if (currentSpeed < targetSpeed )
		 {
			 currentSpeed += ACCEL_RATE;
			 __HAL_TIM_SET_AUTORELOAD(&htim14, speedToTimReload(currentSpeed));
			 __HAL_TIM_SET_AUTORELOAD(&LED_TIM, speedToLedTimReload(currentSpeed));

		 }
		 else
		 {
			 targetSpeed = currentSpeed;
			 nextState = eRunning;
		 }
		 waitUntil = HAL_GetTick() + TIMEBASE;

		 break;
	 case eDecel:
	 case eStopping:

		 // switch dir if we are at the minimum speed
		 if (currentSpeed < MIN_SPEED)
		 {
			 if (inState == eDecel)
			 {
				 currentSpeed = INITAL_SPEED;
				 targetSpeed = MIN_SPEED;
				 dir ^= 1;
				 nextState = eAccel;
				 break;
			 }
			 else
			{
				 nextState = eStop;
				 break;
			}
		 }


		 if (currentSpeed > targetSpeed )
		 {
			 currentSpeed -= DECEL_RATE;
			 __HAL_TIM_SET_AUTORELOAD(&htim14, speedToTimReload(currentSpeed));
			 __HAL_TIM_SET_AUTORELOAD(&LED_TIM, speedToLedTimReload(currentSpeed));
		 }
		 else
		 {
			 targetSpeed = currentSpeed;
			 nextState = eRunning;
		 }
		 waitUntil = HAL_GetTick() + TIMEBASE;


		 break;
	 default:
		 break;
	 }


	 lastState = inState;
	 return nextState;




 }


 /* USER CODE END 4 */

 /**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
 void Error_Handler(void)
 {
	 /* USER CODE BEGIN Error_Handler_Debug */
	 /* User can add his own implementation to report the HAL error return pg_state */
	 __disable_irq();
	 while (1)
	 {
	 }
	 /* USER CODE END Error_Handler_Debug */
 }

#ifdef  USE_FULL_ASSERT
 /**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
 void assert_failed(uint8_t *file, uint32_t line)
 {
	 /* USER CODE BEGIN 6 */
	 /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	 /* USER CODE END 6 */
 }
#endif /* USE_FULL_ASSERT */
