/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "gm6020.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PID_Controller torq_cntlrs[4];
PID_Controller vel_cntlrs[4];
PID_Controller pos_cntlrs[4];
//float ref[] = {0.0f, 0.0f, 0.0f, 2.0f};
//float ref[] = {0, 0, 0, 1.5707f};
float ref[] = {0, 0, 0, 6.283f};
Motor motors[4];
uint16_t pot_hist[100];
int16_t avgtrack;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief	PID controller update
  */
void motor_cntl_update() {
	int32_t avg = 0;
	for(uint8_t i = 99; i > 0; --i)
		pot_hist[i] = pot_hist[i-1];
	pot_hist[0] = get_pot_value(&hadc1);

	for(uint8_t i = 0; i < 100; ++i)
		avg += pot_hist[i];
	avg /= 100;

//	ref[3] = (float) (avg-2048) * 8 * TICK_TO_RAD;
	for(uint8_t i = 0; i < 4; ++i) {
//		pid_update(&torq_cntlrs[i], ref[i], (float) motors[i].cur*MA_TO_NM);

		pid_update(&vel_cntlrs[i], ref[i], (float) motors[i].vel*RPM_TO_RADpS);
//		pid_update(&pos_cntlrs[i], ref[i], (float) motors[i].pos*TICK_TO_RAD);

		pid_update(&torq_cntlrs[i], vel_cntlrs[i].u, (float) motors[i].cur*MA_TO_NM);
		set_voltage(&motors[i], (int16_t) torq_cntlrs[i].u);
	}
	send_voltage(&hcan1, motors);

	unsigned char byte_ptr[16];
	sprintf(&byte_ptr[0], "%6d,%6d\r\n", (int16_t)(ref[3]/RPM_TO_RADpS), motors[3].vel);
//	sprintf(&byte_ptr[0], "%6d,%6d\r\n", (int16_t)(ref[3]/MA_TO_NM), motors[3].cur);	// current, size 16
	HAL_UART_Transmit(&huart2, byte_ptr, 16, HAL_MAX_DELAY);

	return;
}

/**
  * @brief	Send some info over USART
  */
void send_serial() {
	unsigned char byte_ptr[28];
	uint8_t len = sizeof(byte_ptr) / sizeof(byte_ptr[0]);

//	ref[3] *= -1;

	sprintf(&byte_ptr[0], "%6d\t%6d\t%4d\t%6d\r\n", motors[3].volt, motors[3].cur, motors[3].vel, motors[3].pos);

	HAL_UART_Transmit(&huart2, byte_ptr, len, HAL_MAX_DELAY);	// Send data

	HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
	return;
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
  MX_CAN1_Init();
  MX_TIM9_Init();
  MX_ADC1_Init();
  MX_TIM12_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Initialize PID controllers
  for(uint8_t i = 0; i < 4; ++i) {
//	  pid_init(&pos_cntlrs[i], 100.0f, 10.0f, 8.0f, 0.001f, 2.0f, 5.0f, 1.0f);
	  pid_init(&pos_cntlrs[i], 1.0f, 0.5f, 0.055f, 0.001f, 2.0f, 5.0f, 1.0f);
	  pid_init(&vel_cntlrs[i], 0.08f, 50.0f, 0.0f, 0.001f, 2.0f, 5.0f, 1.0f);
//	  pid_init(&torq_cntlrs[i], 50.0f, 10000.0f, 0.0f, 0.001f, 30000.0f, 30000.0f, 1.0f);
	  pid_init(&torq_cntlrs[i], 3000.0f, 1250000.0f, 0.0f, 0.001f, 30000.0f, 30000.0f, 1.0f);
//  pid_init(&cntlrs[i], 40, 3000, 0, 0.001, 30000, 30000, 1);
  }

  // Initialize motors
  for(uint8_t i = 0; i < 4; ++i)
	  motor_init(&motors[i], i+1, -1);
  motors[3].off = 4249;

  // Finish CAN initialization for robot motors.
  can_motors_init(&hcan1);

  // Turn motors on
  power_on_motors();
  HAL_Delay(500);	// Give motors some time to turn on, otherwise will jerk on startup

  // Enable CAN
  HAL_CAN_Start(&hcan1);
  HAL_Delay(100);

  // Enable timer interrupts
  HAL_TIM_Base_Start_IT(&htim9);
  HAL_TIM_Base_Start_IT(&htim12);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
