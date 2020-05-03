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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gm6020.h"
#include "gripper.h"
#include "pid.h"
#include "trajectory.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {Init, Wait, Toggle, Power, Cal, Load, Password, Swing} State;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define TRAJ_FILE_STEPS 9	// Number of time steps in the trajectory
#define TRAJ_FILE_BYTES 152880	// Trajectory file size, in units of bytes (MUST be the exact file size)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define TRAJ_FILE_BYTES (13 * 4 * TRAJ_FILE_STEPS)
#define TRAJ_FILE_SIZE (TRAJ_FILE_BYTES / sizeof(traj_t))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PID_Controller pos_cntlrs[4];	// position PID controller
PID_Controller vel_cntlrs[4];	// velocity PID controller
PID_Controller motor_cntlrs[4];	// motor PID controller (technically voltage, whatever that means)
int16_t ref[] = {0, 0, 0, 0};	// motor reference (for debugging)

Motor motors[4];						// motors
int8_t motor_dirs[4] = {1, -1, -1, 1};	// motor directions
uint8_t motors_on = 0;					// indicates if motors are on (1) or off (2)

Trajectory traj;					// trajectory iterator
uint8_t traj_arr[TRAJ_FILE_BYTES];	// raw trajectory file location
uint8_t file_tx_done = 1;			// indicator for if trajectory file is actively being loaded or not (referenced by DMA callback)
uint8_t swing_done = 1;				// indicator for if swing motion is completed

Gripper Gripper_1_Cal;	// gripper 1
Gripper Gripper_2_Cal;	// gripper 2
Gripper* release_hand;					// pointer to an active gripper, used during swing to reduce if-else pairs
Gripper_Num release_num = Gripper_Null;	// indicator for which gripper will be released during swing

State state = Init;	// main program state
unsigned char cmd;	// UART command

// Status messages sent over UART
unsigned char msg_done[] = "done\r\n\r\n";
unsigned char msg_init[] = "Initialization done\r\n\r\n";
unsigned char msg_wait[] = "Waiting for command\r\n";
unsigned char msg_toggle[] = "Toggling gripper... ";
unsigned char msg_power[] = "Toggling motor power... ";
unsigned char msg_cal[] = "Reading motor positions... ";
unsigned char msg_load[] = "Waiting for trajectory file... ";
unsigned char msg_pwd[] = "Waiting for password... ";
unsigned char msg_swing[] = "Starting swing... ";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
Gripper* get_release_hand(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief	PID controller update
  */
void update_motor_cntl() {
	if(traj_is_finished(&traj) == 1) {
		finish_traj();
	}

	if(traj.cmode == 12) {
		close_gripper(&Gripper_2_Cal);
	} else {
		open_gripper(&Gripper_2_Cal);
	}

//	// Position controller update
//	for(uint8_t i = 0; i < 4; ++i) {
//		pid_update(&pos_cntlrs[i], traj.pos[i], motors[i].pos-motors[i].off);
//	}
//	// Velocity controller update
//	for(uint8_t i = 0; i < 4; ++i) {
//		pid_update(&vel_cntlrs[i], pos_cntlrs[i].u, motors[i].vel);
////		pid_update(&vel_cntlrs[i], pos_cntlrs[i].u+traj.vel, motors[i].vel);
//	}
//	// Motor voltage controller update
//	for(uint8_t i = 0; i < 4; ++i) {
//		pid_update(&motor_cntlrs[i], vel_cntlrs[i].u, motors[i].cur);
//		set_voltage(&motors[i], (int16_t) (motor_cntlrs[i].u+traj.torque[i]));
//	}
	for(uint8_t i = 0; i < 4; ++i) {
		pid_update(&pos_cntlrs[i], traj.pos[i], (float) motors[i].pos*TICK_TO_RAD);
		pid_update(&vel_cntlrs[i], traj.vel[i], (float) motors[i].vel*RPM_TO_RADpS);
		pid_update(&motor_cntlrs[i], pos_cntlrs[i].u+vel_cntlrs[i].u, (float) motors[i].cur*MA_TO_NM+traj.torque[i]);
		set_voltage(&motors[i], (int16_t) motor_cntlrs[i].u);
	}

	send_voltage(&hcan1, motors);
	update_traj(&traj);

//	for(uint8_t i = 0; i < 4; ++i) {
//		pid_update(&motor_cntlrs[i], ref[i], motors[i].vel);
//		set_voltage(&motors[i], (int16_t) motor_cntlrs[i].u);
//	}
//	send_voltage(&hcan1, motors);
	return;
}

/**
  * @brief	Finish trajectory response
  */
void finish_traj() {
//	close_gripper(release_hand);	// close swinging gripper

	// Disable timer interrupts
	HAL_TIM_Base_Stop_IT(&htim9);
//	HAL_TIM_Base_Stop_IT(&htim13);

	reset_traj(&traj);	// reset trajectory iterator

	// Flip the gripper to be released
	if(release_num == Gripper_1)
		release_num = Gripper_2;
	else
		release_num = Gripper_1;

	swing_done = 1;
	return;
}

/**
  * @brief	DMA transfer is complete
  */
void traj_load_done() {
	file_tx_done = 1;
	return;
}

/**
  * @brief	Send some info over USART, for debugging
  */
void send_serial() {
	unsigned char byte_ptr[22];
	uint8_t len = sizeof(byte_ptr) / sizeof(byte_ptr[0]);

	sprintf(&byte_ptr[0], "%4d\t%4d\t%4d\t%4d\r\n", motors[0].pos, motors[1].pos, motors[2].pos, motors[3].pos);

	HAL_UART_Transmit(&huart2, byte_ptr, len, HAL_MAX_DELAY);	// Send data

	HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
	return;
}

/**
  * @brief	Determine the gripper which is released during swing
  */
Gripper* get_release_hand() {
	while(release_num == Gripper_Null) {
		HAL_UART_Transmit(&huart2, "Specify release gripper\r\n", 26, 1000);
		while(HAL_UART_Receive(&huart2, &cmd, 1, 100) != HAL_OK);

		if(cmd == '1')
			release_num = Gripper_1;
		else if(cmd == '2')
			release_num = Gripper_2;
	}

	if(release_num == Gripper_1)
		return &Gripper_1_Cal;
	else
		return &Gripper_2_Cal;
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_TIM9_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */

  // Initialize PID controllers for position (outer-most loop of cascade architecture)
  for(uint8_t i = 0; i < 2; ++i)
	  pid_init(&pos_cntlrs[i], 1.0, 0.5, 0.055, 0.001, 2.0, 5.0, 1.0);
  for(uint8_t i = 2; i < 4; ++i)
	  pid_init(&pos_cntlrs[i], 1.0, 0.5, 0.055, 0.001, 2.0, 5.0, 1.0);

  // Initialize PID controllers for velocity (middle loop of cascade architecture)
  for(uint8_t i = 0; i < 2; ++i)
	  pid_init(&vel_cntlrs[i], 0.08, 50.0, 0.0, 0.001, 2.0, 5.0, 1.0);
  for(uint8_t i = 2; i < 4; ++i)
	  pid_init(&vel_cntlrs[i], 0.08, 50.0, 0.0, 0.001, 2.0, 5.0, 1.0);

  // Initialize PID controllers for motors (inner-most loop of cascade architecture)
  for(uint8_t i = 0; i < 4; ++i)
	  pid_init(&motor_cntlrs[i], 3000.0, 1250000.0, 0.0, 0.001, 30000.0, 30000.0, 1.0);

  // Initialize trajectory structure
  init_traj(&traj, &traj_arr[0], TRAJ_FILE_SIZE);

  // Initialize motors
  for(uint8_t i = 0; i < 4; ++i)
	  motor_init(&motors[i], i+1, motor_dirs[i]);

  // Finish CAN initialization for robot motors.
  can_motors_init(&hcan1);

  // Enable CAN
  HAL_CAN_Start(&hcan1);

  // Initialize grippers
  init_gripper(&Gripper_1_Cal, Gripper_1, GRIPPER_1_OPEN_PWM, GRIPPER_1_CLOSE_PWM, &htim8, 3);
  init_gripper(&Gripper_2_Cal, Gripper_2, GRIPPER_2_OPEN_PWM, GRIPPER_2_CLOSE_PWM, &htim8, 4);

  // Enable PWM
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);	// Start PWM for Gripper 1
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);	// Start PWM for Gripper 2

//  // Enable timer interrupts for serial transmit (debugging)
//  HAL_TIM_Base_Start_IT(&htim12);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch(state) {
	  	  // Initialization state - only ever entered once
	  	  case Init:
	  		  HAL_UART_Transmit(&huart2, msg_init, sizeof(msg_init), 1000);
	  		  state = Wait;
	  		  break;

	  	  // Wait state - wait to receive a command over UART
	  	  case Wait:
	  		  HAL_UART_Transmit(&huart2, msg_wait, sizeof(msg_wait), 1000);
	  		  while(HAL_UART_Receive(&huart2, &cmd, 1, 100) != HAL_OK);

	  		  if(cmd == '1' || cmd == '2')
	  			  state = Toggle;
	  		  else if(cmd == 'c' || cmd == 'C')
	  			  state = Cal;
	  		  else if(cmd == 'p' || cmd == 'P')
	  			  state = Power;
	  		  else if(cmd == 't' || cmd == 'T')
	  			  state = Load;
	  		  else if(cmd == 's' || cmd == 'S')
	  			  state = Swing;
	  		  break;

	  	  // Toggle state - open/close the specified gripper
	  	  case Toggle:
	  		  HAL_UART_Transmit(&huart2, msg_toggle, sizeof(msg_toggle), 1000);
	  		  // Open or close gripper 1
	  		  if(cmd == '1')
	  			  toggle_gripper(&Gripper_1_Cal);
	  		  // Open or close gripper 2
			  else if(cmd == '2')
				  toggle_gripper(&Gripper_2_Cal);
	  		  HAL_UART_Transmit(&huart2, msg_done, sizeof(msg_done), 1000);
	  		  state = Wait;	// return to wait state
	  		  break;

	  	  // Motor power state - turn the GM6020 motor power on/off
	  	  case Power:
	  		  HAL_UART_Transmit(&huart2, msg_power, sizeof(msg_power), 1000);
	  		  if(motors_on) {
	  			  // Turn motors off
	  			  power_off_motors();
	  		  	  motors_on = 0;
	  		  } else {
	  			  // Turn motors on
	  			  power_on_motors();
	  			  motors_on = 1;
	  		  }
	  		  HAL_UART_Transmit(&huart2, msg_done, sizeof(msg_done), 1000);
	  		  state = Wait;	// return to wait state
	  		  break;

	  	  // Calibration state - set the zeros for the motor positions
	  	  case Cal:
	  		  HAL_UART_Transmit(&huart2, msg_cal, sizeof(msg_cal), 1000);
	  		  HAL_Delay(100);	// Give motors time to send sensor feedback
	  		  // Save the zero angle readings
	  		  for(uint8_t i = 0; i < 4; ++i)
	  			  motors[i].off = (float) motors[i].pos * TICK_TO_RAD;
	  		  HAL_UART_Transmit(&huart2, msg_done, sizeof(msg_done), 1000);
	  		  state = Wait;	// return to wait state
	  		  break;

	  	  // Load trajectory state - wait until the trajectory file of the exact specified size is received over UART
	  	  case Load:	// NOTE: Cannot receive a new command until the entire file is received
	  		  HAL_UART_Transmit(&huart2, msg_load, sizeof(msg_load), 1000);
	  		  power_off_motors();
//	  		  file_tx_done = 0;
	  		  for(uint32_t ind = 0; ind < TRAJ_FILE_BYTES; ind += 0xffff) {
		  		  file_tx_done = 0;
	  			  HAL_UART_Receive_DMA(&huart2, &traj_arr[ind], (TRAJ_FILE_BYTES-ind>0xffff ? 0xffff : TRAJ_FILE_BYTES-ind));
	  			  while(!file_tx_done);	// wait for transfer to finish
	  		  }
//	  		  HAL_UART_Receive_DMA(&huart2, &traj_arr[0], TRAJ_FILE_BYTES);	// Set up UART for file transfer
//	  		  while(!file_tx_done);	// wait for transfer to finish
	  		  reset_traj(&traj);	// Load in first set of data
	  		  HAL_UART_Transmit(&huart2, msg_done, sizeof(msg_done), 1000);
	  		  state = Password;	// wait for correct sequence before turning on
	  		  break;

	  	  case Password:
	  		  HAL_UART_Transmit(&huart2, msg_pwd, sizeof(msg_pwd), 1000);
	  		  uint8_t cnt = 0;
	  		  while(cnt == 0) {
	  			  while(HAL_UART_Receive(&huart2, &cmd, 1, 100) != HAL_OK);
	  			  if(cmd != 'a')
	  				  continue;
	  			  while(HAL_UART_Receive(&huart2, &cmd, 1, 100) != HAL_OK);
				  if(cmd != 'b')
					  continue;
				  while(HAL_UART_Receive(&huart2, &cmd, 1, 100) != HAL_OK);
				  if(cmd == 'c')
					  cnt = 1;
	  		  }
			  if(motors_on)
				  power_on_motors();
			  HAL_UART_Transmit(&huart2, msg_done, sizeof(msg_done), 1000);
			  state = Wait;	// return to wait state
			  break;

	  	  // Swing state - Use PID to follow trajectory file
	  	  case Swing:
//	  		  release_hand = get_release_hand();	// determine gripper to be released
	  		  HAL_UART_Transmit(&huart2, msg_swing, sizeof(msg_swing), 1000);
	  		  swing_done = 0;
//	  		  open_gripper(release_hand);			// release gripper, start swing
	  		  HAL_TIM_Base_Start_IT(&htim9);		// enable timer interrupts for PID update
//	  		  HAL_TIM_Base_Start_IT(&htim13);		// enable timer interrupts for gripper close
	  		  while(!swing_done);
	  		  HAL_UART_Transmit(&huart2, msg_done, sizeof(msg_done), 1000);
	  		  state = Wait;	// return to wait state
	  		  break;
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
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
