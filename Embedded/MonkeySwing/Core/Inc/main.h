/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/**
  * @brief	Macro to limit bounded values
  */
#define SATURATE(x, lb, ub) {\
	if(x > ub)\
		x = ub;\
	else if(x < lb)\
		x = lb;\
}
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/**
 * @brief	Used by TIM9 callback to update motor control outputs
 */
void update_motor_cntl(void);

/**
 * @brief	Used by TIM13 callback to finish the trajectory by closing the gripper
 */
void finish_traj(void);

/**
  * @brief	Used by TIM12 callback to send some info over USART
  */
void send_serial(void);

/**
  * @brief	Used by UART2 callback to indicate that DMA transfer is complete
  */
void traj_load_done(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER1_CTRL_Pin GPIO_PIN_2
#define POWER1_CTRL_GPIO_Port GPIOH
#define POWER2_CTRL_Pin GPIO_PIN_3
#define POWER2_CTRL_GPIO_Port GPIOH
#define POWER3_CTRL_Pin GPIO_PIN_4
#define POWER3_CTRL_GPIO_Port GPIOH
#define POWER4_CTRL_Pin GPIO_PIN_5
#define POWER4_CTRL_GPIO_Port GPIOH
#define BUTTON_Pin GPIO_PIN_2
#define BUTTON_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOF
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
