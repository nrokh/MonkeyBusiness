/*
 * Gripper interface.
 */

#ifndef INC_GRIPPER_H_
#define INC_GRIPPER_H_

// Includes
#include "main.h"
#include "tim.h"

// Macros
/**
  * @brief	Macro to update the correct PWM channel
  */
#define UPDATE_DC(pwm, htim, ch) {\
	if(ch == 1)\
		htim->Instance->CCR1 = pwm;\
	else if(ch == 2)\
		htim->Instance->CCR2 = pwm;\
	else if(ch == 3)\
		htim->Instance->CCR3 = pwm;\
	else if(ch == 4)\
		htim->Instance->CCR4 = pwm;\
}

// Defines
// Gripper PWM calibration values
#define GRIPPER_1_OPEN_PWM 3310
#define GRIPPER_1_CLOSE_PWM 4590
#define GRIPPER_2_OPEN_PWM 5700
#define GRIPPER_2_CLOSE_PWM 4430

// Structure definitions
/**
  * @brief	Gripper number enumeration
  */
typedef enum {Gripper_Null, Gripper_1, Gripper_2} Gripper_Num;
typedef enum {Gripper_Open, Gripper_Closed} Gripper_State;

/**
  * @brief	Gripper PWM value structure
  */
typedef struct Gripper {
	Gripper_Num num;			// gripper number (only 2 options)
	uint16_t open_pwm;			// PWM duty cycle for open state
	uint16_t close_pwm;			// PWM duty cycle for closed state
	Gripper_State state;		// current state of gripper (open or closed)
	TIM_HandleTypeDef* htim;	// TIM handle capture/compare register
	uint8_t channel;			// capture/compare channel
} Gripper;

// Function prototypes
/**
  * @brief	Initialize the specified gripper
  */
void init_gripper(Gripper*, Gripper_Num, uint16_t open, uint16_t closed, TIM_HandleTypeDef*, uint8_t channel);

/**
  * @brief	Opens the specified gripper
  */
void open_gripper(Gripper*);

/**
  * @brief	Closes the specified gripper
  */
void close_gripper(Gripper*);

/**
  * @brief	Open or close the specified gripper, depending on what is appropriate
  */
void toggle_gripper(Gripper*);

#endif /* INC_GRIPPER_H_ */
