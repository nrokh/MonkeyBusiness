/*
 * Gripper interface.
 */

#include "gripper.h"

/**
  * @brief	Initialize the specified gripper
  * @param	gripper Gripper structure handle
  * @param	open Gripper open PWM duty cycle
  * @param	close Gripper close PWM duty cycle
  * @param  htim TIM handle
  */
void init_gripper(Gripper* gripper, Gripper_Num n, uint16_t open, uint16_t close, TIM_HandleTypeDef* htim, uint8_t channel) {
	gripper->num = n;
	gripper->open_pwm = open;
	gripper->close_pwm = close;
	gripper->htim = htim;
	gripper->channel = channel;

	gripper->state = Gripper_Closed;
}

/**
  * @brief  Opens the specified gripper.
  * @param	gripper Gripper structure handle
  */
void open_gripper(Gripper* gripper) {
	UPDATE_DC(gripper->open_pwm, gripper->htim, gripper->channel);
	gripper->state = Gripper_Open;
	return;
}

/**
  * @brief  Closes the specified gripper.
  * @param	gripper Gripper structure handle
  */
void close_gripper(Gripper* gripper) {
	UPDATE_DC(gripper->close_pwm, gripper->htim, gripper->channel);
	gripper->state = Gripper_Closed;
	return;
}

/**
  * @brief	Open or close the specified gripper, depending on what is appropriate.
  * @param	gripper Gripper structure handle
  */
void toggle_gripper(Gripper* gripper) {
	if(gripper->state == Gripper_Open)
		close_gripper(gripper);
	else
		open_gripper(gripper);
	return;
}
