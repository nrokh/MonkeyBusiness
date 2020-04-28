/*
 * Robomaster GM6020 BLDC motor interfacing
 */

#include "gm6020.h"

/**
  * @brief	Initializes the motor with the given ID.
  * @param	motor Motor structure handle
  * @param	id Motor ID set on DIP switches (1 - 7)
  */
void motor_init(Motor* m, uint8_t id) {
	m->id = id;
	m->can_addr = CAN_ID_RCV_BASE + id;

	m->volt = 0;

	m->pos = 0;
	m->vel = 0;
	m->cur = 0;

	return;
}

/**
  * @brief	Set the motor voltage
  * @param	m Motor structure handle
  * @param	v Voltage command
  */
void set_voltage(Motor* m, int16_t v) {
	// Limit command to allowable range
	if(v > VOLT_MAX)
		v = VOLT_MAX;
	else if(v < VOLT_MIN)
		v = VOLT_MIN;

	m->volt = v;

	return;
}

/**
  * @brief	Send the motor voltage command to the motors over CAN
  * @param	hcan CAN handle
  * @param	m_ptr Motor structure array handle
  */
void send_voltage(CAN_HandleTypeDef* hcan, Motor* m_ptr) {
	// Local variables
	uint8_t data[8];	// Byte array
	uint32_t tx_mail;	// Unused

	// Set up the transmission parameters
	CAN_TxHeaderTypeDef htx;
	htx.IDE = CAN_ID_STD;	// Standard CAN ID
	htx.StdId = CAN_ID_CMD;	// GM6020 command ID
	htx.DLC = 8;			// Number of data bytes
	htx.RTR = CAN_RTR_DATA;	// Data frame, no request

	// Store the motor voltage values
	data[0] = (m_ptr->volt >> 8) & 0xff;
	data[1] = (m_ptr++)->volt & 0xff;
	data[2] = (m_ptr->volt >> 8) & 0xff;
	data[3] = (m_ptr++)->volt & 0xff;
	data[4] = (m_ptr->volt >> 8) & 0xff;
	data[5] = (m_ptr++)->volt & 0xff;
	data[6] = (m_ptr->volt >> 8) & 0xff;
	data[7] = (m_ptr++)->volt & 0xff;

	// Transmit command
	while(HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);	// Wait for a free transmission mailbox
	HAL_CAN_AddTxMessage(hcan, &htx, data, &tx_mail);	// Fill the mailbox

	return;
}

/**
  * @brief	Receive sensor feedback from a motor over CAN
  * @param	hcan CAN handle
  */
void update_meas(CAN_HandleTypeDef* hcan) {
	// External variables
	extern Motor motors[];

	// Local variables
	uint8_t data[8];
	CAN_RxHeaderTypeDef hrx;
	uint8_t ind;

	// Get next message
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hrx, data);

	// Update motor feedback values
	ind = hrx.StdId - CAN_ID_RCV_BASE - 1;
	motors[ind].pos = (uint16_t) (((uint16_t) data[0]) << 8 | data[1]);
	motors[ind].vel = (int16_t) (((uint16_t) data[2]) << 8 | data[3]);
	motors[ind].cur = (int16_t) (((uint16_t) data[4]) << 8 | data[5]);

	return;
}

/**
  * @brief	Turn on the 24V output to the motors
  */
void power_on_motors() {
	HAL_GPIO_WritePin(GPIOH, (POWER1_CTRL_Pin | POWER2_CTRL_Pin | POWER3_CTRL_Pin | POWER4_CTRL_Pin), GPIO_PIN_SET);
	return;
}

/**
  * @brief	Turn off the 24V output to the motors
  */
void power_off_motors() {
	HAL_GPIO_WritePin(GPIOH, (POWER1_CTRL_Pin | POWER2_CTRL_Pin | POWER3_CTRL_Pin | POWER4_CTRL_Pin), GPIO_PIN_RESET);
	return;
}
