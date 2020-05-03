/*
 * Robomaster GM6020 BLDC motor interfacing
 */

#include "gm6020.h"

/**
  * @brief	Initializes the CAN filter & interrupts for the robot motors.
  * @param	hcan CAN handle
  */
void can_motors_init(CAN_HandleTypeDef* hcan) {
	// Set up the CAN reception filter
	CAN_FilterTypeDef hfilter;
	hfilter.FilterActivation = CAN_FILTER_ENABLE;		// Enable filtering
	hfilter.FilterBank = 0;								// Configure filter 0
	hfilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;	// Filter to RX FIFO 0
	hfilter.FilterMode = CAN_FILTERMODE_IDMASK;			// Configure in mask mode
	hfilter.FilterScale = CAN_FILTERSCALE_16BIT;		// Configure in 16-bit mode
	hfilter.FilterMaskIdHigh = 0x0000;					// Accept all messages
	hfilter.FilterIdHigh = 0x0000;
	HAL_CAN_ConfigFilter(hcan, &hfilter);

	// Set up CAN interrupts
	// TX - Mailbox empty
	// RX - FIFO 0 message pending
	HAL_CAN_ActivateNotification(hcan, (CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING));

	return;
}

/**
  * @brief	Initializes the motor with the given ID.
  * @param	motor Motor structure handle
  * @param	id Motor ID set on DIP switches (1 - 7)
  */
void motor_init(Motor* m, uint8_t id, uint8_t dir) {
	m->id = id;
	m->can_addr = CAN_ID_RCV_BASE + id;

	m->volt = 0;

	m->pos = 0;
	m->num_turns = 0;
	m->vel = 0;
	m->cur = 0;

	m->dir = dir;
	m->off = 0;

	return;
}

/**
  * @brief	Set the motor voltage
  * @param	m Motor structure handle
  * @param	v Voltage command
  */
void set_voltage(Motor* m, int16_t v) {
	SATURATE(v, VOLT_MIN, VOLT_MAX);	// Limit command to allowable range
	m->volt = v * m->dir;
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
  * @brief	Receive sensor feedback from a motor over CAN. Used by CAN RX0 interrupt.
  * @param	hcan CAN handle
  */
void update_meas(CAN_HandleTypeDef* hcan) {
	// External variables
	extern Motor motors[4];

	// Local variables
	uint8_t data[8];
	CAN_RxHeaderTypeDef hrx;
	uint8_t ind;
	int16_t old_pos;

	// Get next message
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hrx, data);

	// Update motor feedback values
	ind = hrx.StdId - CAN_ID_RCV_BASE - 1;

//	if(motors[ind].dir == 1) {
//		motors[ind].pos = (float) (((uint16_t) data[0]) << 8 | data[1]) * TICK_TO_RAD;
//		motors[ind].vel = (float) (((uint16_t) data[2]) << 8 | data[3]) * RPM_TO_RADpS;
//		motors[ind].cur = (int16_t) (((uint16_t) data[4]) << 8 | data[5]);
//	} else {
//		motors[ind].pos = (float) (8191 - (((uint16_t) data[0]) << 8 | data[1])) * TICK_TO_RAD;
//		motors[ind].vel = (float) -(((uint16_t) data[2]) << 8 | data[3]) * RPM_TO_RADpS;
//		motors[ind].cur = (int16_t) -(((uint16_t) data[4]) << 8 | data[5]);
//	}



	old_pos = motors[ind].pos;

	for(uint8_t i = NUM_VEL_STORE-1; i > 0; --i)
		motors[ind].vel_hist[i] = motors[ind].vel_hist[i-1];

	motors[ind].pos = (int16_t) (((uint16_t) data[0]) << 8 | data[1]) - motors[ind].off;
	motors[ind].vel_hist[0] = (int16_t) (((uint16_t) data[2]) << 8 | data[3]);
	motors[ind].cur = (int16_t) (((uint16_t) data[4]) << 8 | data[5]);

	motors[ind].vel = 0;
	for(uint8_t i = 0; i < NUM_VEL_STORE; ++i)
		motors[ind].vel += motors[ind].vel_hist[i];
	motors[ind].vel /= NUM_VEL_STORE;

	if(motors[ind].dir == -1) {
		motors[ind].pos = 8191 - motors[ind].pos;
		motors[ind].vel *= -1;
		motors[ind].cur *= -1;
	}

	motors[ind].pos += motors[ind].num_turns * 8192;	// Incorporate angle wrapping

	if(motors[ind].pos - old_pos > 8000) {
		--motors[ind].num_turns;
		motors[ind].pos -= 8192;
	} else if(motors[ind].pos - old_pos < -8000) {
		++motors[ind].num_turns;
		motors[ind].pos += 8192;
	}
	return;
}

/**
  * @brief	Turn on the 24V output to the motors
  */
void power_on_motors() {
	HAL_GPIO_WritePin(GPIOH, (POWER1_CTRL_Pin | POWER2_CTRL_Pin | POWER3_CTRL_Pin | POWER4_CTRL_Pin), GPIO_PIN_SET);
	HAL_Delay(500);	// Give motors some time to turn on, otherwise will jerk on startup
	return;
}

/**
  * @brief	Turn off the 24V output to the motors
  */
void power_off_motors() {
	HAL_GPIO_WritePin(GPIOH, (POWER1_CTRL_Pin | POWER2_CTRL_Pin | POWER3_CTRL_Pin | POWER4_CTRL_Pin), GPIO_PIN_RESET);
	return;
}
