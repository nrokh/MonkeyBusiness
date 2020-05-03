/*
 * Robomaster GM6020 BLDC motor interfacing
 */

#ifndef INC_GM6020_H_
#define INC_GM6020_H_

// Includes
#include "main.h"
#include "can.h"

// Defines
#define CAN_ID_CMD 0x1ff
#define CAN_ID_RCV_BASE 0x204
#define VOLT_MAX 30000
#define VOLT_MIN -30000
#define TICK_TO_RAD 7.6708403213033652507e-4
#define RPM_TO_RADpS 1.047197551196597746e-1
//#define MA_TO_NM 7.41e-4
#define MA_TO_NM 1.5e-4
#define NUM_VEL_STORE 1

// Structure definitions
/**
  * @brief	GM6020 motor information definition
  */
typedef struct Motor {
	// Motor identification
	uint8_t id;			// Motor ID, as set on DIP switches (1 - 7)
	uint16_t can_addr;	// Motor CAN address for reception

	// Output signal
	int16_t volt;	// Voltage control signal

	// Feedback signals
	int16_t pos;	// Angular position feedback
	int8_t num_turns;
	int16_t vel;	// Angular velocity feedback
	int16_t cur;	// Torque current feedback

	int16_t vel_hist[NUM_VEL_STORE];

	// Direction parameters
	int8_t dir;		// direction scaling
	uint16_t off;	// angle position offset
} Motor;

// Function prototypes
/**
  * @brief	Motor initialization
  */
void motor_init(Motor* m, uint8_t id, uint8_t dir);

/**
  * @brief	Set the motor voltage
  */
void set_voltage(Motor* m, int16_t v);

/**
  * @brief	Send the motor voltage command to the motors over CAN
  */
void send_voltage(CAN_HandleTypeDef* hcan, Motor* m_ptr);

/**
  * @brief	Receive sensor feedback from a motor over CAN
  */
void update_meas(CAN_HandleTypeDef*);

/**
  * @brief	Turn on the 24V output to the motors
  */
void power_on_motors();

/**
  * @brief	Turn off the 24V output to the motors
  */
void power_off_motors();

#endif /* INC_GM6020_H_ */
