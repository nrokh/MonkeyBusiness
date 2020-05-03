/*
 * PID controller with 2-DOF derivative control.
 */

#ifndef INC_PID_H_
#define INC_PID_H_

// Macro definitions
/**
  * @brief	Macro to limit bounded values
  */
#define SATURATE(x, lim) {\
	if(x > lim)\
		x = lim;\
	else if(x < -(lim))\
		x = -(lim);\
}

// Structure definitions
/**
  * @brief	PID controller definition
  */
typedef struct PID_Controller {
	// Controller gains
	float kp;	// Proportional gain
	float ki;	// Integral gain
	float kd;	// Derivative gain

	// Discrete sampling time
	float ts;

	// Controller hard limits (assumed to be symmetric)
	float i_max;	// Maximum allowable error integral
	float u_max;	// Maximum allowable control output

	// Derivative reference weighting (if not in use, set to 1)
	float d_ref_wt;

	// Dynamic values
	float prev_ref;	// Prevoius reference
	float prev_meas;	// Previous measurement
	float i_sum;		// Error integral

	// Controller output
	float u;
} PID_Controller;

// Function prototypes
/**
  * @brief	PID controller initialization
  */
void pid_init(PID_Controller* cntlr, float kp, float ki, float kd, float ts, float i_max, float u_max, float d_ref_wt);

/**
  * @brief	PID controller output calculation
  */
void pid_update(PID_Controller* cntlr, float ref, float meas);

#endif /* INC_PID_H_ */
