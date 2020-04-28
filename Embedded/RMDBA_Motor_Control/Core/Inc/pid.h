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
	double kp;	// Proportional gain
	double ki;	// Integral gain
	double kd;	// Derivative gain

	// Discrete sampling time
	double ts;

	// Controller hard limits (assumed to be symmetric)
	double i_max;	// Maximum allowable error integral
	double u_max;	// Maximum allowable control output

	// Derivative reference weighting (if not in use, set to 1)
	double d_ref_wt;

	// Dynamic values
	double prev_ref;	// Prevoius reference
	double prev_meas;	// Previous measurement
	double i_sum;		// Error integral

	// Controller output
	double u;
} PID_Controller;

// Function prototypes
/**
  * @brief	PID controller initialization
  */
void pid_init(PID_Controller* cntlr, double kp, double ki, double kd, double ts, double i_max, double u_max, double d_ref_wt);

/**
  * @brief	PID controller output calculation
  */
void pid_update(PID_Controller* cntlr, double ref, double meas);

#endif /* INC_PID_H_ */
