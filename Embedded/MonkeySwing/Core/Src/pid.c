/*
 * PID controller with 2-DOF derivative control.
 */

#include "pid.h"

/**
  * @brief	Initializes the PID controller with the given gains, limits, and weight.
  * @param	cntlr PID controller structure handle
  * @param	kp Proportional gain
  * @param	ki Integral gain
  * @param	kd Derivative gain
  * @param	ts Discrete sampling time
  * @param	i_max Maximum allowable error integral	(symmetric)
  * @param	u_max Maximum allowable control output	(symmetric)
  * @param	d_ref_wt Derivative controller setpoint weight (if not used, set to 1)
  */
void pid_init(PID_Controller* cntlr, float kp, float ki, float kd, float ts, float i_max, float u_max, float d_ref_wt) {
	// Move control parameters into controller structure.
	cntlr->kp = kp;
	cntlr->ki = ki;
	cntlr->kd = kd;
	cntlr->ts = ts;
	cntlr->i_max = i_max;
	cntlr->u_max = u_max;
	cntlr->d_ref_wt = d_ref_wt;

	// Initialize memory variables to zero.
	cntlr->prev_ref = 0;
	cntlr->prev_meas = 0;
	cntlr->i_sum = 0;

	return;
}

/**
  * @brief	Calculate a new control output.
  * @param	cntlr PID controller structure handle
  * @param	ref Desired value
  * @param	meas Measured value
  */
void pid_update(PID_Controller* cntlr, float ref, float meas) {
	// Declare local variables
	float e, up, ui, ud;

	// Calculate error;
	e = ref - meas;

	// Update error integral
	cntlr->i_sum += cntlr->ts * e;
	SATURATE(cntlr->i_sum, -cntlr->i_max, cntlr->i_max);

	// Calculate proportional control effort
	up = cntlr->kp * e;

	// Calculate integral control effort
	ui = cntlr->ki * cntlr->i_sum;

	// Calculate derivative control effort
	ud = cntlr->kd * (cntlr->d_ref_wt*(ref - cntlr->prev_ref) - (meas - cntlr->prev_meas)) / cntlr->ts;

	// Calculate total control effort
	cntlr->u = up + ui + ud;
	SATURATE(cntlr->u, -cntlr->u_max, cntlr->u_max);

	// Update error history
	cntlr->prev_ref = ref;
	cntlr->prev_meas = meas;

	return;
}
