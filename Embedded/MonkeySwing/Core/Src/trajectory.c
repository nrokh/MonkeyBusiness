/*
 * Trajectory information.
 */

#include "trajectory.h"

/**
  * @brief	Initialize trajectory structure
  * @param	traj Trajectory structure pointer
  * @param	ptr Pointer to the front of the data array
  * @param	size Number of data elements in the array
  */
void init_traj(Trajectory* traj, uint8_t* ptr, uint32_t size) {
	traj->front_ptr = (traj_t*) ptr;
	traj->curr_ptr = (traj_t*) ptr;
	traj->size = size;

	update_traj(traj);
	return;
}

/**
  * @brief	Reset trajectory back to the beginning of the file
  * @param	traj Trajectory structure pointer
  */
void reset_traj(Trajectory* traj) {
	traj->curr_ptr = traj->front_ptr;

	update_traj(traj);
	return;
}

/**
  * @brief	Update trajectory object to the next time-step of data
  * @param	traj Trajectory structure pointer
  */
void update_traj(Trajectory* traj) {
	// TODO implement trajectory update
	return;
}

/**
  * @brief	Returns whether or not there is any unread data in the trajectory
  */
uint8_t traj_is_finished(Trajectory* traj) {
	return traj->front_ptr + traj->size >= traj->curr_ptr;
}
