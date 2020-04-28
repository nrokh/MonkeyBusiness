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
  * @return	0 if updated, -1 if trajectory is already finished (not updated)
  */
uint8_t update_traj(Trajectory* traj) {
	// Skip update if trajectory file has been fully iterated
	if(traj_is_finished(traj) == 0) {
		traj->torque[3] = *traj->curr_ptr++;	// load in next set of torque signals
		traj->torque[1] = *traj->curr_ptr++;
		traj->torque[2] = *traj->curr_ptr++;
		traj->torque[1] = *traj->curr_ptr++;
		traj->pos[3] = *traj->curr_ptr++;	// load in next set of position references
		traj->pos[1] = *traj->curr_ptr++;
		traj->pos[2] = *traj->curr_ptr++;
		traj->pos[1] = *traj->curr_ptr++;
		traj->vel[3] = *traj->curr_ptr++;	// load in next set of velocity references
		traj->vel[1] = *traj->curr_ptr++;
		traj->vel[2] = *traj->curr_ptr++;
		traj->vel[1] = *traj->curr_ptr++;


//		for(uint8_t i = 0; i < 4; ++i)
//			traj->torque[i] = *traj->curr_ptr++;	// Load in next set of torque signals
//		for(uint8_t i = 0; i < 4; ++i)
//			traj->pos[i] = *traj->curr_ptr++;		// Load in next set of position references
//		for(uint8_t i = 0; i < 4; ++i)
//			traj->vel[i] = *traj->curr_ptr++;		// Load in next set of velocity references
		traj->cmode = *traj->curr_ptr++;			// Load in next contact mode
		return 0;
	}
	return -1;
}

/**
  * @brief	Returns whether or not there is any unread data in the trajectory
  * @return	0 if there is more of the trajectory file to be read, 1 if it has been fully iterated
  */
uint8_t traj_is_finished(Trajectory* traj) {
	return (uint32_t) (traj->curr_ptr - traj->front_ptr) < traj->size ? 0 : 1;
}
