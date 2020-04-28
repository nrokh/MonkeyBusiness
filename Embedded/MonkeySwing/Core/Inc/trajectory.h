/*
 * Trajectory information.
 */

#ifndef INC_TRAJECTORY_H_
#define INC_TRAJECTORY_H_

// Includes
#include "main.h"

// Defines
#define LINE_LENGTH 12

// Typedefs
// Default data type of the trajectory file
typedef float traj_t;

// Structure definitions
/**
  * @brief	Trajectory information definition
  */
typedef struct Trajectory {
	// File reading information
	traj_t* front_ptr;	// first address where the file is stored
	traj_t* curr_ptr;	// current address
	uint32_t size;		// size of file, in storage data type units (i.e. length of the array)

	// Current time step information
	traj_t pos[4];		// active position reference
	traj_t vel[4];		// active velocity reference
	traj_t torque[4];	// active torque signal
	traj_t cmode;		// active contact mode
} Trajectory;

// Function Prototypes
/**
  * @brief	Initialize trajectory structure
  */
void init_traj(Trajectory*, uint8_t*, uint32_t);

/**
  * @brief	Reset trajectory back to the beginning of the file
  */
void reset_traj(Trajectory*);

/**
  * @brief	Update trajectory object to the next time-step of data
  */
uint8_t update_traj(Trajectory*);

/**
  * @brief	Returns whether or not there is any unread data in the trajectory
  */
uint8_t traj_is_finished(Trajectory*);

#endif /* INC_TRAJECTORY_H_ */
