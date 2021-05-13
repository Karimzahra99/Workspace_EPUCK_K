#ifndef MOVING_H_
#define MOVING_H_

#include <stdint.h>

//start the PID regulator thread
void moving_start(void);

typedef enum {
	STRAIGHT_LINE_BACKWARDS = 0,
	PID_FRONTWARDS,
	OBS_AVOIDANCE,
	SEARCH_LINE,
	LOST
} STATE_t;



//remove from .h when finished debuging
void motor_set_position(float position_r, float position_l, int16_t speed_r, int16_t speed_l);
STATE_t get_rolling_mode(void);

#endif /* MOVING_H_ */
