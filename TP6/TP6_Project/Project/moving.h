#ifndef MOVING_H_
#define MOVING_H_

#include <stdint.h>

typedef enum {
	STRAIGHT_LINE_BACKWARDS = 0,
	PID_FRONTWARDS,
	OBS_AVOIDANCE,
	SEARCH_LINE,
	LOST,
	ROTATE_TILL_COLOR
} STATE_t;

//start the moving thread
void moving_start(void);

//for debug remove after
STATE_t get_rolling_mode(void);

#endif /* MOVING_H_ */
