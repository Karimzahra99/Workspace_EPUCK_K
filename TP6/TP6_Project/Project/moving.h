#ifndef MOVING_H_
#define MOVING_H_

#include <stdint.h>
//start the PID regulator thread
void moving_start(void);

void motor_set_position(float position_r, float position_l, int16_t speed_r, int16_t speed_l);

#endif /* MOVING_H_ */
