#ifndef PID_REGULATOR_H_
#define PID_REGULATOR_H_


//start the PID regulator thread
void pid_regulator_start(void);

void motor_set_position(float position_r, float position_l, float speed_r, float speed_l);

#endif /* PID_REGULATOR_H_ */
