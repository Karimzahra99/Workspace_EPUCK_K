#ifndef PID_REGULATOR_H_
#define PID_REGULATOR_H_


//start the PID regulator thread
void pid_regulator_start(void);

void motor_set_position(int16_t position_r, int16_t position_l, int16_t speed_r, int16_t speed_l);

#endif /* PID_REGULATOR_H_ */
