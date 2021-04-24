#ifndef PID_REGULATOR_H_
#define PID_REGULATOR_H_


//start the PID regulator thread
void pid_regulator_start(void);

float speedcms_to_speedsteps (uint8_t speed_cms);

#endif /* PID_REGULATOR_H_ */
