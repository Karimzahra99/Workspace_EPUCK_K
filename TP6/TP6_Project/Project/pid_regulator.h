#ifndef PID_REGULATOR_H_
#define PID_REGULATOR_H_


//start the PID regulator thread
void pid_regulator_start(void);

float speedcms_to_speedsteps (uint8_t speed_cms);

void motor_set_position(float position_r, float position_l, int16_t speed_r, int16_t speed_l);

void mov_circ_right(float vitesse,float rayon,float angle, int mode);

void mov_circ_left(float vitesse,float rayon,float angle, int mode);

int rotate_until_irmax_left(void);

int rotate_until_irmax_right(void);

#endif /* PID_REGULATOR_H_ */
