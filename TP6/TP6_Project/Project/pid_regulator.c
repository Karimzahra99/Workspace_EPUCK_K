#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/proximity.h>

#include <main.h>
#include <motors.h>
#include <pid_regulator.h>
#include <process_image.h>

#define WHEEL_PERIMETER 13 // [cm]
#define NSTEP_ONE_TURN 1000 // number of step for 1 turn of the motor

static uint8_t obstacle_mode = 0;

//simple PI regulator implementation
int16_t pid_regulator(float middle){

	float goal = 350; //milieu theorique d'une ligne parfaitement centre sur le robot
	float error = 0;
	float speed = 0;
	float derivative = 0;

	static float sum_error = 0;
	static float last_error = 0;

	error = middle - goal;

	//get(IR3)
	//get(IR1)

	//disables the PID regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){ //ERROR_THRESHOLD = 0.1cm definit en float
		return 0;
	}

	sum_error += error;//sum_error = sum_error + error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	// Integral anti-windup (Anti-Reset Windup)
	//https://www.youtube.com/watch?v=NVLXCwc8HzM&list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y&index=2
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	derivative = error - last_error;

	last_error = error;

	speed = KP * error + KI * sum_error + KD*derivative;

    return (int16_t)speed;
}


static THD_WORKING_AREA(waPidRegulator, 256);
static THD_FUNCTION(PidRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();

        int ir_front_left = get_prox(Sensor_IR3);
        int ir_front_right = get_prox(Sensor_IR4);

        if (((ir_front_left < IR_THRESHOLD) && (ir_front_right < IR_THRESHOLD)) && !obstacle_mode){

        	//computes the speed to give to the motors
        	//distance_cm is modified by the image processing thread
        	switch (get_color())
        	{
        	case 0:
        		speed = 0;
        		break;
        	case 1: //RED
        		speed = speedcms_to_speedsteps(1);
        	case 2: //GREEN
        		speed = speedcms_to_speedsteps(2);
        		break;
        	case 3: //BLUE
        		speed = speedcms_to_speedsteps(3);
        		break;
        	default:
        		speed = speedcms_to_speedsteps(0);
        		break;
        	}

        	speed_correction = pid_regulator(get_middle_line());

        	//if the line is nearly in front of the camera, don't rotate
        	if(abs(speed_correction) < ROTATION_THRESHOLD){
        		speed_correction = 0;
        	}

        	//applies the speed from the PI regulator and the correction for the rotation
        	right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
        	left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
        }

        //Obstacle Avoidance
        else {
        	obstacle_mode = 1;

        	//Simple PseudoCode to avoid cylindrical shapes with known radius
        	//Move backwards 5cm at speed 7cm/s
//      	motor_set_position(5, 5, 7, 7);
//        	while(motor_position_reached() != POSITION_REACHED);
        	if (ir_front_left > ir_front_right){
        		//Rotate CW 90deg
//        		motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, -vitesse, vitesse);
//        		while(motor_position_reached() != POSITION_REACHED);

        		//Half Circle Trajectory to avoid obstacle
        		//mov_circ_left(vitesse,rayon_obstacle + marge,PI);

        		//Rotate CW 90deg
        		//motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, -vitesse, vitesse);
        		//while(motor_position_reached() != POSITION_REACHED);
        	}
        	else {
        		//Rotate CCW 90deg
        		//motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, vitesse, -vitesse);
        		//while(motor_position_reached() != POSITION_REACHED);

        		//Half Circle Trajectory to avoid obstacle
        		//mov_circ_right(vitesse,rayon_obstacle + marge,PI);

        		//Rotate CCW 90deg
        		//motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, vitesse, -vitesse);
        		//while(motor_position_reached() != POSITION_REACHED);

        	}

        	//More Complex PseudoCode to avoid simple shapes like squares and cylinders :
        	//If IR3 > IR4
        	//Then Rotate robot until IR2 is maximal (remember the angle of rotation)
        	//Then advance (if IR2 is diminishing its a circle -> launch circular trajectory, else its a square)
        	//In the square case, advance until IR1 is small and advance some extra for the camera setup
        	//Rotate by CCW 90deg and advance until finding line
        	//When line found, rotate by the angle previously saved
        	//Else
        	//Then Rotate robot until IR5 is maximal (remember the angle of rotation)
        	//Then advance (if IR6 is diminishing its a circle -> launch circular trajectory, else its a square)
        	//In the square case, advance until IR1 is small and advance some extra for the camera setup
        	//Rotate by CW 90deg and advance until finding line
        	//When line found, rotate by the angle previously saved
        	//After avoidance finished -> Set obstacle_mode to 0

        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pid_regulator_start(void){
	chThdCreateStatic(waPidRegulator, sizeof(waPidRegulator), NORMALPRIO, PidRegulator, NULL);
}

float speedcms_to_speedsteps (uint8_t speed_cms) {
	return speed_cms * NSTEP_ONE_TURN / WHEEL_PERIMETER;
}

//karim
void motor_set_position(float position_r, float position_l, float speed_r, float speed_l){

	int8_t POSITION_REACHED = 0;
	int16_t position_to_reach_left = right_motor_get_pos() + position_l * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	int16_t position_to_reach_right = left_motor_get_pos() -position_r * NSTEP_ONE_TURN / WHEEL_PERIMETER;

	while (!POSITION_REACHED){
	left_motor_set_speed(speed_l);
	right_motor_set_speed(speed_r);
	if (right_motor_get_pos()==position_to_reach_right && left_motor_get_pos()==position_to_reach_left ){
		POSITION_REACHED=1;
	}
	}
}
// Ancien code utilise pour le TP2, faire un truc semblable pour eviter des cylindres
//vitesse en cm/s //rayon en cm //angle en radian //mode = 0 cercle par top //mode = 1 cercle par bot
//void mov_circ_right(float vitesse,float rayon,float angle, int mode){
//	float dg = (rayon+ECART_ROUE/2)*angle;
//	float dd = (rayon-ECART_ROUE/2)*angle;
//	float vg = vitesse;
//	float vd = dd*vg/dg;
//	if (mode == 1){
//		vd = -vd;
//		vg = -vg;
//	}
//	motor_set_position(dd,dg,vd,vg);
//	while(motor_position_reached() != POSITION_REACHED);
//}
//
//void mov_circ_left(float vitesse,float rayon,float angle, int mode){
//	float dg = (rayon-ECART_ROUE/2)*angle;
//	float dd = (rayon+ECART_ROUE/2)*angle;
//	float vd = vitesse;
//	float vg = dg*vd/dd;
//	if (mode == 1){
//		vd = -vd;
//		vg = -vg;
//	}
//	motor_set_position(dd,dg,vd,vg);
//	while(motor_position_reached() != POSITION_REACHED);
//}
