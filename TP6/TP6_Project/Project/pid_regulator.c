#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pid_regulator.h>
#include <process_image.h>

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

        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        switch (get_color())
        {
        case 0:
        	speed = 0;
        	break;
        case 1: //RED
        	speed = speedcms_to_speedsteps(1);
        	break;
        case 2: //GREEN
        	speed = speedcms_to_speedsteps(2);
        	break;
        case 3: //BLUE
        	speed = speedcms_to_speedsteps(3);
        	break;
        default:
        	speed = speedcms_to_speedsteps(2);
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
