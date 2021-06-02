#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <proximity.h>

#define LOW_SPEED				500
#define MEDIUM_SPEED			800
typedef enum {
	LOW = 0,
	MEDIUM,
	STOP,
} STATE_t;

static STATE_t speed_context;

void change_speed(void);

//simple PI regulator implementation
int16_t pi_regulator(void);

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();

        switch(speed_context){
		case LOW :
        	speed = LOW_SPEED;
			break;

		case MEDIUM :
			speed = MEDIUM_SPEED;
			break;

		case STOP :
        	speed = 0;
			break;
		default:
			speed = 0;
			break;
        }
        change_speed();
        speed_correction = pi_regulator();
		right_motor_set_speed(speed -  speed_correction);
		left_motor_set_speed(speed + speed_correction);
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void change_speed(void){
	if (get_prox(2) > 200){
		speed_context = LOW;
		}
	if (get_prox(5) > 200){
		speed_context = MEDIUM;
		}
	if (get_prox(3) > 200 || get_prox(4) > 200){
		speed_context = STOP;
		}
//    chprintf((BaseSequentialStream *)&SD3, " ir4new =%-7d ir4old =%-7d speedcorr =%-7d\r\n\n", ir4_new, ir4_old, speed_correction)


}

int16_t pi_regulator(void){

	float error = 0;
	float speed = 0;
	float derivative = 0;
	static float sum_error = 0;
	static float old_error = 0;

	error = get_middle() - IMAGE_BUFFER_SIZE/2;



	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

////	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
//	if(sum_error > MAX_SUM_ERROR){
//		sum_error = MAX_SUM_ERROR;
//	}else if(sum_error < -MAX_SUM_ERROR){
//		sum_error = -MAX_SUM_ERROR;
//	}
	derivative = error - old_error ;

	old_error = error;
	if (speed_context==MEDIUM){
	speed = KP_M * error + KD_M * derivative;
	}
	else if (speed_context==LOW){
	speed = KP_L * error + KD_L * derivative;
	}
    return (int16_t)speed;
}


void pi_regulator_start(void){
	speed_context = STOP;
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}

