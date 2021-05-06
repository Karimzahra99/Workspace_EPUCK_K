#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <main.h>
#include <motors.h>
#include <pid_regulator.h>
#include "read_image.h"

//Distances parameters
#define PI                 			3.1415926536f
#define WHEEL_PERIMETER     		13 		//[cm]
#define NSTEP_ONE_TURN      		1000	// number of step for 1 turn of the motor
#define WHEEL_DISTANCE      		5.30f    //cm
#define PERIMETER_EPUCK     		(PI * WHEEL_DISTANCE)

//PID Parameters
#define KP							100.0f
#define KI 							3.5f	//must not be zero
#define KD							0.0f	//a tuner -> utiliser deuxieme methode de ZN avec Ku et Pu
#define MAX_SUM_ERROR 				(MOTOR_SPEED_LIMIT/KI)

//Threshold des IR
#define	IR_THRESHOLD				250

//Level des leds
#define LED_ON						10
#define LED_OFF						0

typedef enum {
	LED_RGB_2 = 0,
	LED_RGB_4,
	LED_RGB_6,
	LED_RGB_8
} rgb_leds_index_t;

typedef enum {
	SENSOR_IR1 = 1,
	SENSOR_IR2,
	SENSOR_IR3,
	SENSOR_IR4,
	SENSOR_IR5,
	SENSOR_IR6
} ir_sensors_index_t;

typedef enum {
	STRAIGHT_LINE = 0,
	GO_BACK_10cm,
	TURN_180deg,
	TURNING_PID,
	OBS_AVOIDANCE,
	GO_BACK_5cm,
	TURN_ANGLE,
	ADVANCE
} STATE_t;

typedef struct {
	STATE_t mode;
	uint32_t counter;
}CONTEXT_t;


static CONTEXT_t rolling_context;



static uint8_t POSITION_REACHED = 0;

void motor_set_position(float position_r, float position_l, int16_t speed_r, int16_t speed_l);
void set_leds(uint8_t color_index);
int16_t cms_to_steps (int16_t speed_cms);
float cm_to_step (float cm);


////simple PI regulator implementation
int16_t pid_regulator(int16_t middle_diff){

	float speed_correction = 0;

	float error = (float)middle_diff;

	float derivative = 0;

	static float sum_error = 0;
	static float last_error = 0;

	sum_error += error;//sum_error = sum_error + error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	// Integral anti-windup (Anti-Reset Windup)
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	derivative = error - last_error;

	last_error = error;

	speed_correction = KP * error + KI * sum_error + KD*derivative;



	return (int16_t)speed_correction;
}


static THD_WORKING_AREA(waPidRegulator, 512);
static THD_FUNCTION(PidRegulator, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;


    while(1){
    	time = chVTGetSystemTime();



    	switch(rolling_context.mode){
    	case STRAIGHT_LINE :
    		//dosmth();
    		break;
    	case GO_BACK_10cm :
    		//dosmth();
    		break;
    	case TURN_180deg :
    		//dosmth();
    		break;
    	case TURNING_PID :
    		//dosmth();
    		break;
    	case OBS_AVOIDANCE :
    		//dosmth();
    		break;
    	case GO_BACK_5cm :
    		//dosmth();
    		break;
    	case TURN_ANGLE :
    		//dosmth();
    		break;
    	case ADVANCE :
    		//dosmth();
    		break;

    	default :
    		//dosmth();
    		break;
    	}

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pid_regulator_start(void){
	chThdCreateStatic(waPidRegulator, sizeof(waPidRegulator), NORMALPRIO, PidRegulator, NULL);
}

int16_t cms_to_steps (int16_t speed_cms) {
	return speed_cms * NSTEP_ONE_TURN / WHEEL_PERIMETER;
}

float cm_to_step (float cm) {
	return cm * NSTEP_ONE_TURN / WHEEL_PERIMETER;
}

void set_leds(color_index_t color_index){
	if (color_index == RED_IDX){
		set_rgb_led(LED_RGB_2, LED_ON, LED_OFF, LED_OFF);
		set_rgb_led(LED_RGB_4, LED_ON, LED_OFF, LED_OFF);
		set_rgb_led(LED_RGB_6, LED_ON, LED_OFF, LED_OFF);
		set_rgb_led(LED_RGB_8, LED_ON, LED_OFF, LED_OFF);
	}
	else {
		if (color_index == GREEN_IDX){
			set_rgb_led(LED_RGB_2, LED_OFF, LED_ON, LED_OFF);
			set_rgb_led(LED_RGB_4, LED_OFF, LED_ON, LED_OFF);
			set_rgb_led(LED_RGB_6, LED_OFF, LED_ON, LED_OFF);
			set_rgb_led(LED_RGB_8, LED_OFF, LED_ON, LED_OFF);
		}
		else {
			if (color_index == BLUE_IDX){
				set_rgb_led(LED_RGB_2, LED_OFF, LED_OFF, LED_ON);
				set_rgb_led(LED_RGB_4, LED_OFF, LED_OFF, LED_ON);
				set_rgb_led(LED_RGB_6, LED_OFF, LED_OFF, LED_ON);
				set_rgb_led(LED_RGB_8, LED_OFF, LED_OFF, LED_ON);
			}
			else {
				if (color_index == YELLOW_IDX){
					set_rgb_led(LED_RGB_2, LED_ON, LED_ON, LED_OFF);
					set_rgb_led(LED_RGB_4, LED_ON, LED_ON, LED_OFF);
					set_rgb_led(LED_RGB_6, LED_ON, LED_ON, LED_OFF);
					set_rgb_led(LED_RGB_8, LED_ON, LED_ON, LED_OFF);
				}
				else {
					if (color_index == PURPLE_IDX){
						set_rgb_led(LED_RGB_2, LED_ON, LED_OFF, LED_ON);
						set_rgb_led(LED_RGB_4, LED_ON, LED_OFF, LED_ON);
						set_rgb_led(LED_RGB_6, LED_ON, LED_OFF, LED_ON);
						set_rgb_led(LED_RGB_8, LED_ON, LED_OFF, LED_ON);
					}
					else {
						if (color_index == NO_COLOR){
							set_rgb_led(LED_RGB_2, LED_OFF, LED_OFF, LED_OFF);
							set_rgb_led(LED_RGB_4, LED_OFF, LED_OFF, LED_OFF);
							set_rgb_led(LED_RGB_6, LED_OFF, LED_OFF, LED_OFF);
							set_rgb_led(LED_RGB_8, LED_OFF, LED_OFF, LED_OFF);
						}
					}
				}
			}
		}
	}
}

//position in cm and speed en cm/s
//int : -2^32/2 to 2^32/2-1
//motor set position -2^31/2 to 2^31/2-1
void motor_set_position(float position_r, float position_l, int16_t speed_r, int16_t speed_l){

	POSITION_REACHED = 0;
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	int position_to_reach_left = cm_to_step(position_l);
	int position_to_reach_right = - cm_to_step(position_r);

	while (!POSITION_REACHED){
		left_motor_set_speed(cms_to_steps(speed_l));
		right_motor_set_speed(cms_to_steps(speed_r));

		if (abs(right_motor_get_pos()) > abs(position_to_reach_right) && abs(left_motor_get_pos()) > abs(position_to_reach_left) ){
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			POSITION_REACHED=1;
		}
	}
}
