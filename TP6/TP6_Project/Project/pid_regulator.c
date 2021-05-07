#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include<stdbool.h>
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
#define KP_R						100.0f
#define KI_R						3.5f
#define KD_R						0.0f
#define KP_G						100.0f
#define KI_G						3.5f
#define KD_G						0.0f
#define KP_B						100.0f
#define KI_B						3.5f
#define KD_B						0.0f
#define MAX_SUM_ERROR_R 			(MOTOR_SPEED_LIMIT/KI_R)
#define MAX_SUM_ERROR_G 			(MOTOR_SPEED_LIMIT/KI_G)
#define MAX_SUM_ERROR_B 			(MOTOR_SPEED_LIMIT/KI_B)

//Distance to travel with middle_diff < DEAD_ZONE_WIDTH to go back to STRAIGHT_LINE_BACKWARDS mode
#define STRAIGHT_LINE_COUNT			10000

//Threshold des IR
#define	IR_THRESHOLD				250

//Level des leds
#define LED_ON						10
#define LED_OFF						0

//Color speeds
#define LOW_SPEED					2
#define MEDIUM_SPEED				4
#define HIGH_SPEED 					6

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
	NOT_REACHED = 0,
	REACHED
} position_status_t;

typedef enum {
	STRAIGHT_LINE_BACKWARDS = 0,
	PID_FRONTWARDS,
	OBS_AVOIDANCE,
} STATE_t;

typedef struct {
	STATE_t mode;
	uint32_t counter;
	color_index_t color;
	int16_t speed;
	position_status_t position_reached;
} MOVE_CONTEXT_t;

static MOVE_CONTEXT_t rolling_context;

void motor_set_position(float position_r, float position_l, int16_t speed_r, int16_t speed_l);
void set_leds(uint8_t color_index);
int16_t cms_to_steps (int16_t speed_cms);
int cm_to_step (float cm);
void move_straight_backwards(void);
void pid_front(void);
void init_context(void);
void prepare_pid_front(void);
void avoid_obs(void);

//PID Implementation
int16_t pid_regulator(int16_t middle_diff){

	float speed_correction = 0;

	float error = (float)middle_diff;

	float derivative = 0;

	static float sum_error = 0;
	static float last_error = 0;

	sum_error += error;

	if (rolling_context.color == RED_IDX){
		if(sum_error > MAX_SUM_ERROR_R){
			sum_error = MAX_SUM_ERROR_R;
		}else if(sum_error < -MAX_SUM_ERROR_R){
			sum_error = -MAX_SUM_ERROR_R;
		}
	}

	if (rolling_context.color == GREEN_IDX){
		if(sum_error > MAX_SUM_ERROR_G){
			sum_error = MAX_SUM_ERROR_G;
		}else if(sum_error < -MAX_SUM_ERROR_G){
			sum_error = -MAX_SUM_ERROR_G;
		}
	}

	if (rolling_context.color == BLUE_IDX){
		if(sum_error > MAX_SUM_ERROR_B){
			sum_error = MAX_SUM_ERROR_B;
		}else if(sum_error < -MAX_SUM_ERROR_B){
			sum_error = -MAX_SUM_ERROR_B;
		}
	}

	derivative = error - last_error;

	last_error = error;

	if (rolling_context.color == RED_IDX){
		speed_correction = KP_R * error + KI_R * sum_error + KD_R * derivative;
	}

	if (rolling_context.color == GREEN_IDX){
		speed_correction = KP_G * error + KI_G * sum_error + KD_G * derivative;
	}

	if (rolling_context.color == BLUE_IDX){
		speed_correction = KP_B * error + KI_B * sum_error + KD_B * derivative;
	}

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
		case STRAIGHT_LINE_BACKWARDS :
			move_straight_backwards();
			break;
		case PID_FRONTWARDS :
			pid_front();
			break;
		case OBS_AVOIDANCE :
			avoid_obs();
			break;
		default :
			move_straight_backwards();
			break;
		}

		//100Hz
		chThdSleepUntilWindowed(time, time + MS2ST(10));
	}
}

bool check_ir_front(void){

	if ((get_prox(SENSOR_IR3) > IR_THRESHOLD) && (get_prox(SENSOR_IR4) > IR_THRESHOLD)){
		return true;
	}
	else return false;

}

void init_context(void){
	rolling_context.mode = STRAIGHT_LINE_BACKWARDS;
	rolling_context.counter = 0;
	rolling_context.color = NO_COLOR;
	rolling_context.speed = 2;
	rolling_context.position_reached = NOT_REACHED;
}

void move_straight_backwards(void){
	if (check_ir_front()){
		rolling_context.color = get_color();
		rolling_context.mode = OBS_AVOIDANCE;
	}
	else {
		if (get_middle_diff()>DEAD_ZONE_WIDTH){
			rolling_context.color = get_color();
			prepare_pid_front();
			rolling_context.mode = PID_FRONTWARDS;
		}
		else {
			color_index_t color = get_color();
			switch (color)
			{
			case 0: //NO COLOR
				set_leds(color);
				rolling_context.speed = 0;
				break;
			case 1: //RED
				set_leds(color);
				rolling_context.speed = cms_to_steps(LOW_SPEED);
				break;
			case 2: //GREEN
				set_leds(color);
				rolling_context.speed = cms_to_steps(MEDIUM_SPEED);
				break;
			case 3: //BLUE
				set_leds(color);
				rolling_context.speed = cms_to_steps(HIGH_SPEED);
				break;
			default:
				rolling_context.speed = cms_to_steps(MEDIUM_SPEED);
				break;
			}
			//rolling backwards
			right_motor_set_speed(-rolling_context.speed);
			left_motor_set_speed(-rolling_context.speed);
		}
	}
}

void prepare_pid_front(void){
	motor_set_position(10, 10, rolling_context.speed, rolling_context.speed);
	motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, rolling_context.speed, -rolling_context.speed);
}

void prepare_to_follow_line(void){
	motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, rolling_context.speed, -rolling_context.speed);
}

void pid_front(void){
	int16_t middle_diff = get_middle_diff();
	int16_t speed_corr = pid_regulator(middle_diff);
	right_motor_set_speed(rolling_context.speed - speed_corr);
	left_motor_set_speed(rolling_context.speed + speed_corr);

	if (middle_diff < DEAD_ZONE_WIDTH){
		rolling_context.counter = rolling_context.counter + 1;
		if (rolling_context.counter >= STRAIGHT_LINE_COUNT){
			prepare_to_follow_line();
			rolling_context.counter = 0;
			rolling_context.mode = STRAIGHT_LINE_BACKWARDS;
		}
	}

}

void avoid_obs(void){

	//to complete

}

void pid_regulator_start(void){
	init_context();
	chThdCreateStatic(waPidRegulator, sizeof(waPidRegulator), NORMALPRIO, PidRegulator, NULL);
}

int16_t cms_to_steps (int16_t speed_cms) {
	return speed_cms * NSTEP_ONE_TURN / WHEEL_PERIMETER;
}

int cm_to_step (float cm) {
	return (int)(cm * NSTEP_ONE_TURN / WHEEL_PERIMETER);
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

	rolling_context.position_reached = NOT_REACHED;
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	int position_to_reach_left = cm_to_step(position_l);
	int position_to_reach_right = - cm_to_step(position_r);

	left_motor_set_speed(cms_to_steps(speed_l));
	right_motor_set_speed(cms_to_steps(speed_r));

	while (!rolling_context.position_reached){

		if (abs(right_motor_get_pos()) > abs(position_to_reach_right) && abs(left_motor_get_pos()) > abs(position_to_reach_left) ){
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			rolling_context.position_reached = REACHED;
		}
	}
}

