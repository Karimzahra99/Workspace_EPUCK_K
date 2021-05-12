#include "moving.h"
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include<stdbool.h>
#include <sensors/proximity.h>
#include <main.h>
#include <motors.h>
#include "read_image.h"
#include <audio/play_melody.h>
#include <leds.h>

//Distances parameters
#define PI                 			3.1415926536f
#define WHEEL_PERIMETER     		13 		//[cm]
#define NSTEP_ONE_TURN      		1000	// number of step for 1 turn of the motor
#define WHEEL_DISTANCE      		5.30f    //cm
#define PERIMETER_EPUCK     		(PI * WHEEL_DISTANCE)

//PID Parameters
//0.2 0.02 pour 100 r230
#define KP_R						0.8f
#define KI_R						0.0f
#define KD_R						0.0f
#define KP_G						0.0f
#define KI_G						0.0f
#define KD_G						0.0f
#define KP_B						0.0f
#define KI_B						0.0f
#define KD_B						0.0f
#define MAX_SUM_ERROR_R 			200/KI_R //(MOTOR_SPEED_LIMIT/KI_R)
#define MAX_SUM_ERROR_G 			(MOTOR_SPEED_LIMIT/KI_G)
#define MAX_SUM_ERROR_B 			(MOTOR_SPEED_LIMIT/KI_B)

// Straight line correction zone
#define STRAIGHT_ZONE_WIDTH_MAX		75
#define STRAIGHT_ZONE_WIDTH_MIN		15

//Distance to travel with middle_diff < DEAD_ZONE_WIDTH to go back to STRAIGHT_LINE_BACKWARDS mode
#define STRAIGHT_LINE_COUNT			10000

//Threshold des IR
#define	IR_THRESHOLD				250

//Color speeds
#define SEARCH_SPEED				2
#define LOW_SPEED					3
#define MEDIUM_SPEED				4
#define HIGH_SPEED 					5

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

//bring back state_t

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
int16_t cms_to_steps (float speed_cms);
int cm_to_step (float cm);
int step_to_cm (int step);
void move_straight_backwards(void);
void pid_front(void);
void init_context(void);
void prepare_pid_front(void);
void avoid_obs(void);
void set_speed_with_color(void);
void find_next_color(void);
void help_me_please(void);

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

//		chprintf((BaseSequentialStream *)&SD3, "TOP =%-7d BOT =%-7d DIFF =%-7d COLOR =%-7d \r\n\n",
		//				get_middle_top(), get_middle_bot(), get_middle_diff(),get_color());

		switch(rolling_context.mode){
//		case STRAIGHT_LINE_BACKWARDS :
//			move_straight_backwards();
//			break;

		case PID_FRONTWARDS :
			pid_front();
			break;

//		case OBS_AVOIDANCE :
//			avoid_obs();
//			break;
//
//		case SEARCH_LINE :
//			find_next_color();
//			break;
//
//		case LOST :
//			help_me_please();
//			break;

		default :
			pid_front();
			break;
		}

		//100Hz
		chThdSleepUntilWindowed(time, time + MS2ST(10));
	}
}

bool check_ir_front(void){

	if ((get_calibrated_prox(SENSOR_IR3) > IR_THRESHOLD) && (get_calibrated_prox(SENSOR_IR4) > IR_THRESHOLD)){
		return true;
	}
	else return false;

}

void init_context(void){
	rolling_context.mode = STRAIGHT_LINE_BACKWARDS;
	rolling_context.counter = 0;
	rolling_context.color = get_color();

	rolling_context.speed = LOW_SPEED;
	rolling_context.position_reached = NOT_REACHED;
}

void move_straight_backwards(void){
	if (check_ir_front()){
		rolling_context.color = get_color();
		rolling_context.mode = OBS_AVOIDANCE;
	}
	else {
		if (abs(get_middle_diff()) > STRAIGHT_ZONE_WIDTH_MAX){
			//bad start case : start wasn't performed on a straight line
			set_leds(NO_LINE);
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}
		else {
			if ((abs(get_middle_diff())>STRAIGHT_ZONE_WIDTH_MIN) && (abs(get_middle_diff())<STRAIGHT_ZONE_WIDTH_MAX)){
				if (get_middle_diff()<0){
					if(abs(get_middle_diff()) > STRAIGHT_ZONE_WIDTH_MIN){
						rolling_context.color = get_color();
						right_motor_set_speed(0);
						left_motor_set_speed(cms_to_steps(0.8));
					}
					if ((abs(get_middle_diff()) > STRAIGHT_ZONE_WIDTH_MAX) || (get_middle_top() < 250) || (get_middle_bot() < 250) || (get_middle_top() > 420) || (get_middle_bot() > 420)) {

						prepare_pid_front();
					}
				}
				else {
					if(abs(get_middle_diff()) > STRAIGHT_ZONE_WIDTH_MIN){
						rolling_context.color = get_color();
						right_motor_set_speed(cms_to_steps(0.8));
						left_motor_set_speed(0);
					}
					if ((abs(get_middle_diff()) > STRAIGHT_ZONE_WIDTH_MAX) || (get_middle_top() < 250) || (get_middle_bot() < 250) || (get_middle_top() > 420) || (get_middle_bot() > 420)) {
						prepare_pid_front();
					}
				}
			}
			else {
				rolling_context.color = get_color();

				set_speed_with_color();

				//rolling backwards
				right_motor_set_speed(-rolling_context.speed);
				left_motor_set_speed(-rolling_context.speed);
			}
		}
	}
}

void prepare_pid_front(void){

	set_leds(PURPLE_IDX);

	rolling_context.counter = 0;

	motor_set_position(10, 10,  MEDIUM_SPEED,  MEDIUM_SPEED);

	motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, MEDIUM_SPEED, -MEDIUM_SPEED);

	rolling_context.mode = PID_FRONTWARDS;

}

void prepare_to_follow_line(void){
	motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, rolling_context.speed, -rolling_context.speed);
}

void pid_front(void){

	rolling_context.color = get_color();
	set_speed_with_color();

	int16_t middle_diff = get_middle_bot()- 320;
	int16_t speed_corr = pid_regulator(middle_diff);
	right_motor_set_speed(rolling_context.speed - speed_corr);
	left_motor_set_speed(rolling_context.speed + speed_corr);
//
//	if (middle_diff < DEAD_ZONE_WIDTH){
//		rolling_context.counter = rolling_context.counter + 1;
//		if (rolling_context.counter >= STRAIGHT_LINE_COUNT){
//			prepare_to_follow_line();
//			rolling_context.counter = 0;
//			rolling_context.mode = STRAIGHT_LINE_BACKWARDS;
//		}
//	}

}

void avoid_obs(void){
	//temporary function :
	set_leds(YELLOW_IDX);
	rolling_context.counter = 0;
	rolling_context.speed = 0;
	right_motor_set_speed(rolling_context.speed);
	left_motor_set_speed(rolling_context.speed);
	if (!check_ir_front()){
		rolling_context.mode = STRAIGHT_LINE_BACKWARDS;
	}
}

void moving_start(void){
	init_context();
	chThdCreateStatic(waPidRegulator, sizeof(waPidRegulator), NORMALPRIO, PidRegulator, NULL);
}

int16_t cms_to_steps (float speed_cms) {
	return (int16_t) (speed_cms * NSTEP_ONE_TURN / WHEEL_PERIMETER);
}

int cm_to_step (float cm) {
	return (int)(cm * NSTEP_ONE_TURN / WHEEL_PERIMETER);
}

//remove if not needed
int step_to_cm (int step) {
	return (step * WHEEL_PERIMETER / NSTEP_ONE_TURN);
}

void set_speed_with_color(void){

	switch (rolling_context.color)
	{
	case 0: //NO COLOR
		set_leds(FIND_COLOR);
		reset_middle_positions();
		rolling_context.counter = 0;
		rolling_context.speed = 0;
		left_motor_set_speed(0);
		right_motor_set_speed(0);
		left_motor_set_pos(0);
		right_motor_set_pos(0);
		rolling_context.mode = SEARCH_LINE;
		break;
	case 1: //RED
		set_leds(RED_IDX);
		rolling_context.speed = cms_to_steps(LOW_SPEED);
		break;
	case 2: //GREEN
		set_leds(GREEN_IDX);
		rolling_context.speed = cms_to_steps(MEDIUM_SPEED);
		break;
	case 3: //BLUE
		set_leds(BLUE_IDX);
		rolling_context.speed = cms_to_steps(HIGH_SPEED);
		break;
	default:
		rolling_context.speed = cms_to_steps(MEDIUM_SPEED);
		break;
	}

}

void find_next_color(void){

	rolling_context.speed = cms_to_steps(SEARCH_SPEED);
	right_motor_set_speed(-rolling_context.speed);
	left_motor_set_speed(-rolling_context.speed);
	rolling_context.counter = right_motor_get_pos();

	chprintf((BaseSequentialStream *)&SD3, "Counter =%-7d \r\n\n",rolling_context.counter);
	if (get_color() != NO_COLOR){
		rolling_context.color = get_color();
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		rolling_context.color = get_color();
		rolling_context.mode = STRAIGHT_LINE_BACKWARDS;
		chprintf((BaseSequentialStream *)&SD3, "Inside1 \r\n\n");
	}
	else {
		if (abs(rolling_context.counter) > 500){
			chprintf((BaseSequentialStream *)&SD3, "Inside2 \r\n\n");
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			rolling_context.counter = 0;
			rolling_context.mode = LOST;
		}
	}

}

void help_me_please(void){
	set_leds(NO_COLOR);

	playMelody(IMPOSSIBLE_MISSION, ML_SIMPLE_PLAY, NULL);

	if (rolling_context.counter == 0){
		set_body_led(1);
		rolling_context.counter = 1;
	}
	else{
		set_body_led(0);
		rolling_context.counter = 0;
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

STATE_t get_rolling_mode (void){
	return rolling_context.mode;
}
