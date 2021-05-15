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
#define KP_R						0.8f
#define KI_R						0.0f
#define KD_R						0.0f
#define KP_G						0.7f
#define KI_G						0.0f
#define KD_G						0.0f
#define KP_B						0.6f
#define KI_B						0.0f
#define KD_B						0.0f
#define KP_IR						0.2f
#define KI_IR						0
#define KD_IR						0.2f
#define MAX_SUM_ERROR_R 			(MOTOR_SPEED_LIMIT/KI_R)
#define MAX_SUM_ERROR_G 			(MOTOR_SPEED_LIMIT/KI_G)
#define MAX_SUM_ERROR_B 			(MOTOR_SPEED_LIMIT/KI_B)
#define MAX_SUM_ERROR_IR 			100

// Straight line correction zone
#define STRAIGHT_ZONE_WIDTH_MAX		200
#define STRAIGHT_ZONE_WIDTH_MIN		15

//Distance to travel with middle_diff < DEAD_ZONE_WIDTH to go back to STRAIGHT_LINE_BACKWARDS mode
#define STRAIGHT_LINE_COUNT			500

//Threshold des IR
#define	IR_THRESHOLD				250
#define IR_BRUIT_BLANC              10
#define IR_OBS_OFFSET				5

//Color speeds
#define ADJUST_SPEED 				1
#define OBS_CALIBRATION_SPEED		1
#define SEARCH_SPEED				2
#define LOW_SPEED					5.2f
#define MEDIUM_SPEED				6
#define HIGH_SPEED 					7

//middle position boundaries
#define MIN_MIDDLE					250
#define MAX_MIDDLE					450
#define MAX_DIFF_MIDDLE				35

//Blinking counter
#define DUTY_CYCLE_50				10

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

typedef struct {
	int16_t ir3_adjusted;
	int16_t ir2_adjusted;
	int16_t ir4_adjusted;
	int16_t ir5_adjusted;
	bool first_line_passed;
	bool obstacle_at_left;
} OBSTACLE_CONTEXT_t;

typedef struct {
	STATE_t mode;
	uint32_t counter;
	uint32_t counter2;
	color_index_t color;
	color_index_t prev_color;
	int16_t speed;
	position_status_t position_reached;
	OBSTACLE_CONTEXT_t obstacle_context;
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
void set_speed_with_color(void);
void find_next_color(void);
void help_me_please(void);
void init_pid_front(void);
int16_t pid_regulator(int16_t middle_diff);

void avoid_obs(void);
bool back_to_track(void);
void adjust (void);
void reset_obstacle_context(void);
int16_t rotate_until_irmax_left(void);
int16_t rotate_until_irmax_right(void);




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

		case SEARCH_LINE :
			find_next_color();
			break;

		case LOST :
			help_me_please();
			break;

		default :
			init_pid_front();
			break;
		}

		//100Hz
		chThdSleepUntilWindowed(time, time + MS2ST(10));
	}
}

bool check_ir_front(void){
	int16_t ir_front_left = get_calibrated_prox(SENSOR_IR3);
	int16_t ir_front_right = get_calibrated_prox(SENSOR_IR4);

	if ((ir_front_left > IR_THRESHOLD) || (ir_front_right > IR_THRESHOLD)){
		if (ir_front_left > ir_front_right){
			rolling_context.obstacle_context.obstacle_at_left = 1;
		}

		return true;
	}
	else return false;
}

void init_context(void){
	rolling_context.mode = STRAIGHT_LINE_BACKWARDS;
	set_frontwards_bool(false);
	rolling_context.counter = 0;
	rolling_context.counter2 = 0;
	rolling_context.color = get_color();
	rolling_context.prev_color = get_color();

	reset_obstacle_context();

	rolling_context.speed = LOW_SPEED;
	rolling_context.position_reached = NOT_REACHED;
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}

void move_straight_backwards(void){

	chprintf((BaseSequentialStream *)&SD3, "T =%-7d B =%-7d D =%-7d C =%-7d Mode =%-7d \r\n\n",
						get_middle_top(),get_middle_bot(), get_middle_diff(),get_color(),get_rolling_mode());

	chprintf((BaseSequentialStream *)&SD3, "Counter1 =%-7d Counter2 =%-7d\r\n\n",
						rolling_context.counter,rolling_context.counter2);

	if (check_ir_front()){
		rolling_context.color = get_color();
		set_leds(YELLOW_IDX);
		//adjust();
		rolling_context.mode = OBS_AVOIDANCE;
	}
	else {
		if ((abs(get_middle_diff()) > STRAIGHT_ZONE_WIDTH_MAX) ) {
								rolling_context.color = get_color();
								prepare_pid_front();
							}
//			if ((abs(get_middle_diff())>STRAIGHT_ZONE_WIDTH_MIN) && (abs(get_middle_diff())<STRAIGHT_ZONE_WIDTH_MAX)){
//				if (get_middle_diff() < 0){
//					if(abs(get_middle_diff()) > STRAIGHT_ZONE_WIDTH_MIN){
//						rolling_context.color = get_color();
//						right_motor_set_speed(0);
//						left_motor_set_speed(cms_to_steps(ADJUST_SPEED ));
//					}
//					if ((abs(get_middle_diff()) > STRAIGHT_ZONE_WIDTH_MAX)) {
//						prepare_pid_front();
//					}
//				}
//				else {
//					if(abs(get_middle_diff()) > STRAIGHT_ZONE_WIDTH_MIN){
//						rolling_context.color = get_color();
//						right_motor_set_speed(cms_to_steps(ADJUST_SPEED ));
//						left_motor_set_speed(0);
//					}
//					if ((abs(get_middle_diff()) > STRAIGHT_ZONE_WIDTH_MAX) ) {
//						rolling_context.color = get_color();
//						prepare_pid_front();
//					}
//				}
//			}
			else {
				if ((get_color() != NO_COLOR) && (get_color() != rolling_context.color)){
					rolling_context.counter ++;
					if (rolling_context.counter > 10){
						rolling_context.color = get_color();
					}
				}
				else{
					rolling_context.color = get_color();
					rolling_context.counter = 0;
				}


				set_speed_with_color();

				//rolling backwards
				right_motor_set_speed(-rolling_context.speed);
				left_motor_set_speed(-rolling_context.speed);
			}

	}
}


//void move_straight_backwards(void){
//	if (check_ir_front()){
//		rolling_context.color = get_color();
//				set_leds(YELLOW_IDX);
//				//adjust();
//				rolling_context.mode = OBS_AVOIDANCE;	}
//	else {
//		if (abs(get_middle_diff()) > STRAIGHT_ZONE_WIDTH_MAX){
//			//bad start case : start wasn't performed on a straight line
//			set_leds(NO_LINE);
//			left_motor_set_speed(0);
//			right_motor_set_speed(0);
//			playMelody(WE_ARE_THE_CHAMPIONS, ML_SIMPLE_PLAY, NULL);
//		}
//		else {
//			if ((abs(get_middle_diff())>STRAIGHT_ZONE_WIDTH_MIN) && (abs(get_middle_diff())<STRAIGHT_ZONE_WIDTH_MAX)){
//				if (get_middle_diff()<0){
//					if(abs(get_middle_diff()) > STRAIGHT_ZONE_WIDTH_MIN){
//						rolling_context.color = get_color();
//						right_motor_set_speed(0);
//						left_motor_set_speed(cms_to_steps(0.8));
//					}
//					if ((get_middle_top() < 100) || (get_middle_bot() < 100) || (get_middle_top() > 500) || (get_middle_bot() > 500)) {
//
//						prepare_pid_front();
//					}
//				}
//				else {
//					if(abs(get_middle_diff()) > STRAIGHT_ZONE_WIDTH_MIN){
//						rolling_context.color = get_color();
//						right_motor_set_speed(cms_to_steps(0.8));
//						left_motor_set_speed(0);
//					}
//					if ((get_middle_top() < 100) || (get_middle_bot() < 100) || (get_middle_top() > 500) || (get_middle_bot() > 500)) {
//						prepare_pid_front();
//					}
//				}
//			}
//			else {
//				rolling_context.color = get_color();
//
//				set_speed_with_color();
//
//				//rolling backwards
//				right_motor_set_speed(-rolling_context.speed);
//				left_motor_set_speed(-rolling_context.speed);
//			}
//		}
//	}
//}

void prepare_pid_front(void){

	set_leds(PURPLE_IDX);

	rolling_context.counter = 0;

	rolling_context.counter2 = 0;

	motor_set_position(10, 10,  SEARCH_SPEED,  SEARCH_SPEED);

	motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, SEARCH_SPEED, -SEARCH_SPEED);

	right_motor_set_speed(0);

	left_motor_set_speed(0);

	set_frontwards_bool(true);

	chThdSleepMilliseconds(1000);

	rolling_context.mode = PID_FRONTWARDS;



}

void prepare_to_follow_line(void){

	motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2,SEARCH_SPEED, -SEARCH_SPEED);

	rolling_context.counter = 0;

	rolling_context.counter2 = 0;

	set_frontwards_bool(false);

	right_motor_set_speed(0);

	left_motor_set_speed(0);

	chThdSleepMilliseconds(1000);

	rolling_context.mode = STRAIGHT_LINE_BACKWARDS;

}

void pid_front(void){

	if ((get_color() != NO_COLOR) && (get_color() != rolling_context.color)){
		rolling_context.counter2 ++;
		if (rolling_context.counter2 > 10){
			rolling_context.color = get_color();
		}
	}
	else{
		rolling_context.color = get_color();
		rolling_context.counter2 = 0;
	}

	rolling_context.color = get_color();
	set_speed_with_color();

	int16_t middle_diff = get_middle_bot()- 320; //change to top after
	int16_t speed_corr = pid_regulator(middle_diff);
	right_motor_set_speed(rolling_context.speed - speed_corr);
	left_motor_set_speed(rolling_context.speed + speed_corr);


	if (abs(get_middle_diff())<35){

		rolling_context.counter = rolling_context.counter + 1;

		if (rolling_context.counter >= 200){
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			rolling_context.counter = 0;
			rolling_context.counter2 = 0;
			prepare_to_follow_line();
		}
	}
	else {
		rolling_context.counter = 0;
	}
}




void reset_obstacle_context(void){
	rolling_context.obstacle_context.ir2_adjusted = 0;
	rolling_context.obstacle_context.ir3_adjusted = 0;
	rolling_context.obstacle_context.ir4_adjusted = 0;
	rolling_context.obstacle_context.ir5_adjusted = 0;
	rolling_context.obstacle_context.first_line_passed = 0;
	rolling_context.obstacle_context.obstacle_at_left = 0;
}





void moving_start(void){

	init_context();

	chThdCreateStatic(waPidRegulator, sizeof(waPidRegulator), NORMALPRIO, PidRegulator, NULL);
}



void set_speed_with_color(void){

	switch (rolling_context.color)
	{
	case 0: //NO COLOR
		set_leds(FIND_COLOR);
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

	if (get_color() != NO_COLOR){
		rolling_context.color = get_color();
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		rolling_context.color = get_color();
		rolling_context.mode = STRAIGHT_LINE_BACKWARDS;
	}
	else {
		if (abs(rolling_context.counter) > STRAIGHT_LINE_COUNT){
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
	if (rolling_context.counter < DUTY_CYCLE_50){
		set_body_led(LED_ON);
		rolling_context.counter ++;
	}
	else{
		set_body_led(LED_OFF);
		rolling_context.counter ++;
		if (rolling_context.counter == 2*DUTY_CYCLE_50){
			rolling_context.counter = 0;
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

STATE_t get_rolling_mode (void){
	return rolling_context.mode;
}


bool back_to_track(void){
	if (get_color() != NO_COLOR){
		if (!rolling_context.obstacle_context.first_line_passed){
			rolling_context.obstacle_context.first_line_passed = 1;
			motor_set_position(5, 5, -SEARCH_SPEED, -SEARCH_SPEED);
		}
		else{
			if (rolling_context.obstacle_context.obstacle_at_left){
				left_motor_set_speed(-cms_to_steps(0));
				right_motor_set_speed(-cms_to_steps(0));
				motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, SEARCH_SPEED, -SEARCH_SPEED);
			}
			else {
				left_motor_set_speed(-cms_to_steps(0));
				right_motor_set_speed(-cms_to_steps(0));
				motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, -SEARCH_SPEED, SEARCH_SPEED);
			}
			return 1;
		}
	}
	return 0;
}

void adjust (void){
	if (rolling_context.obstacle_context.obstacle_at_left){
		rolling_context.obstacle_context.ir2_adjusted = rotate_until_irmax_left();
		rolling_context.obstacle_context.ir3_adjusted = get_calibrated_prox(SENSOR_IR3);
	}
	else {
		rolling_context.obstacle_context.ir5_adjusted = rotate_until_irmax_right();
		rolling_context.obstacle_context.ir4_adjusted = get_calibrated_prox(SENSOR_IR4);
	}
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

void avoid_obs(void){
	right_motor_set_speed(0);
	left_motor_set_speed(0);


//	int16_t speed_correction=0;

//	if (rolling_context.obstacle_context.obstacle_at_left){
//		speed_correction = pid_regulator_IR(rolling_context.obstacle_context.ir2_adjusted);
//		int16_t ir3_new = get_calibrated_prox(SENSOR_IR3);
//		if (ir3_new < rolling_context.obstacle_context.ir3_adjusted - IR_BRUIT_BLANC){
//			left_motor_set_speed(-cms_to_steps(SEARCH_SPEED) + speed_correction);
//			right_motor_set_speed(-cms_to_steps(SEARCH_SPEED) - speed_correction);
//		}
//		else if (ir3_new > rolling_context.obstacle_context.ir3_adjusted + IR_BRUIT_BLANC  ){
//			left_motor_set_speed(-cms_to_steps(SEARCH_SPEED)- speed_correction);
//			right_motor_set_speed(-cms_to_steps(SEARCH_SPEED)+ speed_correction);
//		}
//		else {
//			left_motor_set_speed(-cms_to_steps(SEARCH_SPEED));
//			right_motor_set_speed(-cms_to_steps(SEARCH_SPEED));
//		}
//
//	}
//	else{
//		speed_correction = pid_regulator_IR(rolling_context.obstacle_context.ir5_adjusted);
//		int16_t ir4_new = get_calibrated_prox(SENSOR_IR4);
//		//				chprintf((BaseSequentialStream *)&SD3, " ir4new =%-7d ir4old =%-7d speedcorr =%-7d\r\n\n", ir4_new, obstacle_context.ir4_adjusted, speed_correction);
//		if (ir4_new < rolling_context.obstacle_context.ir4_adjusted - IR_BRUIT_BLANC){
//			left_motor_set_speed(-cms_to_steps(SEARCH_SPEED) - speed_correction);
//			right_motor_set_speed(-cms_to_steps(SEARCH_SPEED) + speed_correction);
//		}
//		else if (ir4_new > rolling_context.obstacle_context.ir4_adjusted + IR_BRUIT_BLANC){
//			left_motor_set_speed(-cms_to_steps(SEARCH_SPEED)+ speed_correction);
//			right_motor_set_speed(-cms_to_steps(SEARCH_SPEED)- speed_correction);
//		}
//		else {
//			left_motor_set_speed(-cms_to_steps(SEARCH_SPEED));
//			right_motor_set_speed(-cms_to_steps(SEARCH_SPEED));
//		}
//
//	}
//
//	if(back_to_track()){
//		reset_obstacle_context();
//		rolling_context.mode = STRAIGHT_LINE_BACKWARDS;
//	}
}


//PID Implementation
int16_t rotate_until_irmax_left(void)
{
	int16_t	ir_left_ancien = 0;
	int16_t	ir_left_nouvau = 0;
	uint8_t start = 0;
	while ((ir_left_nouvau > ir_left_ancien + IR_OBS_OFFSET) || start == 0){
		start = 1;
		ir_left_ancien = get_calibrated_prox(SENSOR_IR2);
		motor_set_position(PERIMETER_EPUCK/16, PERIMETER_EPUCK/16, -OBS_CALIBRATION_SPEED, OBS_CALIBRATION_SPEED);
		ir_left_nouvau = get_calibrated_prox(SENSOR_IR2);
	}
	motor_set_position(PERIMETER_EPUCK/16, PERIMETER_EPUCK/16, OBS_CALIBRATION_SPEED, -OBS_CALIBRATION_SPEED);
	return ir_left_ancien;
}

int16_t rotate_until_irmax_right(void)
{
	int16_t	ir_right_ancien = 0;
	int16_t	ir_right_nouvau = 0;
	uint8_t start = 0;
	while ((ir_right_nouvau > ir_right_ancien + IR_OBS_OFFSET) || start == 0){
		start = 1;
		ir_right_ancien = get_calibrated_prox(SENSOR_IR5);
		motor_set_position(PERIMETER_EPUCK/16, PERIMETER_EPUCK/16, OBS_CALIBRATION_SPEED, -OBS_CALIBRATION_SPEED);
		ir_right_nouvau = get_calibrated_prox(SENSOR_IR5);
	}
	motor_set_position(PERIMETER_EPUCK/16, PERIMETER_EPUCK/16, -OBS_CALIBRATION_SPEED, OBS_CALIBRATION_SPEED);
	return ir_right_ancien;
}

void init_pid_front(void){
	rolling_context.counter = 0;
	rolling_context.counter2 = 0;
	pid_front();
};

int16_t pid_regulator_IR(int16_t middle){

	int16_t goal = middle;
	float error = 0;
	float speed = 0;
	float derivative = 0;

	static float sum_error = 0;
	static float last_error = 0;

	if (rolling_context.obstacle_context.obstacle_at_left){
		error =  get_calibrated_prox(SENSOR_IR2) - goal;
	}
	else{
		error =  get_calibrated_prox(SENSOR_IR5) - goal;
	}
	sum_error += error;

	if(sum_error > MAX_SUM_ERROR_IR){
		sum_error = MAX_SUM_ERROR_IR;
	}else if(sum_error < -MAX_SUM_ERROR_IR){
		sum_error = -MAX_SUM_ERROR_IR;
	}

	derivative = error - last_error ;

	last_error = error;

	speed = KP_IR * error + KI_IR * sum_error + KD_IR * derivative;
	return (int16_t) speed;
}


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
		speed_correction = KP_R * error; //+ KI_R * sum_error + KD_R * derivative;
	}

	if (rolling_context.color == GREEN_IDX){
		speed_correction = KP_G * error + KI_G * sum_error + KD_G * derivative;
	}

	if (rolling_context.color == BLUE_IDX){
		speed_correction = KP_B * error + KI_B * sum_error + KD_B * derivative;
	}

	return (int16_t)speed_correction;
}
