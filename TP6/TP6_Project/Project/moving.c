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
#define MAX_LINE_WIDTH 				4

//PID Parameters
//0.2 pour 100 pas
#define KP_R						0.8f
#define KI_R						0.0f//4*0.02f
#define KD_R						0.0f
#define KP_G						0.7f
#define KI_G						0.0f
#define KD_G						0.0f
#define KP_B						0.6f
#define KI_B						0.0f
#define KD_B						0.0f
#define MAX_SUM_ERROR_R 			(MOTOR_SPEED_LIMIT/KI_R)
#define MAX_SUM_ERROR_G 			(MOTOR_SPEED_LIMIT/KI_G)
#define MAX_SUM_ERROR_B 			(MOTOR_SPEED_LIMIT/KI_B)
#define BACKWARD_CORR_SPEED			0.8

// Straight line correction zone
#define STRAIGHT_ZONE_WIDTH_MAX		300
#define STRAIGHT_ZONE_WIDTH_MIN		20
#define MIDDLE_LINE_MIN				100
#define MIDDLE_LINE_MAX				500

//Distance to travel with middle_diff < DEAD_ZONE_WIDTH to go back to STRAIGHT_LINE_BACKWARDS mode
#define STRAIGHT_LINE_COUNT			1000

//Threshold des IR
#define	IR_THRESHOLD				250
#define IR_BRUIT_BLANC              10


//Color speeds
#define SEARCH_SPEED				2
#define LOW_SPEED					5.2
#define MEDIUM_SPEED				7.15
#define HIGH_SPEED 					9.1

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
	STATE_t mode;
	uint32_t counter;
	color_index_t color;
	int16_t speed;
	position_status_t position_reached;
} MOVE_CONTEXT_t;

typedef struct {
bool first_line_passed;
uint32_t ir3_adjusted;
uint32_t ir2_adjusted;
bool obstacle_at_left;
uint32_t ir4_adjusted;
uint32_t ir5_adjusted;
} OBSTACLE_CONTEXT_t;

static OBSTACLE_CONTEXT_t obstacle_context;
static MOVE_CONTEXT_t rolling_context;

///////////// INTERN FUNCTIONS /////////////

// motor goes to defined position at defines speed
void motor_set_position(float position_r, float position_l, int16_t speed_r, int16_t speed_l);
// set leds depending on color observed
void set_leds(uint8_t color_index);
//converts cm/sec speed to motor step/sec
int16_t cms_to_steps (float speed_cms);
//converts cm to motor step
int cm_to_step (float cm);
//converts motor step to step
int step_to_cm (int step);
// move backwards untill difference between botom and top is too big
void move_straight_backwards(void);
//follow line with camera on front and if line is straight for a while turn 180 degrees and start moving backwards
void pid_front(void);
// give initial values to rolling context structure parameters
void init_context(void);
// go back a little and rotate 180 degrees
void prepare_pid_front(void);
// turn around any obstacle until back to track
void avoid_obs(void);
// each color has a different speed
void set_speed_with_color(void);
// move backward until a color is found, if not you're lost
void find_next_color(void);
// I'm lost in the dark
void help_me_please(void);
// check if an obstacle is ahead (while moving backwards)
bool check_ir_front(void);
// neglige first line and look for the second one
bool back_to_track(void);
// adjust parallel to obstacle
void adjust (void);
// reset all obstacle context structures to zero
void reset_obstacle_context(void);
// rotate until a color is found
void rotate_till_color(bool left_obs);
//PID regulator implementation to manage line alignment in front
int16_t pid_regulator_line(int16_t middle_diff);
//PID regulator implementation to manage distance between obstacle and IR sensors
int16_t pid_regulator_ir(int middle);
// scanning of obstacle surface and return the maximum value found when positioned parallel
//to obstacle at left and right
int rotate_until_irmax_left(void);
int rotate_until_irmax_right(void);


//////////// THREAD /////////////

/*
*	Main Finite State Machine
*   Determines the robot behavior depending on its environment
*
*	params :
*   -
*
*/
static THD_WORKING_AREA(waPidRegulator, 512);
static THD_FUNCTION(PidRegulator, arg) {
	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;


	while(1){
		time = chVTGetSystemTime();

//		chprintf((BaseSequentialStream *)&SD3, "TOP =%-7d BOT =%-7d DIFF =%-7d COLOR =%-7d \r\n\n",
//						get_middle_top(), get_middle_bot(), get_middle_diff(),get_color());

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

		case SEARCH_LINE :	// continue moving backwards a little more while searching for a line
			find_next_color();
			break;

		case LOST :
			help_me_please();
			break;

		case ROTATE_TILL_COLOR :
			 rotate_till_color(obstacle_context.obstacle_at_left);
			 break;

		default :
			find_next_color();;
			break;
		}

		//100Hz
		chThdSleepUntilWindowed(time, time + MS2ST(10));
	}
}

bool check_ir_front(void){
	int ir_front_left = get_calibrated_prox(SENSOR_IR3);
	int ir_front_right = get_calibrated_prox(SENSOR_IR4);

	if ((ir_front_left > IR_THRESHOLD) || (ir_front_right > IR_THRESHOLD)){
		//obstacle found in front
		if (ir_front_left > ir_front_right){
			obstacle_context.obstacle_at_left = 1;
		}
		set_leds(YELLOW_IDX);
		adjust();
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
	left_motor_set_pos(0);
	right_motor_set_pos(0);
}

void move_straight_backwards(void){
	rolling_context.color = get_color();
	set_speed_with_color();
	if (check_ir_front()){
		rolling_context.mode = OBS_AVOIDANCE;
	}
	else {
		//chprintf((BaseSequentialStream *)&SD3, " get_middle_diff() =%-7d \r\n\n", get_middle_diff());
		if ((get_middle_top() < MIDDLE_LINE_MIN) || (get_middle_bot() < MIDDLE_LINE_MIN) || (get_middle_top() > MIDDLE_LINE_MAX) || (get_middle_bot() > MIDDLE_LINE_MAX)) {
			// error too big
			prepare_pid_front();
			}
		else if ((abs(get_middle_diff())>STRAIGHT_ZONE_WIDTH_MIN)){
			// perform very small error correction
			if (get_middle_diff()<0){
					right_motor_set_speed(0);
					left_motor_set_speed(cms_to_steps(BACKWARD_CORR_SPEED));
			}
				else {
					right_motor_set_speed(cms_to_steps(BACKWARD_CORR_SPEED));
					left_motor_set_speed(0);
				}
		}
		else {
			//rolling straight backwards
			right_motor_set_speed(-rolling_context.speed);
			left_motor_set_speed(-rolling_context.speed);
		}
	}
}


void prepare_pid_front(void){

	set_leds(PURPLE_IDX);

	rolling_context.counter = 0;
	rolling_context.mode = PID_FRONTWARDS;

	// go back a little and rotate 180 degrees
	motor_set_position(10, 10,  MEDIUM_SPEED,  MEDIUM_SPEED);
	motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, SEARCH_SPEED, -SEARCH_SPEED);

	left_motor_set_pos(0);
	right_motor_set_pos(0);

	chThdSleepMilliseconds(500);



}


void pid_front(void){

	rolling_context.color = get_color();
	set_speed_with_color();

	int16_t middle_diff = get_middle_bot()- IMAGE_BUFFER_SIZE/2;
	int16_t speed_corr = pid_regulator_line(middle_diff);
	// if middle diff between top and bottom camera lines is > threshold, then it's not a straight line.
	if (abs(get_middle_diff())>30){
		left_motor_set_pos(0);
		right_motor_set_pos(0);
	}
		if ((right_motor_get_pos() >= cm_to_step(5))){
			motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, 5 , -5);
			left_motor_set_pos(0); //a essayer a la place de motor_init
			right_motor_set_pos(0);
			//motors_init();
			rolling_context.mode = STRAIGHT_LINE_BACKWARDS;
			// wait for camera to get botom and top middle
			chThdSleepMilliseconds(1000);

		}
		else {
			right_motor_set_speed(rolling_context.speed - speed_corr);
			left_motor_set_speed(rolling_context.speed + speed_corr);
		}
	}


void avoid_obs(void){
	if(back_to_track()){
		motor_set_position(MAX_LINE_WIDTH, MAX_LINE_WIDTH, SEARCH_SPEED, SEARCH_SPEED);
		rolling_context.mode= ROTATE_TILL_COLOR;
	}
	else{
		int16_t speed_correction=0;
		if (obstacle_context.obstacle_at_left){
			speed_correction = pid_regulator_ir(obstacle_context.ir2_adjusted);
			uint32_t ir3_new = get_calibrated_prox(SENSOR_IR3);
			if (ir3_new < obstacle_context.ir3_adjusted - IR_BRUIT_BLANC){
				left_motor_set_speed(-cms_to_steps(SEARCH_SPEED) + speed_correction);
				right_motor_set_speed(-cms_to_steps(SEARCH_SPEED) - speed_correction);
			}
			else if (ir3_new > obstacle_context.ir3_adjusted + IR_BRUIT_BLANC){
				left_motor_set_speed(-cms_to_steps(SEARCH_SPEED)- speed_correction);
				right_motor_set_speed(-cms_to_steps(SEARCH_SPEED)+ speed_correction);
			}
			else {
				left_motor_set_speed(-cms_to_steps(SEARCH_SPEED));
				right_motor_set_speed(-cms_to_steps(SEARCH_SPEED));
			}

		}
		else{
			speed_correction = pid_regulator_ir(obstacle_context.ir5_adjusted);
			uint32_t ir4_new = get_calibrated_prox(SENSOR_IR4);
			//				chprintf((BaseSequentialStream *)&SD3, " ir4new =%-7d ir4old =%-7d speedcorr =%-7d\r\n\n", ir4_new, obstacle_context.ir4_adjusted, speed_correction);
			if (ir4_new < obstacle_context.ir4_adjusted - IR_BRUIT_BLANC){
				left_motor_set_speed(-cms_to_steps(SEARCH_SPEED) - speed_correction);
				right_motor_set_speed(-cms_to_steps(SEARCH_SPEED) + speed_correction);
			}
			else if (ir4_new > obstacle_context.ir4_adjusted + IR_BRUIT_BLANC){
				left_motor_set_speed(-cms_to_steps(SEARCH_SPEED)+ speed_correction);
				right_motor_set_speed(-cms_to_steps(SEARCH_SPEED)- speed_correction);
			}
			else {
				left_motor_set_speed(-cms_to_steps(SEARCH_SPEED));
				right_motor_set_speed(-cms_to_steps(SEARCH_SPEED));
			}

		}
	}

}

void reset_obstacle_context(void){
	obstacle_context.first_line_passed=0;
	obstacle_context.ir3_adjusted=0;
	obstacle_context.ir2_adjusted=0;
	obstacle_context.obstacle_at_left=0;
	obstacle_context.ir4_adjusted=0;
	obstacle_context.ir5_adjusted=0;
}

void adjust (void){
	if (obstacle_context.obstacle_at_left){
		obstacle_context.ir2_adjusted = rotate_until_irmax_left();
		obstacle_context.ir3_adjusted = get_calibrated_prox(SENSOR_IR3);
	}
	else {
		obstacle_context.ir5_adjusted = rotate_until_irmax_right();
		obstacle_context.ir4_adjusted = get_calibrated_prox(SENSOR_IR4);
	}
}


bool back_to_track(void){
	if (get_color() != NO_COLOR){
		if(!obstacle_context.first_line_passed){
			obstacle_context.first_line_passed = 1;
			motor_set_position(MAX_LINE_WIDTH, MAX_LINE_WIDTH, -SEARCH_SPEED, -SEARCH_SPEED);
			return 0;
		}
		else{
				return 1;
		}
	}
	return 0;

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
//		reset_middle_positions();
//		rolling_context.counter = 0;
//		rolling_context.speed = 0;
//		left_motor_set_speed(0);
//		right_motor_set_speed(0);
		rolling_context.speed = cms_to_steps(SEARCH_SPEED);
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

	right_motor_set_speed(-rolling_context.speed);
	left_motor_set_speed(-rolling_context.speed);
	rolling_context.counter = right_motor_get_pos();

	if (get_color() != NO_COLOR){
		rolling_context.mode = STRAIGHT_LINE_BACKWARDS;
	}
	else {
		if (abs(rolling_context.counter) > 1000){
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
		set_body_led(1);

}

//position in cm and speed en cm/s
//int : -2^32/2 to 2^32/2-1
//motor set position -2^31/2 to 2^31/2-1
void motor_set_position(float position_r, float position_l, int16_t speed_r, int16_t speed_l){

	rolling_context.position_reached = NOT_REACHED;
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	int position_to_reach_left = left_motor_get_pos()+ cm_to_step(position_l);
	int position_to_reach_right = right_motor_get_pos() - cm_to_step(position_r);

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

/////////////////////////////////////////////
void rotate_till_color(bool left_obs){
	set_leds(NO_LINE);
	// TURN WHILE YOU DONT SEE LINE AND WHILE THE LINE IS NOT IN MIDDLE
	if (get_color() == NO_COLOR){
		if (left_obs){
			left_motor_set_speed(cms_to_steps(-1));
			right_motor_set_speed(cms_to_steps(1));
//					chprintf((BaseSequentialStream *)&SD3, "TOP =%-7d BOT =%-7d DIFF =%-7d COLOR =%-7d \r\n\n",
//									get_middle_top(), get_middle_bot(), get_middle_diff(),get_color());
		}
		else { //ir4
			left_motor_set_speed(cms_to_steps(1));
			right_motor_set_speed(cms_to_steps(-1));
		}
	}
	//encore un pas stp et pid
	else {
		if (left_obs){
					left_motor_set_speed(cms_to_steps(-1));
					right_motor_set_speed(cms_to_steps(1));
				}
				else {
					left_motor_set_speed(cms_to_steps(1));
					right_motor_set_speed(cms_to_steps(-1));
				}
		rolling_context.mode = PID_FRONTWARDS;
		reset_obstacle_context();
	}


}

//replace 1 SEARCH_SPEED

int rotate_until_irmax_left(void)
{
	int	ir_left_ancien =0;
	int	ir_left_nouvau =0;
	int start = 0;
	while ((ir_left_nouvau > ir_left_ancien + 5 ) || start==0){
			start =1;
			ir_left_ancien = get_calibrated_prox(SENSOR_IR2);
			motor_set_position(PERIMETER_EPUCK/16, PERIMETER_EPUCK/16,  -1, 1);
			ir_left_nouvau = get_calibrated_prox(SENSOR_IR2);

		}
	motor_set_position(PERIMETER_EPUCK/16, PERIMETER_EPUCK/16,  1, -1);

return ir_left_ancien;
}

int rotate_until_irmax_right(void)
{
	int	ir_right_ancien =0;
	int	ir_right_nouvau =0;
	int start = 0;
	while ((ir_right_nouvau > ir_right_ancien + 5 ) || start==0){
			start =1;
			ir_right_ancien = get_calibrated_prox(SENSOR_IR5);
			motor_set_position(PERIMETER_EPUCK/16, PERIMETER_EPUCK/16,  1, -1);
			ir_right_nouvau = get_calibrated_prox(SENSOR_IR5);

		}
	motor_set_position(PERIMETER_EPUCK/16, PERIMETER_EPUCK/16,  -1, 1);

return ir_right_ancien;
}

int16_t pid_regulator_ir(int middle){

	int goal = middle;
	float error = 0;
	float speed = 0;
	float derivative = 0;

	static float sum_error = 0;
	static float last_error = 0;
	if (obstacle_context.obstacle_at_left){
		error =  get_calibrated_prox(SENSOR_IR2) - goal;
	}
	else{
		error =  get_calibrated_prox(SENSOR_IR5) - goal;
	}
	sum_error += error;

	if(sum_error > 100){
		sum_error = 100;
	}else if(sum_error < -100){
		sum_error = -100;
	}

	derivative = error - last_error ;

	last_error = error;

	speed = 0.2 * error + 0.2 * derivative; //0.01 * sum_error; //+ 0.1 * derivative;

    return (int16_t) speed;
}


//PID Implementation
int16_t pid_regulator_line(int16_t middle_diff){

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
