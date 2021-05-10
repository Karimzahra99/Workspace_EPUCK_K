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
#define ECART_ROUE 5.8

int16_t cms_to_steps (int16_t speed_cms);
float cm_to_step (float cm);
void motor_set_position(float position_r, float position_l, int16_t speed_r, int16_t speed_l);

static int ir3_old = 0;
static int ir2_new = 0;
static int ir3_new = 0;
static int ir_left_max=0;

static int ir4_old = 0;
static int ir4_new = 0;
static int ir5_new = 0;
static uint8_t  adjust = 0;
static uint8_t obstacele_at_left =0;
static uint8_t obstacele_at_right =0;
static int ir_right_max=0;
static uint8_t POSITION_REACHED = 0;
static uint8_t obstacle_mode = 0;

//simple PI regulator implementation
//int16_t pid_regulator(float middle){
//
//	float goal = 320; //milieu theorique d'une ligne parfaitement centre sur le robot ou 350 ?
//	float error = 0;
//	float speed = 0;
//	float derivative = 0;
//
//	static float sum_error = 0;
//	static float last_error = 0;
//
//	error = middle - goal;
//
//	//get(IR3)
//	//get(IR1)
//
//	//disables the PID regulator if the error is to small
//	//this avoids to always move as we cannot exactly be where we want and
//	//the camera is a bit noisy
////	if(fabs(error) < ERROR_THRESHOLD){ //ERROR_THRESHOLD = 0.1cm definit en float
////		return 0;
////	}
//
//	sum_error += error;//sum_error = sum_error + error;
//
//	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
//	// Integral anti-windup (Anti-Reset Windup)
//	//https://www.youtube.com/watch?v=NVLXCwc8HzM&list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y&index=2
//	if(sum_error > MAX_SUM_ERROR){
//		sum_error = MAX_SUM_ERROR;
//	}else if(sum_error < -MAX_SUM_ERROR){
//		sum_error = -MAX_SUM_ERROR;
//	}
//
//	derivative = error - last_error;
//
//	last_error = error;
//
//	speed = 2 * error + 0.01 * sum_error; //+ KD * derivative;
//
//    return (int16_t)speed;
//}
//

int16_t pid_regulator_S(int middle){

	int goal = middle; //milieu theorique d'une ligne parfaitement centre sur le robot ou 350 ?
	float error = 0;
	float speed = 0;
	float derivative = 0;

	static float sum_error = 0;
	static float last_error = 0;
	ir2_new = get_calibrated_prox(Sensor_IR2);

	error =  ir2_new - goal;

	//get(IR3)
	//get(IR1)

	//disables the PID regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
//	if(fabs(error) < ERROR_THRESHOLD){ //ERROR_THRESHOLD = 0.1cm definit en float
//		return 0;
//	}

	sum_error += error;//sum_error = sum_error + error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	// Integral anti-windup (Anti-Reset Windup)
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

static THD_WORKING_AREA(waPidRegulator, 256);
static THD_FUNCTION(PidRegulator, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

//	int16_t speed = 0;
	int16_t speed_correction = 0;


	while(1){

		time = chVTGetSystemTime();
		int ir_front_left = get_prox(Sensor_IR3);
		int ir_front_right = get_prox(Sensor_IR4);


		if (((ir_front_left < IR_THRESHOLD) && (ir_front_right < IR_THRESHOLD) && !obstacle_mode)){
			left_motor_set_speed(-cms_to_steps(2));
			right_motor_set_speed(-cms_to_steps(2));
		}
		else {
			obstacle_mode = 1;
			if (ir_front_left > IR_THRESHOLD || obstacele_at_left ){
				obstacele_at_left =1;
				if (!adjust){
					adjust = 1;
					ir_left_max = rotate_until_irmax_left();
					ir3_old = get_calibrated_prox(Sensor_IR3);
				}
				else{
					speed_correction = pid_regulator_S(ir_left_max);
					ir3_new = get_calibrated_prox(Sensor_IR3);
//					chprintf((BaseSequentialStream *)&SD3, " ir3new =%-7d ir3old =%-7d speedcorr =%-7d\r\n\n", ir3_new, ir3_old, speed_correction);
					if (ir3_new < ir3_old - 10){
						left_motor_set_speed(-cms_to_steps(2) + speed_correction);
						right_motor_set_speed(-cms_to_steps(2) - speed_correction);
					}
					else if ( ir3_new > ir3_old + 10  ){
						left_motor_set_speed(-cms_to_steps(2)- speed_correction);
						right_motor_set_speed(-cms_to_steps(2)+ speed_correction);
					}
					else {
						left_motor_set_speed(-cms_to_steps(2));
						right_motor_set_speed(-cms_to_steps(2));
					}

				}
			}
			else{
				if (!adjust){
					adjust = 1;
					ir_right_max = rotate_until_irmax_right();
					ir4_old = get_calibrated_prox(Sensor_IR4);
				}
				else{
					speed_correction = pid_regulator_S(ir_right_max);
					ir4_new = get_calibrated_prox(Sensor_IR4);
//					chprintf((BaseSequentialStream *)&SD3, " ir4new =%-7d ir4old =%-7d speedcorr =%-7d\r\n\n", ir4_new, ir4_old, speed_correction);
					if (ir4_new < ir4_old - 20){
						left_motor_set_speed(-cms_to_steps(2) - speed_correction);
						right_motor_set_speed(-cms_to_steps(2) + speed_correction);
					}
					else if ( ir4_new > ir4_old + 10  ){
						left_motor_set_speed(-cms_to_steps(2)+ speed_correction);
						right_motor_set_speed(-cms_to_steps(2)- speed_correction);
					}
					else {
						left_motor_set_speed(-cms_to_steps(2));
						right_motor_set_speed(-cms_to_steps(2));
					}

				}
			}
		}

	}
	chThdSleepUntilWindowed(time, time + MS2ST(10));
}



//    	int ir_front_left = get_prox(Sensor_IR3);
//    	int ir_front_right = get_prox(Sensor_IR4);

//    	if (((ir_front_left < IR_THRESHOLD) && (ir_front_right < IR_THRESHOLD))){

    	//computes the speed to give to the motors
    	//distance_cm is modified by the image processing thread
//    	switch (get_color())
//    	{
//    	case 0:
//    		speed = cms_to_steps(0);
//    		break;
//    	case 1: //RED
//    		speed = cms_to_steps(1);
//    		break;
//    	case 2: //GREEN
//    		speed = cms_to_steps(3);
//    		break;
//    	case 3: //BLUE
//    		speed = cms_to_steps(5);
//    		break;
//    	default:
//    		speed = cms_to_steps(2);
//    		break;
//    	}
//
//    	speed_correction = pid_regulator(get_middle_line());
//
////    	// if the line is nearly in front of the camera, don't rotate
////    	if(abs(speed_correction) < 5){
////    		speed_correction = 0;
////    	}
//
//    	//applies the speed from the PI regulator and the correction for the rotation
//    	//?
//    	left_motor_set_speed(-speed + speed_correction);
//    	right_motor_set_speed(-speed - speed_correction);
////    }

    //Obstacle Avoidance
//    else {
//    	//Simple PseudoCode to avoid cylindrical shapes with known radius
//    	//Move backwards 5cm at speed 7cm/s
//    	motor_set_position (4, 4, speedcms_to_speedsteps(1), speedcms_to_speedsteps(1));
//    	if (ir_front_left > ir_front_right){
//    		//Rotate CW 90deg
//    		motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4,  -speedcms_to_speedsteps(3), speedcms_to_speedsteps(3));
//
//    		//Half Circle Trajectory to avoid obstacle
//    		//Half Circle Trajectory to avoid obstacle = de gauche
//    		mov_circ_right(speedcms_to_speedsteps(4), 12 ,PI, 1);
//    		//Rotate CW 90deg
//    		motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4,  -speedcms_to_speedsteps(3), speedcms_to_speedsteps(3));
//
//    	}
//    	else {
//    		//Rotate CCW 90deg
//    		motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4,  speedcms_to_speedsteps(3), - speedcms_to_speedsteps(3));
//
//    		//Half Circle Trajectory to avoid obstacle
//    		mov_circ_left(speedcms_to_speedsteps(4), 12 ,PI, 1);
//
//    		//Rotate CCW 90deg
//    		motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4,  speedcms_to_speedsteps(3), - speedcms_to_speedsteps(3));
//    		//while(motor_position_reached() != POSITION_REACHED);
//
//    	}

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

//    	}
    	//100Hz
//    	chThdSleepUntilWindowed(time, time + MS2ST(10));
//    }
//}

void pid_regulator_start(void){
	chThdCreateStatic(waPidRegulator, sizeof(waPidRegulator), NORMALPRIO, PidRegulator, NULL);
}

int16_t cms_to_steps (int16_t speed_cms) {
	return speed_cms * NSTEP_ONE_TURN / WHEEL_PERIMETER;
}

float cm_to_step (float cm) {
	return cm * NSTEP_ONE_TURN / WHEEL_PERIMETER;
}


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
// Ancien code utilise pour le TP2, faire un truc semblable pour eviter des cylindres
//vitesse en cm/s //rayon en cm //angle en radian //mode = 0 cercle par top //mode = 1 cercle par bot
void mov_circ_right(float vitesse,float rayon,float angle, int mode){
	float dg = (rayon+ECART_ROUE/2)*angle;
	float dd = (rayon-ECART_ROUE/2)*angle;
	float vg = vitesse;
	float vd = dd*vg/dg;
	if (mode == 1){
		vd = -vd;
		vg = -vg;
	}
	motor_set_position(dd,dg,vd,vg);
}

void mov_circ_left(float vitesse,float rayon,float angle, int mode){
	float dg = (rayon-ECART_ROUE/2)*angle;
	float dd = (rayon+ECART_ROUE/2)*angle;
	float vd = vitesse;
	float vg = dg*vd/dd;
	if (mode == 1){
		vd = -vd;
		vg = -vg;
	}
	motor_set_position(dd,dg,vd,vg);
}

int rotate_until_irmax_left(void)
{
	int	ir_left_ancien =0;
	int	ir_left_nouvau =0;
	int start = 0;
	while ((ir_left_nouvau > ir_left_ancien + 5 ) || start==0){
			start =1;
			ir_left_ancien = get_calibrated_prox(Sensor_IR2);
			motor_set_position(PERIMETER_EPUCK/16, PERIMETER_EPUCK/16,  -1, 1);
			ir_left_nouvau = get_calibrated_prox(Sensor_IR2);

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
			ir_right_ancien = get_calibrated_prox(Sensor_IR5);
//			ir_avant_ancien = get_prox(Sensor_IR3);
			motor_set_position(PERIMETER_EPUCK/16, PERIMETER_EPUCK/16,  1, -1);
			ir_right_nouvau = get_calibrated_prox(Sensor_IR5);
//			ir_avant_nouvau = get_prox(Sensor_IR3);

		}
	motor_set_position(PERIMETER_EPUCK/16, PERIMETER_EPUCK/16,  -1, 1);

return ir_right_ancien;
}
