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
#include <process_image.h>

static uint8_t rolling_mode = 0;//0 = rolling backwards in strait line, 1 = rolling frontwards for turns, 2 =
static uint8_t POSITION_REACHED = 0;

void set_leds(uint8_t color_index);
float speedcms_to_speedsteps (uint8_t speed_cms);

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

//        chprintf((BaseSequentialStream *)&SD3, "IR4 =%-7d IR3 =%-7d \r\n\n",
//        ir_front_right, ir_front_left);



        //if (((ir_front_left < IR_THRESHOLD) && (ir_front_right < IR_THRESHOLD)) && !obstacle_mode){
        	if ((ir_front_left < IR_THRESHOLD) && (ir_front_right < IR_THRESHOLD)){
        		//computes the speed to give to the motors
        		//distance_cm is modified by the image processing thread
        		uint8_t index_color = get_color();

        		chprintf((BaseSequentialStream *)&SD3, "Dif =%-7d \r\n\n",
        				get_middle_diff());

        		chprintf((BaseSequentialStream *)&SD3, "Top =%-7d Bot =%-7d \r\n\n",
        				get_middle_top(),get_middle_bot());


        		switch (index_color)
        		{
        		case 0: //NO COLOR
        			set_leds(index_color);
        			speed = 0;
        			break;
        		case 1: //RED
        			set_leds(index_color);
        			speed = speedcms_to_speedsteps(2);
        			break;
        		case 2: //GREEN
        			set_leds(index_color);
        			speed = speedcms_to_speedsteps(4);
        			break;
        		case 3: //BLUE
        			set_leds(index_color);
        			speed = speedcms_to_speedsteps(6);
        			break;
        		default:
        			speed = speedcms_to_speedsteps(1.3);
        			break;
        		}

        		int16_t middle_diff = get_middle_diff();



        		//rolling backwards in strait line
        		if ((middle_diff < DEAD_ZONE_WIDTH) && (rolling_mode == 0)){
        			rolling_mode = 0;
        			right_motor_set_speed(-speed);
        			left_motor_set_speed(-speed);
        		}
        		//turning mode
        		else {
        			rolling_mode = 1;

        			int16_t speed_correction = pid_regulator(middle_diff);
        		}


        	}
        	else {
        		set_leds(YELLOW_IDX);
        		speed = speedcms_to_speedsteps(0);
        		right_motor_set_speed(speed);
        		left_motor_set_speed(speed);
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

float cm_to_steps (uint8_t cm) {
	return cm * NSTEP_ONE_TURN / WHEEL_PERIMETER;
}

void set_leds(uint8_t color_index){
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

//position en cm et vitesse en cm/s
void motor_set_position(int16_t position_r, int16_t position_l, int16_t speed_r, int16_t speed_l){

	POSITION_REACHED = 0;
	left_motor_set_pos(0);
	right_motor_set_pos(0);

	int16_t position_to_reach_left = cm_to_stepsposition_l(position_l);
	int16_t position_to_reach_right = - cm_to_stepsposition_l(position_r);

	while (!POSITION_REACHED){
		left_motor_set_speed(cms_to_steps(speed_l));
		right_motor_set_speed(cms_to_steps(speed_r));

		chprintf((BaseSequentialStream *)&SD3, "R =%-7d L =%-7d \r\n\n",
				right_motor_get_pos(), left_motor_get_pos());

		if (abs(right_motor_get_pos()) > abs(position_to_reach_right) && abs(left_motor_get_pos()) > abs(position_to_reach_left) ){
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			POSITION_REACHED=1;
		}
	}
}
