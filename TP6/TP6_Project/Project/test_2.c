//#include "ch.h"
//#include "hal.h"
//#include <math.h>
//#include <usbcfg.h>
//#include <chprintf.h>
//#include <sensors/proximity.h>
//#include <leds.h>
//#include <main.h>
//#include <motors.h>
//#include <pid_regulator.h>
//#include <process_image.h>
//
//static uint8_t rolling_mode = 0;//0 = rolling backwards in strait line, 1 = rolling frontwards for turns, 2 = obstacle mode
//// remove caps its not a define
//static uint8_t POSITION_REACHED = 0;
//static uint8_t start_move = 0;
//
//void set_leds(uint8_t color_index);
//
//int16_t cms_to_steps (int16_t speed_cms);
//float cm_to_step (float cm);
//
//void motor_set_position(float position_r, float position_l, int16_t speed_r, int16_t speed_l);
//
//////simple PI regulator implementation
//int16_t pid_regulator(int16_t middle_diff){
//
//	float speed_correction = 0;
//
//	float error = (float)middle_diff;
//
//	float derivative = 0;
//
//	static float sum_error = 0;
//	static float last_error = 0;
//
//	sum_error += error;//sum_error = sum_error + error;
//
//	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
//	// Integral anti-windup (Anti-Reset Windup)
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
//	speed_correction = KP * error + KI * sum_error + KD*derivative;
//
//
//
//	return (int16_t)speed_correction;
//}
//
//
//static THD_WORKING_AREA(waPidRegulator, 512);
//static THD_FUNCTION(PidRegulator, arg) {
//    chRegSetThreadName(__FUNCTION__);
//    (void)arg;
//
//    //waits 2s
//    chThdSleepMilliseconds(2000);
//
//    rolling_mode = 0;
//    start_move = 0;
//
//    systime_t time;
//    //maybe put in static
//    int16_t speed = 0;
//    int16_t speed_correction = 0;
//    //maybe put in static
//    int16_t middle_diff = 0;
//
//    uint8_t start_count = 0;
//
//    while(1){
//    	time = chVTGetSystemTime();
//
//    	int ir_front_left = get_prox(Sensor_IR3);
//    	int ir_front_right = get_prox(Sensor_IR4);
//
//    	chprintf((BaseSequentialStream *)&SD3, "IRL =%-7d IRR =%-7d \r\n\n",
//    			get_prox(Sensor_IR3), get_prox(Sensor_IR4));
//
//    	chprintf((BaseSequentialStream *)&SD3, "MidDif =%-7d Rolling Mode =%-7d \r\n\n",
//    			get_middle_diff(), rolling_mode);
//
//    	chprintf((BaseSequentialStream *)&SD3, "MidTop =%-7d MidBot =%-7d \r\n\n",
//    			get_middle_top(),get_middle_top());
//
//    	// Removed rolling_mode == 0 as condition
//		if ((ir_front_left > IR_THRESHOLD) && (ir_front_right > IR_THRESHOLD)) {
//			rolling_mode = 2;
//		}
//
//		if (rolling_mode == 0) {
//			middle_diff = get_middle_diff();
//			if (abs(middle_diff) > DEAD_ZONE_WIDTH){
//				rolling_mode = 1;
//			}
//		}
//
//		if (rolling_mode == 0){
//
//			uint8_t index_color = get_color();
//			switch (index_color)
//			{
//			case 0: //NO COLOR
//				set_leds(index_color);
//				speed = 0;
//				break;
//			case 1: //RED
//				set_leds(index_color);
//				speed = cms_to_steps(2);
//				break;
//			case 2: //GREEN
//				set_leds(index_color);
//				speed = cms_to_steps(4);
//				break;
//			case 3: //BLUE
//				set_leds(index_color);
//				speed = cms_to_steps(6);
//				break;
//			default:
//				speed = cms_to_steps(1.3);
//				break;
//			}
//
//			right_motor_set_speed(-speed);
//			left_motor_set_speed(-speed);
//
//		}
//		else {
//			if (rolling_mode == 1){
//
//				set_leds(PURPLE_IDX);
//
//				//if robot dosen't start on a line
//				if (speed == 0){
//					speed = 2;
//				}
//
//				// CCW 180deg
//				middle_diff = get_middle_diff();
//
//				if (start_move == 0){
//					motor_set_position(PERIMETER_EPUCK/2.0, PERIMETER_EPUCK/2, speed, -speed);
//					motor_set_position(15, 15, -speed, -speed);
//					right_motor_set_speed(0);
//					left_motor_set_speed(0);
//					start_move = 1;
//				}
//
//				//			if (middle_diff > DEAD_ZONE_WIDTH ){
//				//				speed_correction = pid_regulator(middle_diff);
//				//				right_motor_set_speed(speed);
//				//				left_motor_set_speed(speed_correction);
//				//				right_motor_set_speed(- speed_correction);
//				//				// maybe remove
//				//				start_count = 0;
//				//			}
//				//			else {
//				//				if (start_count == 0){
//				//					left_motor_set_pos(0);
//				//					right_motor_set_pos(0);
//				//					left_motor_set_speed(cms_to_steps(speed));
//				//					right_motor_set_speed(cms_to_steps(speed));
//				//					start_count = 1;
//				//				}
//				//				else {
//				//					if ((left_motor_get_pos() >= 1000) && (right_motor_get_pos() >= 1000)){
//				//						motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, speed, -speed);
//				//						start_count = 0;
//				//						start_move = 0;
//				//						rolling_mode = 0;
//				//
//				//					}
//				//				}
//				//
//				//			}
//			}else {
//
//				if (rolling_mode == 2){
//					set_leds(YELLOW_IDX);
//					right_motor_set_speed(cms_to_steps(0));
//					left_motor_set_speed(cms_to_steps(0));
//
//				}
//			}
//		}
//        //100Hz
//        chThdSleepUntilWindowed(time, time + MS2ST(10));
//    }
//}
//
//void pid_regulator_start(void){
//	chThdCreateStatic(waPidRegulator, sizeof(waPidRegulator), NORMALPRIO, PidRegulator, NULL);
//}
//
//int16_t cms_to_steps (int16_t speed_cms) {
//	return speed_cms * NSTEP_ONE_TURN / WHEEL_PERIMETER;
//}
//
//float cm_to_step (float cm) {
//	return cm * NSTEP_ONE_TURN / WHEEL_PERIMETER;
//}
//
//void set_leds(uint8_t color_index){
//	if (color_index == RED_IDX){
//		set_rgb_led(LED_RGB_2, LED_ON, LED_OFF, LED_OFF);
//		set_rgb_led(LED_RGB_4, LED_ON, LED_OFF, LED_OFF);
//		set_rgb_led(LED_RGB_6, LED_ON, LED_OFF, LED_OFF);
//		set_rgb_led(LED_RGB_8, LED_ON, LED_OFF, LED_OFF);
//	}
//	else {
//		if (color_index == GREEN_IDX){
//			set_rgb_led(LED_RGB_2, LED_OFF, LED_ON, LED_OFF);
//			set_rgb_led(LED_RGB_4, LED_OFF, LED_ON, LED_OFF);
//			set_rgb_led(LED_RGB_6, LED_OFF, LED_ON, LED_OFF);
//			set_rgb_led(LED_RGB_8, LED_OFF, LED_ON, LED_OFF);
//		}
//		else {
//			if (color_index == BLUE_IDX){
//				set_rgb_led(LED_RGB_2, LED_OFF, LED_OFF, LED_ON);
//				set_rgb_led(LED_RGB_4, LED_OFF, LED_OFF, LED_ON);
//				set_rgb_led(LED_RGB_6, LED_OFF, LED_OFF, LED_ON);
//				set_rgb_led(LED_RGB_8, LED_OFF, LED_OFF, LED_ON);
//			}
//			else {
//				if (color_index == YELLOW_IDX){
//					set_rgb_led(LED_RGB_2, LED_ON, LED_ON, LED_OFF);
//					set_rgb_led(LED_RGB_4, LED_ON, LED_ON, LED_OFF);
//					set_rgb_led(LED_RGB_6, LED_ON, LED_ON, LED_OFF);
//					set_rgb_led(LED_RGB_8, LED_ON, LED_ON, LED_OFF);
//				}
//				else {
//					if (color_index == PURPLE_IDX){
//						set_rgb_led(LED_RGB_2, LED_ON, LED_OFF, LED_ON);
//						set_rgb_led(LED_RGB_4, LED_ON, LED_OFF, LED_ON);
//						set_rgb_led(LED_RGB_6, LED_ON, LED_OFF, LED_ON);
//						set_rgb_led(LED_RGB_8, LED_ON, LED_OFF, LED_ON);
//					}
//					else {
//						if (color_index == NO_COLOR){
//							set_rgb_led(LED_RGB_2, LED_OFF, LED_OFF, LED_OFF);
//							set_rgb_led(LED_RGB_4, LED_OFF, LED_OFF, LED_OFF);
//							set_rgb_led(LED_RGB_6, LED_OFF, LED_OFF, LED_OFF);
//							set_rgb_led(LED_RGB_8, LED_OFF, LED_OFF, LED_OFF);
//						}
//					}
//				}
//			}
//		}
//	}
//}
//
////position in cm and speed en cm/s
////int : -2^32/2 to 2^32/2-1
////motor set position -2^31/2 to 2^31/2-1
//void motor_set_position(float position_r, float position_l, int16_t speed_r, int16_t speed_l){
//
//	POSITION_REACHED = 0;
//	left_motor_set_pos(0);
//	right_motor_set_pos(0);
//
//	int position_to_reach_left = cm_to_step(position_l);
//	int position_to_reach_right = - cm_to_step(position_r);
//
//	while (!POSITION_REACHED){
//		left_motor_set_speed(cms_to_steps(speed_l));
//		right_motor_set_speed(cms_to_steps(speed_r));
//
////		chprintf((BaseSequentialStream *)&SD3, "R =%-7d L =%-7d \r\n\n",
////				right_motor_get_pos(), left_motor_get_pos());
//
//		if (abs(right_motor_get_pos()) > abs(position_to_reach_right) && abs(left_motor_get_pos()) > abs(position_to_reach_left) ){
//			left_motor_set_speed(0);
//			right_motor_set_speed(0);
//			POSITION_REACHED=1;
//		}
//	}
//}
