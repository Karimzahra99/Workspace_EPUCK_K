////static THD_WORKING_AREA(waPidRegulator, 256);
////static THD_FUNCTION(PidRegulator, arg) {
////    chRegSetThreadName(__FUNCTION__);
////    (void)arg;
////    systime_t time;
////    int16_t speed = 0;
////    int16_t speed_correction = 0;
////    while(1){
////        time = chVTGetSystemTime();
////        int ir_front_left = get_prox(Sensor_IR3);
////        int ir_front_right = get_prox(Sensor_IR4);
//////        chprintf((BaseSequentialStream *)&SD3, "IR4 =%-7d IR3 =%-7d \r\n\n",
//////        ir_front_right, ir_front_left);
////        //if (((ir_front_left < IR_THRESHOLD) && (ir_front_right < IR_THRESHOLD)) && !obstacle_mode){
////        	if ((ir_front_left < IR_THRESHOLD) && (ir_front_right < IR_THRESHOLD)){
////        		//computes the speed to give to the motors
////        		//distance_cm is modified by the image processing thread
////        		uint8_t index_color = get_color();
////        		chprintf((BaseSequentialStream *)&SD3, "Dif =%-7d \r\n\n",
////        				get_middle_diff());
////        		chprintf((BaseSequentialStream *)&SD3, "Top =%-7d Bot =%-7d \r\n\n",
////        				get_middle_top(),get_middle_bot());
////        		switch (index_color)
////        		{
////        		case 0: //NO COLOR
////        			set_leds(index_color);
////        			speed = 0;
////        			break;
////        		case 1: //RED
////        			set_leds(index_color);
////        			speed = cms_to_steps(2);
////        			break;
////        		case 2: //GREEN
////        			set_leds(index_color);
////        			speed = cms_to_steps(4);
////        			break;
////        		case 3: //BLUE
////        			set_leds(index_color);
////        			speed = cms_to_steps(6);
////        			break;
////        		default:
////        			speed = cms_to_steps(1.3);
////        			break;
////        		}
////        		int16_t middle_diff = get_middle_diff();
////        		//rolling backwards in strait line
////        		if ((abs(middle_diff) < DEAD_ZONE_WIDTH) && (rolling_mode == 0)){
////        			rolling_mode = 0;
////        			right_motor_set_speed(-speed);
////        			left_motor_set_speed(-speed);
////        		}
////        		//turning mode
////        		else {
////        			rolling_mode = 1;
////        			int16_t speed_correction = pid_regulator(middle_diff);
////        		}
////        	}
////        	else {
////        		set_leds(YELLOW_IDX);
////        		speed = cms_to_steps(0);
////        		right_motor_set_speed(speed);
////        		left_motor_set_speed(speed);
////        	}
////        //100Hz
////        chThdSleepUntilWindowed(time, time + MS2ST(10));
////    }
////}
////
////
////
////
//static THD_WORKING_AREA(waPidRegulator, 512);
//static THD_FUNCTION(PidRegulator, arg) {
//
//	chRegSetThreadName(__FUNCTION__);
//	(void)arg;
//
//	systime_t time;
//	//maybe put in static
//	int16_t speed = 0;
//	int16_t speed_correction = 0;
//	//maybe put in static
//	int16_t middle_diff = 0;
//
//	uint8_t start_count = 0;
//
//	while(1){
//		time = chVTGetSystemTime();
//
//		int ir_front_left = get_prox(Sensor_IR3);
//		int ir_front_right = get_prox(Sensor_IR4);
//
//		chprintf((BaseSequentialStream *)&SD3, "IRL =%-7d IRR =%-7d \r\n\n",
//				get_prox(Sensor_IR3), get_prox(Sensor_IR4));
//
//		chprintf((BaseSequentialStream *)&SD3, "rolling_mode =%-7d  =%-7d \r\n\n",
//						rolling_mode, get_color());
//
//
//		//Neglecting obstacles in rolling_mode 1
//		if (((ir_front_left > IR_THRESHOLD) && (ir_front_right > IR_THRESHOLD)) && rolling_mode == 0){
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
//
//		if (rolling_mode == 1){
//			// CCW 180deg
//			middle_diff = get_middle_diff();
//			if (start_move == 0){
//				motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, speed, -speed);
//				start_move = 1;
//			}
//			if (middle_diff > DEAD_ZONE_WIDTH ){
//				speed_correction = pid_regulator(middle_diff);
//				right_motor_set_speed(speed);
//				left_motor_set_speed(speed + speed_correction);
//				right_motor_set_speed(speed - speed_correction);
//				// maybe remove
//				start_count = 0;
//			}
//			else {
//				if (start_count == 0){
//					left_motor_set_pos(0);
//					right_motor_set_pos(0);
//					left_motor_set_speed(cms_to_steps(speed));
//					right_motor_set_speed(cms_to_steps(speed));
//					start_count = 1;
//				}
//				else {
//					if ((left_motor_get_pos() >= 1000) && (right_motor_get_pos() >= 1000)){
//						motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, speed, -speed);
//						start_count = 0;
//						start_move = 0;
//						rolling_mode = 0;
//
//					}
//				}
//
//			}
//		}
//
//		// to complete
//		if (rolling_mode == 2){
//			set_leds(YELLOW_IDX);
//			speed = cms_to_steps(0);
//			right_motor_set_speed(speed);
//			left_motor_set_speed(speed);
//		}
//
//		//100Hz
//		chThdSleepUntilWindowed(time, time + MS2ST(10));
//	}
//}
