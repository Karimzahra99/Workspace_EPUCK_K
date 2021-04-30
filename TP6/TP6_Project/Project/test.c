//uint16_t calc_middle(uint8_t *buffer){
//
//	int index_start = 0;
//	int index_end = 0;
//	int init_start = 0;
//
//
//	int line_streak = 0;
//
//	for (int i = 0; i < IMAGE_BUFFER_SIZE; ++i){
//
//		if ((init_start == 0)&&(buffer[i] > 0)){
//			index_start = i;
//			init_start = 1;
//			line_streak = 1;
//		}
//
//		if ((init_start == 1)&&(buffer[i] > 0)){
//			line_streak ++;
//		}
//
//		if ((init_start == 1)&&(buffer[i] == 0)){
//			int null_streak = 1;
//			for (int j = i+1; j < IMAGE_BUFFER_SIZE-i; j++){
//				if (buffer[j] == 0 ){
//					null_streak ++;
//				}
//
//				else break;
//
//				if (null_streak > MIN_WIDTH){
//					if (j-index_start > 50){
//						index_end = j;
//					}
//					else {
//						init_start = 0;
//						index_start = -1;
//						i = j+1;
//						break;
//					}
//				}
//			}
//
//		}
//	}
//
//
//}
//
//
//uint16_t calc_middle(uint8_t *buffer){
//
//	int max_hole_width = 15;
//	int min_line_width = 50;
//
//	int start_p = 0;
//	int end_p = 0;
//	int start_n = 0;
//	int end_n = 0;
//
//	int begin_line = 0;
//	int hole_size = 0;
//
//	for (int i = 0; i < IMAGE_BUFFER_SIZE; ++i){
//
//		if ((begin_line == 0)&&(buffer[i]>0)){
//			begin_line = 1;
//			start_n = i;
//			line_size = 1;
//			continue;
//		}
//
//		else {
//			if ((begin_line == 1)&&(buffer[i] == 0)){
//
//				hole_size = 1;
//				end_n = i;
//
//				for (int j = i+1; j < IMAGE_BUFFER_SIZE; ++j){
//
//					if (buffer[j] == 0){
//
//						hole_size ++;
//
//						if (hole_size > max_hole_width){
//
//							begin_line = 0;
//
//							if ((end_n - start_n > min_line_width) && (end_n - start_n > end_p - start_p)){
//								start_p = start_n;
//								end_p = end_n;
//							}
//							i = j;
//						}
//					}
//				}
//			}
//		}
//	}
//
//	if (((i = IMAGE_BUFFER_SIZE -1) && (buffer[i] > 0)) && (begin_line = 1)){
//
//	}
//}
//
//
//
//
//
//uint16_t calc_middle(uint8_t *buffer){
//
//	uint16_t begin = 0;
//	uint16_t end = 0;
//	uint8_t start = 0;
//	uint8_t no_begin = 0;
//	uint8_t no_end = 0;
//
//	//Check partial line cases :
//	//Line has already started
//	if (buffer[0] != 0){
//		no_begin = 1;
//	}
//	//Line has no end
//	if (buffer[IMAGE_BUFFER_SIZE] != 0){
//		no_end = 1;
//		end = IMAGE_BUFFER_SIZE; //begin + LINEWIDTH dans la for loop ?
//	}
//
//	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++){
//
//		if (!no_begin){
//			if ((buffer[i] > 0) && (start == 0)){
//				begin = i;
//				start = 1;
//				//If the line dosen't end => quit the loop
//				if (no_end) break;
//			}
//			// Searching for end point
//			if ((buffer[i] == 0) && (start == 1)){
//				end= i-1;
//				break;
//			}
//		}
//		//No begin point case :
//		else {
//			// Searching for end point
//			if (buffer[i] == 0){
//				end = i-1;
//				break;
//			}
//		}
//	}
//	//Calculated middle of line
//	return (begin + end)/2;
//
////	chprintf((BaseSequentialStream *)&SD3, "%Middle =%-7d \r\n\n", middle_line);
//
//}
