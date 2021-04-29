#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <string.h>
#include <main.h>
#include <camera/po8030.h>
#include <process_image.h>
#include <selector.h>

//Color detection settings (uncomment only one) :

//Identify color only using max values
#define USE_ONLY_MAX

//Identify color only using mean values
//#define USE_ONLY_MEAN

//Identify color using max and mean values
//#define USE_MAX_N_MEAN

//Uncomment to use plot_image.py :
//#define PLOT_ON_COMPUTER

//Unomment to send general color data (max, mean, count) to Realterm or Screen
//#define SEND_DATA

#define CONTRAST 			90 //default constrast is 64

void find_color(void);
void set_threshold_color(int selector_pos);
void calc_max_mean(void);
void max_count(void);
void calc_line_middle(uint8_t alternator);
void filter_noise(uint16_t index, uint8_t red_value, uint8_t green_value, uint8_t blue_value);

static uint16_t middle_line_top = IMAGE_BUFFER_SIZE/2; //middle of line
static uint16_t middle_line_bot = IMAGE_BUFFER_SIZE/2;

static uint8_t alternate_lines = 0; //0 = top line, 1 = bottom line

static uint8_t color_idx = 0; //0 = NO_COLOR, 1 = RED, 2 = GREEN, 3 = BLUE
static uint8_t threshold_color = 0;

static uint16_t count_red  = 0;
static uint16_t count_green  = 0;
static uint16_t count_blue = 0;

static uint8_t max_red  = 0;
static uint8_t max_green  = 0;
static uint8_t max_blue = 0;

static uint8_t mean_red  = 0;
static uint8_t mean_green  = 0;
static uint8_t mean_blue = 0;
static uint8_t image_red[IMAGE_BUFFER_SIZE] = {0};
static uint8_t image_green[IMAGE_BUFFER_SIZE] = {0};
static uint8_t image_blue[IMAGE_BUFFER_SIZE] = {0};

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);


/*
* Returns the line's width extracted from the image buffer given
* Returns 0 if line not found
*/
uint16_t calc_middle(uint8_t *buffer){

	uint16_t begin = 0;
	uint16_t end = 0;
	uint8_t start = 0;
	uint8_t no_begin = 0;
	uint8_t no_end = 0;

	//Check partial line cases :
	//Line has already started
	if (buffer[0] != 0){
		no_begin = 1;
	}
	//Line has no end
	if (buffer[IMAGE_BUFFER_SIZE] != 0){
		no_end = 1;
		end = IMAGE_BUFFER_SIZE; //begin + LINEWIDTH dans la for loop ?
	}

	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++){

		if (!no_begin){
			if ((buffer[i] > 0) && (start == 0)){
				begin = i;
				start = 1;
				//If the line dosen't end => quit the loop
				if (no_end) break;
			}
			// Searching for end point
			if ((buffer[i] == 0) && (start == 1)){
				end= i-1;
				break;
			}
		}
		//No begin point case :
		else {
			// Searching for end point
			if (buffer[i] == 0){
				end = i-1;
				break;
			}
		}
	}
	//Calculated middle of line
	return (begin + end)/2;

//	chprintf((BaseSequentialStream *)&SD3, "%Middle =%-7d \r\n\n", middle_line);

}


static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    //chThdSleepMilliseconds(12);

    (void)arg;

	while(1){

		chprintf((BaseSequentialStream *)&SD3, "In the While \r\n\n");

		//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line LINE_INDEX + LINE_INDEX+1 (minimum 2 lines because reasons)
		if (alternate_lines == TOP){
			po8030_advanced_config(FORMAT_RGB565, 0, LINE_INDEX_TOP, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
			alternate_lines = BOTTOM;
			chprintf((BaseSequentialStream *)&SD3, "TOP CASE \r\n\n");
		}
		if (alternate_lines == BOTTOM){
			po8030_advanced_config(FORMAT_RGB565, 0, LINE_INDEX_BOT, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
			alternate_lines = TOP;
			chprintf((BaseSequentialStream *)&SD3, "BOT CASE \r\n\n");
		}

		chprintf((BaseSequentialStream *)&SD3, "PASSED CASES \r\n\n");

		//Line index 413 detecting colors goes wrong
		//po8030_advanced_config(FORMAT_RGB565, 0, 413, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);

		dcmi_enable_double_buffering();
		dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
		dcmi_prepare();
		po8030_set_awb(0);
		//po8030_set_mirror(0, 1);
		po8030_set_contrast(CONTRAST);

    	//starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready(); //fait l'attente dans le while(1)
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);

    }

}

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {


	chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;

#ifdef PLOT_ON_COMPUTER
	bool send_to_computer = true; //to use plot_image.py
#endif

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		set_threshold_color(get_selector());

		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){

			//extracting red 5 bits and shifting them right
			uint8_t r = ((uint8_t)img_buff_ptr[i]&0xF8) >> SHIFT_3;

			//Extract G6G5G4G3G2
			uint8_t g = (((uint8_t)img_buff_ptr[i]&0x07) << 2) + (((uint8_t)img_buff_ptr[i+1]&0xC0) >> 6);

			//extracting blue 5 bits
			uint8_t b = (uint8_t)img_buff_ptr[i+1]&0x1F;

			filter_noise(i, r, g, b);

		}
		calc_max_mean();
		max_count();
		find_color();

		//search for a line in the image and gets its middle position
		calc_line_middle(alternate_lines);

//		int test1 = get_prox(3);
//		int test2 = get_prox(4);
//		chprintf((BaseSequentialStream *)&SD3, "IR4 =%-7d IR3 =%-7d IR2 =%-7d IR1 =%-7d  \r\n\n",
//				get_prox(4),get_prox(3),get_prox(2),get_prox(1));


#ifdef PLOT_ON_COMPUTER
		//		To visualize one image on computer with plotImage.py
		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image_red, IMAGE_BUFFER_SIZE);
		}

		//invert the bool
		send_to_computer = !send_to_computer;
#endif

		}

}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

uint8_t get_color(void){
	return color_idx;
}

uint16_t get_middle_diff(void) {
	return middle_line_top-middle_line_bot;
}

uint16_t get_middle_top(void) {
	return middle_line_top;
}

uint16_t get_middle_bot(void) {
	return middle_line_bot;
}

void calc_line_middle(uint8_t alternator){

	chprintf((BaseSequentialStream *)&SD3, "Alternator =%-7d \r\n\n",
	               alternator);

	uint16_t middle = 0;

	if (color_idx == RED_IDX){
		middle = calc_middle(image_red);
	}
	else {
		if (color_idx == GREEN_IDX){
			middle = calc_middle(image_green);
		}

		else {
			if (color_idx == BLUE_IDX){
				middle = calc_middle(image_blue);
			}
		}
	}

	if (alternator == TOP){
		middle_line_top = middle;
	}
	else middle_line_bot = middle;
}

void filter_noise(uint16_t index, uint8_t red_value, uint8_t green_value, uint8_t blue_value){
	//filtering noise for each color
	if (red_value > threshold_color){
		image_red[index/2] = red_value;
	}
	else image_red[index/2] = 0;

	if (green_value > threshold_color){
		image_green[index/2] = green_value;
	}
	else image_green[index/2] = 0;

	if (blue_value >threshold_color){
		image_blue[index/2] = blue_value;
	}
	else image_blue[index/2] = 0;
}

void set_threshold_color(int selector_pos){

	switch (selector_pos)
	{
	case 0: //No noise filtering
		threshold_color = 0;
		break;
	case 1: //Minimum noise filtering
		threshold_color = 5;
		break;
	case 2:
		threshold_color = 10;
		break;
	case 3:
		threshold_color = 12;
		break;
	case 4:
		threshold_color = 14;
		break;
	case 5:
		threshold_color = 16;
		break;
	case 6:
		threshold_color = 18;
		break;
	case 7:
		threshold_color = 20;
		break;
	case 8:
		threshold_color = 22;
		break;
	case 9:
		threshold_color = 23;
		break;
	case 10:
		threshold_color = 24;
		break;
	case 11:
		threshold_color = 25;
		break;
	case 12:
		threshold_color = 26;
		break;
	case 13:
		threshold_color = 27;
		break;
	case 14:
		threshold_color = 28;
		break;
	case 15: //Maximum noise filtering
		threshold_color = 29;
		break;
	default:
		threshold_color = 15;
		break;
	}
}

void calc_max_mean(void){

	uint16_t temp_r = 0;
	uint16_t temp_g = 0;
	uint16_t temp_b = 0;

	uint16_t count_r = 0;
	uint16_t count_g = 0;
	uint16_t count_b = 0;

	uint8_t max_r = 0;
	uint8_t max_g = 0;
	uint8_t max_b = 0;

	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++){

		//MEAN
		if(image_red[i]> 0){
			temp_r = temp_r + image_red[i];
			count_r = count_r + 1;
		}

		if(image_green[i]> 0){
			temp_g = temp_g + image_green[i];
			count_g = count_g + 1;
		}

		if(image_blue[i]> 0){
			temp_b = temp_b + image_blue[i];
			count_b = count_b + 1;
		}

		//MAX
		if(image_red[i]> max_r){
			max_r = image_red[i];
		}

		if(image_green[i]> max_g){
			max_g = image_green[i];
		}

		if(image_blue[i]> max_b){
			max_b = image_blue[i];
		}
	}

//	chprintf((BaseSequentialStream *)&SD3, "%R Temp =%-7d G Temp =%-7d B Temp =%-7d %R C =%-7d G C =%-7d B C =%-7d \r\n\n",
//						              temp_r,temp_g,temp_b,count_r,count_g,count_b);

	mean_red = temp_r / count_r;
	mean_green = temp_g / count_g;
	mean_blue = temp_b / count_b;

//	chprintf((BaseSequentialStream *)&SD3, "%R Mean =%-7d G Mean =%-7d B Mean =%-7d \r\n\n",
//					              mean_red, mean_green, mean_blue);

	max_red = max_r;
	max_green = max_g;
	max_blue = max_b;

//	chprintf((BaseSequentialStream *)&SD3, "R Max =%-7d G Max =%-7d B Max =%-7d \r\n\n",
//						              max_red, max_green, max_blue);

}

void max_count(void){

	uint16_t count_r = 0;
	uint16_t count_g = 0;
	uint16_t count_b = 0;

	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++){
		if(image_red[i] > max_red - TOLERANCE){
			count_r = count_r +1;
		}
		if(image_green[i] > max_green - TOLERANCE){
			count_g = count_g +1;
		}
		if(image_blue[i] > max_blue - TOLERANCE){
			count_b = count_b +1;
		}
	}

	count_red = count_r;
	count_green = count_g;
	count_blue = count_b;

//	chprintf((BaseSequentialStream *)&SD3, "%R Count =%-7d G Count =%-7d B Count =%-7d \r\n\n",
//						              count_red, count_green, count_blue);

}

#ifdef USE_MAX_N_MEAN
void find_color(void){

	if ((((max_red > max_green) && (max_red > max_blue)) || ((mean_red > mean_green) && (mean_red > mean_blue))) && (count_red > MIN_COUNT)){
		color_idx = RED_IDX;
	}
	else{

		if ((((max_green > max_red) && (max_green > max_blue)) || ((mean_green > mean_red) && (mean_green > mean_blue))) && (count_green > MIN_COUNT)){
			color_idx = GREEN_IDX;
		}

		else {
			if ((((max_blue > max_red) && (max_blue > max_green)) || ((mean_blue > mean_red) && (mean_blue > mean_green))) && (count_blue > MIN_COUNT)){
				color_idx = BLUE_IDX;
			}
			else {
				//chprintf((BaseSequentialStream *)&SD3, "Resetting \n\n");
					color_idx = NO_COLOR;
			}
		}
	}

#ifdef SEND_DATA
		chprintf((BaseSequentialStream *)&SD3, "%R Max =%-7d G Max =%-7d B Max =%-7d \r\n\n",
							              max_red, max_green, max_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Mean =%-7d G Mean =%-7d B Mean =%-7d \r\n\n",
							              mean_red, mean_green, mean_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Count =%-7d G Count =%-7d B Count =%-7d \r\n\n",
								              count_red, count_green, count_blue);
#endif

}
#endif

#ifdef USE_ONLY_MAX
void find_color(void){

	if (((max_red > max_green) && (max_red > max_blue)) && (count_red > MIN_COUNT)){
		color_idx = RED_IDX;
	}
	else{
		if (((max_green > max_red) && (max_green > max_blue)) && (count_green > MIN_COUNT)){
			color_idx = GREEN_IDX;
		}
		else {
			if (((max_blue > max_red) && (max_blue > max_green)) && (count_blue > MIN_COUNT)){
				color_idx = BLUE_IDX;
			}
			else {
					color_idx = NO_COLOR;
			}
		}
	}

#ifdef SEND_DATA
		chprintf((BaseSequentialStream *)&SD3, "%R Max =%-7d G Max =%-7d B Max =%-7d \r\n\n",
							              max_red, max_green, max_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Mean =%-7d G Mean =%-7d B Mean =%-7d \r\n\n",
							              mean_red, mean_green, mean_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Count =%-7d G Count =%-7d B Count =%-7d \r\n\n",
								              count_red, count_green, count_blue);
#endif
}
#endif

#ifdef USE_ONLY_MEAN
void find_color(void){

	if (((mean_red > mean_green) && (mean_red > mean_blue)) && (count_red > MIN_COUNT)){
		color_idx = RED_IDX;
	}
	else{
		if (((mean_green > mean_red) && (mean_green > mean_blue)) && (count_green > MIN_COUNT)){
			color_idx = GREEN_IDX;
		}
		else {
			if (((mean_blue > mean_red) && (mean_blue > mean_green)) && (count_blue > MIN_COUNT)){
				color_idx = BLUE_IDX;
			}
			else {
					color_idx = NO_COLOR;
			}
		}
	}
#ifdef SEND_DATA
			chprintf((BaseSequentialStream *)&SD3, "%R Max =%-7d G Max =%-7d B Max =%-7d \r\n\n",
								              max_red, max_green, max_blue);

			chprintf((BaseSequentialStream *)&SD3, "%R Mean =%-7d G Mean =%-7d B Mean =%-7d \r\n\n",
								              mean_red, mean_green, mean_blue);

			chprintf((BaseSequentialStream *)&SD3, "%R Count =%-7d G Count =%-7d B Count =%-7d \r\n\n",
									              count_red, count_green, count_blue);
#endif
}
#endif

