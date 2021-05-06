#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <string.h>
#include <stdbool.h>
#include <leds.h>
#include <main.h>
#include <camera/po8030.h>
#include <tune_camera.h>
#include <selector.h>

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

void tune_filter_noise(uint16_t index, uint8_t red_value, uint8_t green_value, uint8_t blue_value);
void tune_set_threshold_color(int selector_pos);
void tune_calc_max_mean(void);
void tune_max_count(void);
void set_detect_mode(detect_mode_t detection_mode);
void set_data_bool(bool send_data);

//semaphore
static BSEMAPHORE_DECL(tune_image_ready_sem, TRUE);

static THD_WORKING_AREA(waTuneCaptureImage, 256);
static THD_FUNCTION(TuneCaptureImage, arg) {

	chRegSetThreadName(__FUNCTION__);

	uint8_t start = 0;
	uint8_t contr = 0;
	uint16_t idx = 0;
	if (start == 0){
		struct tunning_config *tune = (struct tunning_config *)arg;
		contr = tune->contrast;
		idx = tune->line_idx;
		set_detect_mode(tune->detection_mode);
		set_data_bool(tune->send_data_terminal);
		start = 1;
	}

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line LINE_INDEX + LINE_INDEX+1 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, idx, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);

	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();
	po8030_set_awb(0);
	po8030_set_contrast(contr);

	while(1){
		//starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&tune_image_ready_sem);
	}

}

static THD_WORKING_AREA(waTuneProcessImage, 1024);
static THD_FUNCTION(TuneProcessImage, arg) {


	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	uint8_t *img_buff_ptr;

	bool send_to_computer = true;

	while(1){
		//waits until an image has been captured
		chBSemWait(&tune_image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();

		tune_set_threshold_color(get_selector());

		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){

			//extracting red 5 bits and shifting them right
			uint8_t r = ((uint8_t)img_buff_ptr[i]&0xF8) >> SHIFT_3;

			//Extract G6G5G4G3G2
			uint8_t g = (((uint8_t)img_buff_ptr[i]&0x07) << 2) + (((uint8_t)img_buff_ptr[i+1]&0xC0) >> 6);

			//extracting blue 5 bits
			uint8_t b = (uint8_t)img_buff_ptr[i+1]&0x1F;

			tune_filter_noise(i, r, g, b);

		}
		tune_calc_max_mean();
		tune_max_count();

		//To visualize one image on computer with plotImage.py
		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image_red, IMAGE_BUFFER_SIZE);
		}

		//invert the bool
		send_to_computer = !send_to_computer;
	}

}

void tune_image_start(struct tunning_config arg_contrast){
	chThdCreateStatic(waTuneProcessImage, sizeof(waTuneProcessImage), NORMALPRIO, TuneProcessImage, NULL);
	chThdCreateStatic(waTuneCaptureImage, sizeof(waTuneCaptureImage), NORMALPRIO, TuneCaptureImage, &arg_contrast);
}

void tune_filter_noise(uint16_t index, uint8_t red_value, uint8_t green_value, uint8_t blue_value){
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

void tune_set_threshold_color(int selector_pos){

	switch (selector_pos)
	{
	case 0:
		threshold_color = 0;
		break;
	case 1:
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
	case 15:
		threshold_color = 29;
		break;
	default:
		threshold_color = 15;
		break;
	}
}

void tune_calc_max_mean(void){

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

	mean_red = temp_r / count_r;
	mean_green = temp_g / count_g;
	mean_blue = temp_b / count_b;

	max_red = max_r;
	max_green = max_g;
	max_blue = max_b;
}

void tune_max_count(void){

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

}

#ifdef USE_MAX_N_MEAN
void tune_find_color(void){

	if ((((max_red > max_green) && (max_red > max_blue)) || ((mean_red > mean_green) && (mean_red > mean_blue))) && (count_red > MIN_COUNT)){
		set_rgb_led(0, 10, 0, 0);
		set_rgb_led(1, 10, 0, 0);
		set_rgb_led(2, 10, 0, 0);
		set_rgb_led(3, 10, 0, 0);
		color_idx = RED_IDX;
	}
	else{

		if ((((max_green > max_red) && (max_green > max_blue)) || ((mean_green > mean_red) && (mean_green > mean_blue))) && (count_green > MIN_COUNT)){
			set_rgb_led(0, 0, 10, 0);
			set_rgb_led(1, 0, 10, 0);
			set_rgb_led(2, 0, 10, 0);
			set_rgb_led(3, 0, 10, 0);
			color_idx = GREEN_IDX;
		}

		else {
			if ((((max_blue > max_red) && (max_blue > max_green)) || ((mean_blue > mean_red) && (mean_blue > mean_green))) && (count_blue > MIN_COUNT)){
				set_rgb_led(0, 0, 0, 10);
				set_rgb_led(1, 0, 0, 10);
				set_rgb_led(2, 0, 0, 10);
				set_rgb_led(3, 0, 0, 10);
				color_idx = BLUE_IDX;
			}
			else {
				//chprintf((BaseSequentialStream *)&SD3, "Resetting \n\n");
				set_rgb_led(0, 0, 0, 0);
				set_rgb_led(1, 0, 0, 0);
				set_rgb_led(2, 0, 0, 0);
				set_rgb_led(3, 0, 0, 0);
				color_idx = NO_COLOR;
			}
		}
	}

	chprintf((BaseSequentialStream *)&SD3, "%R Max =%-7d G Max =%-7d B Max =%-7d \r\n\n",
			max_red, max_green, max_blue);

	chprintf((BaseSequentialStream *)&SD3, "%R Mean =%-7d G Mean =%-7d B Mean =%-7d \r\n\n",
			mean_red, mean_green, mean_blue);

	chprintf((BaseSequentialStream *)&SD3, "%R Count =%-7d G Count =%-7d B Count =%-7d \r\n\n",
			count_red, count_green, count_blue);

}
#endif

#ifdef USE_ONLY_MAX
void tune_find_color(void){

	if (((max_red > max_green) && (max_red > max_blue)) && (count_red > MIN_COUNT)){
		set_rgb_led(0, 10, 0, 0);
		set_rgb_led(1, 10, 0, 0);
		set_rgb_led(2, 10, 0, 0);
		set_rgb_led(3, 10, 0, 0);
		color_idx = RED_IDX;
	}
	else{

		if (((max_green > max_red) && (max_green > max_blue)) && (count_green > MIN_COUNT)){
			set_rgb_led(0, 0, 10, 0);
			set_rgb_led(1, 0, 10, 0);
			set_rgb_led(2, 0, 10, 0);
			set_rgb_led(3, 0, 10, 0);
			color_idx = GREEN_IDX;
		}

		else {
			if (((max_blue > max_red) && (max_blue > max_green)) && (count_blue > MIN_COUNT)){
				set_rgb_led(0, 0, 0, 10);
				set_rgb_led(1, 0, 0, 10);
				set_rgb_led(2, 0, 0, 10);
				set_rgb_led(3, 0, 0, 10);
				color_idx = BLUE_IDX;
			}
			else {
				//chprintf((BaseSequentialStream *)&SD3, "Resetting \n\n");
				set_rgb_led(0, 0, 0, 0);
				set_rgb_led(1, 0, 0, 0);
				set_rgb_led(2, 0, 0, 0);
				set_rgb_led(3, 0, 0, 0);
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
void tune_find_color(void){

	if (((mean_red > mean_green) && (mean_red > mean_blue)) && (count_red > MIN_COUNT)){
		set_rgb_led(0, 10, 0, 0);
		set_rgb_led(1, 10, 0, 0);
		set_rgb_led(2, 10, 0, 0);
		set_rgb_led(3, 10, 0, 0);
		color_idx = RED_IDX;
	}
	else{

		if (((mean_green > mean_red) && (mean_green > mean_blue)) && (count_green > MIN_COUNT)){
			set_rgb_led(0, 0, 10, 0);
			set_rgb_led(1, 0, 10, 0);
			set_rgb_led(2, 0, 10, 0);
			set_rgb_led(3, 0, 10, 0);
			color_idx = GREEN_IDX;
		}

		else {
			if (((mean_blue > mean_red) && (mean_blue > mean_green)) && (count_blue > MIN_COUNT)){
				set_rgb_led(0, 0, 0, 10);
				set_rgb_led(1, 0, 0, 10);
				set_rgb_led(2, 0, 0, 10);
				set_rgb_led(3, 0, 0, 10);
				color_idx = BLUE_IDX;
			}
			else {
				//chprintf((BaseSequentialStream *)&SD3, "Resetting \n\n");
				set_rgb_led(0, 0, 0, 0);
				set_rgb_led(1, 0, 0, 0);
				set_rgb_led(2, 0, 0, 0);
				set_rgb_led(3, 0, 0, 0);
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

void set_detect_mode(detect_mode_t detection_mode){
	if (detection_mode == MAX_ONLY){
		#define USE_MAX
	}
	else {
		if (detection_mode == MEAN_ONLY){
			#define USE_MEAN
		}
		else {
			#define USE_MAX_N_MEAN
		}
	}
}
void set_data_bool(bool send_data){
	if (send_data) {
		#define SEND_DATA
	}
}