#include "read_image.h"
#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <string.h>
#include <stdbool.h>
#include <main.h>
#include <camera/po8030.h>
#include <selector.h>

//Longeur d'une ligne de pixels de la camera et tolerance pour les comptages du nombre de pixels egaux a l'intensite maximal
#define IMAGE_BUFFER_SIZE			640
#define TOLERANCE					3

//Pour alterner lors du traitement des deux lignes de la camera
#define TOP							0
#define BOTTOM						1

//Le nombre minimum de pixel pour valider une detection de ligne pour une certaine couleur
#define MIN_COUNT					5

//Pour trouver le milieu de la ligne, condition sur largeur de ligne et "trous" dans une ligne
#define MIN_LINE_WIDTH				70
#define MIN_HOLE_WIDTH				20

//Shift pour remettre les bits des couleurs dans l'ordre lors de l'extraction du format RGB565
#define SHIFT_2						2
#define SHIFT_3						3
#define SHIFT_6						6

typedef struct {

	//After tuning adjust to the desired detection mode
	detect_mode_t detection;
	//To visualize maxs, means and counts for each color
	visualize_mode_t send_data;

	rgb_gain_t rgb_gains;

	uint8_t contrast;

	uint16_t line_idx_top;

	uint16_t count_red;
	uint16_t count_green;
	uint16_t count_blue;

	uint8_t max_red;
	uint8_t max_green;
	uint8_t max_blue;

	uint8_t mean_red;
	uint8_t mean_green;
	uint8_t mean_blue;

	uint8_t image_red[IMAGE_BUFFER_SIZE];
	uint8_t image_green[IMAGE_BUFFER_SIZE];
	uint8_t image_blue[IMAGE_BUFFER_SIZE];

	color_index_t color_index;
	uint8_t threshold_color;

#ifndef TUNE
	uint16_t line_idx_bot;
	uint8_t image_bot[IMAGE_BUFFER_SIZE];
	int16_t middle_line_top; //middle of line
	int16_t middle_line_bot;
#endif

} VISUAL_CONTEXT_t;

static VISUAL_CONTEXT_t image_context;

void find_color(void);
void set_threshold_color(int selector_pos);
void calc_max_mean(void);
void max_count(void);
void filter_noise(uint16_t index, uint8_t red_value, uint8_t green_value, uint8_t blue_value);
uint8_t get_color(void);


#ifdef TUNE
//semaphore
static BSEMAPHORE_DECL(tune_image_ready_sem, TRUE);

// Tuning threads and functions
static THD_WORKING_AREA(waTuneCaptureImage, 256);
static THD_FUNCTION(TuneCaptureImage, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line LINE_INDEX + LINE_INDEX+1 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, image_context.line_idx_top, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);

	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();
	po8030_set_awb(0);
	po8030_set_contrast(image_context.contrast);
	//default : 94 (1.46875), 64 (1), 93 (1.453125)
	//po8030_set_rgb_gain(94, 80, 0);
	po8030_set_rgb_gain(image_context.rgb_gains.red_gain,image_context.rgb_gains.green_gain,image_context.rgb_gains.blue_gain);

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

		set_threshold_color(get_selector());

		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){

			//extracting red 5 bits and shifting them right
			uint8_t r = ((uint8_t)img_buff_ptr[i]&0xF8) >> SHIFT_3;

			//Extract G6G5G4G3G2
			uint8_t g = (((uint8_t)img_buff_ptr[i]&0x07) << SHIFT_2) + (((uint8_t)img_buff_ptr[i+1]&0xC0) >> SHIFT_6);

			//extracting blue 5 bits
			uint8_t b = (uint8_t)img_buff_ptr[i+1]&0x1F;

			filter_noise(i, r, g, b);

		}

		calc_max_mean();
		max_count();
		find_color();

		if (image_context.send_data == NO_VISUALIZE_PARAMS){
			//To visualize one image on computer with plotImage.py
			if(send_to_computer){
				//sends to the computer the image
				if (image_context.color_index == RED_IDX) SendUint8ToComputer(image_context.image_red, IMAGE_BUFFER_SIZE);
				if (image_context.color_index == GREEN_IDX) SendUint8ToComputer(image_context.image_green, IMAGE_BUFFER_SIZE);
				if (image_context.color_index == BLUE_IDX) SendUint8ToComputer(image_context.image_blue, IMAGE_BUFFER_SIZE);
			}

			//invert the bool
			send_to_computer = !send_to_computer;
		}

	}

}

void init_visual_context_tune(tuning_config_t received_config){


	image_context.contrast = received_config.contrast;
	image_context.line_idx_top = received_config.line_idx;
	image_context.color_index = received_config.color_idx;
	image_context.detection = received_config.detection_mode;
	image_context.send_data = received_config.send_data_terminal;

	image_context.rgb_gains.red_gain = received_config.rgb_gain.red_gain;
	image_context.rgb_gains.green_gain = received_config.rgb_gain.green_gain;
	image_context.rgb_gains.blue_gain = received_config.rgb_gain.blue_gain;

	image_context.count_red = 0;
	image_context.count_green = 0;
	image_context.count_blue = 0;

	image_context.max_red = 0;
	image_context.max_green = 0;
	image_context.max_blue = 0;

	image_context.mean_red = 0;
	image_context.mean_green = 0;
	image_context.mean_blue = 0;

	image_context.threshold_color = 0;

	for (int16_t i = 0; i < IMAGE_BUFFER_SIZE; ++i){
		image_context.image_red [i] = 0;
		image_context.image_green [i] = 0;
		image_context.image_blue [i] = 0;
	}

}

void tune_image_start(tuning_config_t arg_tune_settings){
	init_visual_context_tune(arg_tune_settings);
	chThdCreateStatic(waTuneProcessImage, sizeof(waTuneProcessImage), NORMALPRIO, TuneProcessImage, NULL);
	chThdCreateStatic(waTuneCaptureImage, sizeof(waTuneCaptureImage), NORMALPRIO, TuneCaptureImage, NULL);
}

#else

//Uncomment to use plot_image.py for debug
//#define PLOT_ON_COMPUTER

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);
static BSEMAPHORE_DECL(image_ready_sem_2, TRUE);

void init_visual_context(config_t received_config);
void calc_line_middle(uint8_t alternator);
uint8_t filter_noise_single(uint8_t couleur);
int16_t calc_middle(uint8_t *buffer){

	uint16_t start_p = 0;
	uint16_t end_p = 0;
	uint16_t start_n = 0;
	uint16_t end_n = 0;

	uint16_t begin_line = 0;
	uint16_t hole_size = 0;

	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; ++i){
		if ((begin_line == 0) && (buffer[i] > 0)){
			begin_line = 1;
			start_n = i;
			continue;
		}
		else{
			if (((i == IMAGE_BUFFER_SIZE - 1) && (buffer[i] > 0)) && (begin_line == 1)){
				end_n = i;
				if ((end_n - start_n > MIN_LINE_WIDTH) && (end_n - start_n > end_p - start_p)){
					start_p = start_n;
					end_p = end_n;
				}
			}
			else{
				if ((begin_line == 1) && (buffer[i] == 0)){
					hole_size = 1;
					end_n = i-1;
					if (i == IMAGE_BUFFER_SIZE - 1){
						end_n = i - 1;
						if ((end_n - start_n > MIN_LINE_WIDTH) && (end_n - start_n > end_p - start_p)){
							start_p = start_n;
							end_p = end_n;
						}
						break;
					}

					for (int j = i + 1; j < IMAGE_BUFFER_SIZE; ++j){

						if (buffer[j] == 0){

							hole_size++;

							if (hole_size > MIN_HOLE_WIDTH){

								begin_line = 0;

								if ((end_n - start_n > MIN_LINE_WIDTH) && (end_n - start_n > end_p - start_p)){
									start_p = start_n;
									end_p = end_n;
								}
								i = j;
								break;
							}
						}
						else{
							i = j-1;
							break;
						}
					}
				}
			}
		}
	}
	if (end_n - start_n < end_p - start_p) {
		start_n = start_p;
		end_n = end_p;
	}

	return (end_n+start_n)/2;
}


void calc_line_middle(uint8_t alternator){

	if (alternator == TOP){
		if (image_context.color_index == RED_IDX){
			image_context.middle_line_top = calc_middle(image_context.image_red);
		}
		else {
			if (image_context.color_index == GREEN_IDX){
				image_context.middle_line_top = calc_middle(image_context.image_green);
			}

			else {
				if (image_context.color_index == BLUE_IDX){
					image_context.middle_line_top = calc_middle(image_context.image_blue);
				}
			}
		}

	}
	else {
		image_context.middle_line_bot = calc_middle(image_context.image_bot);
	}
}

uint8_t filter_noise_single(uint8_t couleur){
	if (couleur > image_context.threshold_color){
		return couleur;
	}
	else return 0;
}

int16_t get_middle_diff(void) {
	return image_context.middle_line_top - image_context.middle_line_bot;
}

int16_t get_middle_top(void) {
	return image_context.middle_line_top;
}

int16_t get_middle_bot(void) {
	return image_context.middle_line_bot;
}

static THD_WORKING_AREA(waCaptureImage, 512);
static THD_FUNCTION(CaptureImage, arg) {

	chRegSetThreadName(__FUNCTION__);

	(void)arg;

	while(1){

		//Line index 413 detecting colors goes wrong
		//po8030_advanced_config(FORMAT_RGB565, 0, 413, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);

		po8030_advanced_config(FORMAT_RGB565, 0, image_context.line_idx_top, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
		dcmi_enable_double_buffering();
		dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
		dcmi_prepare();
		po8030_set_awb(0);
		//po8030_set_mirror(0, 1);
		po8030_set_contrast(image_context.contrast);

		//starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready(); //fait l'attente dans le while(1)

		//signals an image has been captured
		chBSemSignal(&image_ready_sem);

		po8030_advanced_config(FORMAT_RGB565, 0, image_context.line_idx_bot, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
		dcmi_enable_double_buffering();
		dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
		dcmi_prepare();
		po8030_set_awb(0);
		//po8030_set_mirror(0, 1);
		po8030_set_contrast(image_context.contrast);
		po8030_set_rgb_gain(image_context.rgb_gains.red_gain,image_context.rgb_gains.green_gain,image_context.rgb_gains.blue_gain);

		//starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready(); //fait l'attente dans le while(1)

		//signals an image has been captured
		chBSemSignal(&image_ready_sem_2);
	}
}

static THD_WORKING_AREA(waProcessImage, 2048);
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
			uint8_t g = (((uint8_t)img_buff_ptr[i]&0x07) << SHIFT_2) + (((uint8_t)img_buff_ptr[i+1]&0xC0) >> SHIFT_6);

			//extracting blue 5 bits
			uint8_t b = (uint8_t)img_buff_ptr[i+1]&0x1F;

			filter_noise(i, r, g, b);

		}
		calc_max_mean();
		max_count();
		find_color();

		//search for a line in the image and gets its middle position
		calc_line_middle(TOP);

		chBSemWait(&image_ready_sem_2);

		img_buff_ptr = dcmi_get_last_image_ptr();

		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			uint8_t c = 0;
			if (image_context.color_index == RED_IDX){
				c = ((uint8_t)img_buff_ptr[i]&0xF8) >> SHIFT_3;
			}
			else {
				if (image_context.color_index == GREEN_IDX){
					c = (((uint8_t)img_buff_ptr[i]&0x07) << SHIFT_2) + (((uint8_t)img_buff_ptr[i+1]&0xC0) >> SHIFT_6);
				}
				else {
					if (image_context.color_index == BLUE_IDX){
						c = (uint8_t)img_buff_ptr[i+1]&0x1F;
					}
				}
			}
			image_context.image_bot[i/2] = filter_noise_single(c);
		}
		calc_line_middle(BOTTOM);

#ifdef PLOT_ON_COMPUTER
		// To visualize one image on computer with plotImage.py
		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image_context.image_red, IMAGE_BUFFER_SIZE);
		}

		//invert the bool
		send_to_computer = !send_to_computer;
#endif

	}
}

void init_visual_context(config_t received_config){

	//After tuning adjust to the desired detection mode
	image_context.detection = received_config.detection_mode;
	//To visualize maxs, means and counts for each color
	image_context.send_data = received_config.send_data_terminal;

	image_context.contrast = received_config.contrast;

	image_context.line_idx_top = received_config.line_idx_top;
	image_context.line_idx_bot = received_config.line_idx_bot;

	image_context.count_red = 0;
	image_context.count_green = 0;
	image_context.count_blue = 0;

	image_context.max_red = 0;
	image_context.max_green = 0;
	image_context.max_blue = 0;

	image_context.mean_red = 0;
	image_context.mean_green = 0;
	image_context.mean_blue = 0;

	for (int16_t i = 0; i < IMAGE_BUFFER_SIZE; ++i){
		image_context.image_red [i] = 0;
		image_context.image_green [i] = 0;
		image_context.image_blue [i] = 0;
		image_context.image_bot [i] = 0;
	}

	image_context.color_index = 0;
	image_context.threshold_color = 0;

	image_context.middle_line_top = IMAGE_BUFFER_SIZE/2;; //middle of line
	image_context.middle_line_bot = IMAGE_BUFFER_SIZE/2;

}

void read_image_start(config_t arg_config){
	init_visual_context(arg_config);
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

#endif


void filter_noise(uint16_t index, uint8_t red_value, uint8_t green_value, uint8_t blue_value){
	//filtering noise for each color
	if (red_value > image_context.threshold_color){
		image_context.image_red[index/2] = red_value;
	}
	else image_context.image_red[index/2] = 0;

	if (green_value > image_context.threshold_color){
		image_context.image_green[index/2] = green_value;
	}
	else image_context.image_green[index/2] = 0;

	if (blue_value > image_context.threshold_color){
		image_context.image_blue[index/2] = blue_value;
	}
	else image_context.image_blue[index/2] = 0;
}



void set_threshold_color(int selector_pos){

	switch (selector_pos)
	{
	case 0: //No noise filtering
		image_context.threshold_color = 0;
		break;
	case 1: //Minimum noise filtering
		image_context.threshold_color = 5;
		break;
	case 2:
		image_context.threshold_color = 10;
		break;
	case 3:
		image_context.threshold_color = 12;
		break;
	case 4:
		image_context.threshold_color = 14;
		break;
	case 5:
		image_context.threshold_color = 16;
		break;
	case 6:
		image_context.threshold_color = 18;
		break;
	case 7:
		image_context.threshold_color = 20;
		break;
	case 8:
		image_context.threshold_color = 22;
		break;
	case 9:
		image_context.threshold_color = 23;
		break;
	case 10:
		image_context.threshold_color = 24;
		break;
	case 11:
		image_context.threshold_color = 25;
		break;
	case 12:
		image_context.threshold_color = 26;
		break;
	case 13:
		image_context.threshold_color = 27;
		break;
	case 14:
		image_context.threshold_color = 28;
		break;
	case 15: //Maximum noise filtering
		image_context.threshold_color = 29;
		break;
	default:
		image_context.threshold_color = 15;
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

	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; ++i){

		//MEAN
		if(image_context.image_red[i]> 0){
			temp_r = temp_r + image_context.image_red[i];
			count_r = count_r + 1;
		}

		if(image_context.image_green[i]> 0){
			temp_g = temp_g + image_context.image_green[i];
			count_g = count_g + 1;
		}

		if(image_context.image_blue[i]> 0){
			temp_b = temp_b + image_context.image_blue[i];
			count_b = count_b + 1;
		}

		//MAX
		if(image_context.image_red[i]> max_r){
			max_r = image_context.image_red[i];
		}

		if(image_context.image_green[i]> max_g){
			max_g = image_context.image_green[i];
		}

		if(image_context.image_blue[i]> max_b){
			max_b = image_context.image_blue[i];
		}
	}


	image_context.mean_red = temp_r / count_r;
	image_context.mean_green = temp_g / count_g;
	image_context.mean_blue = temp_b / count_b;

	image_context.max_red = max_r;
	image_context.max_green = max_g;
	image_context.max_blue = max_b;

}

void max_count(void){

	uint16_t count_r = 0;
	uint16_t count_g = 0;
	uint16_t count_b = 0;

	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; ++i){
		if(image_context.image_red[i] > image_context.max_red - TOLERANCE){
			count_r = count_r +1;
		}
		if(image_context.image_green[i] > image_context.max_green - TOLERANCE){
			count_g = count_g +1;
		}
		if(image_context.image_blue[i] > image_context.max_blue - TOLERANCE){
			count_b = count_b +1;
		}
	}

	if (count_r == IMAGE_BUFFER_SIZE){
		count_r = 0;
	}

	if (count_g == IMAGE_BUFFER_SIZE){
		count_g = 0;
	}

	if (count_b == IMAGE_BUFFER_SIZE){
		count_b = 0;
	}

	image_context.count_red = count_r;
	image_context.count_green = count_g;
	image_context.count_blue = count_b;

}

uint8_t get_color(void){
	return image_context.color_index;
}

void find_color_max_n_mean(void){

	if ((((image_context.max_red > image_context.max_green) && (image_context.max_red > image_context.max_blue)) || ((image_context.mean_red > image_context.mean_green) && (image_context.mean_red > image_context.mean_blue))) && (image_context.count_red > MIN_COUNT)){
		image_context.color_index = RED_IDX;
#ifdef TUNE
		set_leds(RED_IDX);
#endif
	}
	else{

		if ((((image_context.max_green > image_context.max_red) && (image_context.max_green > image_context.max_blue)) || ((image_context.mean_green > image_context.mean_red) && (image_context.mean_green > image_context.mean_blue))) && (image_context.count_green > MIN_COUNT)){
			image_context.color_index = GREEN_IDX;
#ifdef TUNE
			set_leds(GREEN_IDX);
#endif
		}

		else {
			if ((((image_context.max_blue > image_context.max_red) && (image_context.max_blue > image_context.max_green)) || ((image_context.mean_blue > image_context.mean_red) && (image_context.mean_blue > image_context.mean_green))) && (image_context.count_blue > MIN_COUNT)){
				image_context.color_index = BLUE_IDX;
#ifdef TUNE
			set_leds(BLUE_IDX);
#endif
			}
			else {
				image_context.color_index = NO_COLOR;
#ifdef TUNE
				set_leds(NO_COLOR);
#endif
			}
		}
	}

	if(image_context.send_data){
		chprintf((BaseSequentialStream *)&SD3, "%R Max =%-7d G Max =%-7d B Max =%-7d \r\n\n",
				image_context.max_red, image_context.max_green, image_context.max_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Mean =%-7d G Mean =%-7d B Mean =%-7d \r\n\n",
				image_context.mean_red, image_context.mean_green, image_context.mean_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Count =%-7d G Count =%-7d B Count =%-7d \r\n\n",
				image_context.count_red, image_context.count_green, image_context.count_blue);
	}

}

void find_color_max(void){

	if (((image_context.max_red > image_context.max_green) && (image_context.max_red > image_context.max_blue)) && (image_context.count_red > MIN_COUNT)){
		image_context.color_index = RED_IDX;
#ifdef TUNE
		set_leds(RED_IDX);
#endif
	}
	else{
		if (((image_context.max_green > image_context.max_red) && (image_context.max_green > image_context.max_blue)) && (image_context.count_green > MIN_COUNT)){
			image_context.color_index = GREEN_IDX;
#ifdef TUNE
			set_leds(GREEN_IDX);
#endif
		}
		else {
			if (((image_context.max_blue > image_context.max_red) && (image_context.max_blue > image_context.max_green)) && (image_context.count_blue > MIN_COUNT)){
				image_context.color_index = BLUE_IDX;
#ifdef TUNE
				set_leds(BLUE_IDX);
#endif
			}
			else {
				image_context.color_index = NO_COLOR;
#ifdef TUNE
				set_leds(NO_COLOR);
#endif
			}
		}
	}

	if(image_context.send_data){
		chprintf((BaseSequentialStream *)&SD3, "%R Max =%-7d G Max =%-7d B Max =%-7d \r\n\n",
				image_context.max_red, image_context.max_green, image_context.max_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Mean =%-7d G Mean =%-7d B Mean =%-7d \r\n\n",
				image_context.mean_red, image_context.mean_green, image_context.mean_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Count =%-7d G Count =%-7d B Count =%-7d \r\n\n",
				image_context.count_red, image_context.count_green, image_context.count_blue);
	}
}

void find_color_mean(void){

	if (((image_context.mean_red > image_context.mean_green) && (image_context.mean_red > image_context.mean_blue)) && (image_context.count_red > MIN_COUNT)){
		image_context.color_index = RED_IDX;
#ifdef TUNE
		set_leds(RED_IDX);
#endif
	}
	else{
		if (((image_context.mean_green > image_context.mean_red) && (image_context.mean_green > image_context.mean_blue)) && (image_context.count_green > MIN_COUNT)){
			image_context.color_index = GREEN_IDX;
#ifdef TUNE
			set_leds(GREEN_IDX);
#endif
		}
		else {
			if (((image_context.mean_blue > image_context.mean_red) && (image_context.mean_blue > image_context.mean_green)) && (image_context.count_blue > MIN_COUNT)){
				image_context.color_index = BLUE_IDX;
#ifdef TUNE
				set_leds(BLUE_IDX);
#endif
			}
			else {
				image_context.color_index = NO_COLOR;
#ifdef TUNE
				set_leds(NO_COLOR);
#endif
			}
		}
	}
	if(image_context.send_data){
		chprintf((BaseSequentialStream *)&SD3, "%R Max =%-7d G Max =%-7d B Max =%-7d \r\n\n",
				image_context.max_red, image_context.max_green, image_context.max_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Mean =%-7d G Mean =%-7d B Mean =%-7d \r\n\n",
				image_context.mean_red, image_context.mean_green, image_context.mean_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Count =%-7d G Count =%-7d B Count =%-7d \r\n\n",
				image_context.count_red, image_context.count_green, image_context.count_blue);
	}

}

void find_color_max_n_count(void){

	if (((image_context.max_red > image_context.max_green) && (image_context.max_red > image_context.max_blue)) && (image_context.count_red > MIN_COUNT)  && (image_context.count_red > image_context.count_green)  && (image_context.count_red > image_context.count_blue)){
		image_context.color_index = RED_IDX;
#ifdef TUNE
		set_leds(RED_IDX);
#endif
	}
	else{
		if (((image_context.max_green > image_context.max_red) && (image_context.max_green > image_context.max_blue)) && (image_context.count_green > MIN_COUNT)  && (image_context.count_green > image_context.count_red)  && (image_context.count_green > image_context.count_blue)){
			image_context.color_index = GREEN_IDX;
#ifdef TUNE
			set_leds(GREEN_IDX);
#endif
		}
		else {
			if (((image_context.max_blue > image_context.max_red) && (image_context.max_blue > image_context.max_green)) && (image_context.count_blue > MIN_COUNT)  && (image_context.count_blue > image_context.count_red)  && (image_context.count_blue > image_context.count_green)){
				image_context.color_index = BLUE_IDX;
#ifdef TUNE
				set_leds(BLUE_IDX);
#endif
			}
			else {
				image_context.color_index = NO_COLOR;
#ifdef TUNE
				set_leds(NO_COLOR);
#endif
			}
		}
	}
	if(image_context.send_data){
		chprintf((BaseSequentialStream *)&SD3, "%R Max =%-7d G Max =%-7d B Max =%-7d \r\n\n",
				image_context.max_red, image_context.max_green, image_context.max_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Mean =%-7d G Mean =%-7d B Mean =%-7d \r\n\n",
				image_context.mean_red, image_context.mean_green, image_context.mean_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Count =%-7d G Count =%-7d B Count =%-7d \r\n\n",
				image_context.count_red, image_context.count_green, image_context.count_blue);
	}

}

void find_color_rainy_day(void){

	if ((image_context.max_red < 29) && (image_context.max_green < 29) && (image_context.max_blue < 29)){
		image_context.color_index = NO_COLOR;

#ifdef TUNE
		set_leds(NO_COLOR);
#endif

	}

	else {
		if ((image_context.max_red - image_context.max_green > 2) && (image_context.max_red - image_context.max_blue > 2)){
			image_context.color_index = RED_IDX;
#ifdef TUNE
			set_leds(RED_IDX);
#endif
		}
		else {
			if ((image_context.max_red - image_context.max_green < 2) && (image_context.max_red - image_context.max_blue > 2)){
				if ((image_context.mean_red > image_context.mean_green) && (image_context.count_red > MIN_COUNT)){
					image_context.color_index = RED_IDX;
#ifdef TUNE
					set_leds(RED_IDX);
#endif
				}
				else {
					if ((image_context.mean_red < image_context.mean_green) && (image_context.count_green > MIN_COUNT)){
						image_context.color_index = GREEN_IDX;
	#ifdef TUNE
						set_leds(GREEN_IDX);
	#endif
					}
				}
			}
			else {
				if ((image_context.max_red - image_context.max_blue < 2) && (image_context.max_red - image_context.max_green > 2)){
					if ((image_context.mean_red > image_context.mean_blue) && (image_context.count_red > MIN_COUNT)){
						image_context.color_index = RED_IDX;
#ifdef TUNE
						set_leds(RED_IDX);
#endif
					}
					else {
						if ((image_context.mean_blue > image_context.mean_red) && (image_context.count_blue > MIN_COUNT)){
							image_context.color_index = BLUE_IDX;
#ifdef TUNE
							set_leds(BLUE_IDX);
#endif
						}
					}
				}
			}
		}


		if ((image_context.max_green - image_context.max_red > 2) && (image_context.max_green - image_context.max_blue > 2)){
			image_context.color_index = GREEN_IDX;
#ifdef TUNE
			set_leds(GREEN_IDX);
#endif
		}
		else {
			if ((image_context.max_green - image_context.max_red < 2) && (image_context.max_green - image_context.max_blue > 2)){
				if ((image_context.mean_green > image_context.mean_red) && (image_context.count_green > MIN_COUNT)){
					image_context.color_index = GREEN_IDX;
#ifdef TUNE
					set_leds(GREEN_IDX);
#endif
				}
				else {
					if ((image_context.mean_green < image_context.mean_red) && (image_context.count_red > MIN_COUNT)){
						image_context.color_index = RED_IDX;
#ifdef TUNE
						set_leds(RED_IDX);
#endif
					}
				}
			}
			else {
				if ((image_context.max_green - image_context.max_blue < 2) && (image_context.max_green - image_context.max_red > 2)){
					if ((image_context.mean_green > image_context.mean_blue) && (image_context.count_green > MIN_COUNT)){
						image_context.color_index = GREEN_IDX;
#ifdef TUNE
						set_leds(GREEN_IDX);
#endif
					}
					else {
						if ((image_context.mean_blue > image_context.mean_green) && (image_context.count_blue > MIN_COUNT)){
							image_context.color_index = BLUE_IDX;
#ifdef TUNE
							set_leds(BLUE_IDX);
#endif
						}
					}
				}
			}
		}

		if ((image_context.max_blue - image_context.max_red > 2) && (image_context.max_blue - image_context.max_green > 2)){
			image_context.color_index = BLUE_IDX;
#ifdef TUNE
			set_leds(BLUE_IDX);
#endif
		}

		else {
			if ((image_context.max_blue - image_context.max_red < 2) && (image_context.max_blue - image_context.max_green > 2)){
				if ((image_context.mean_blue > image_context.mean_red) && (image_context.count_blue > MIN_COUNT)){
					image_context.color_index = BLUE_IDX;
#ifdef TUNE
					set_leds(BLUE_IDX);
#endif
				}
				else {
					if ((image_context.mean_blue < image_context.mean_red) && (image_context.count_red > MIN_COUNT)){
						image_context.color_index = RED_IDX;
#ifdef TUNE
						set_leds(RED_IDX);
#endif
					}
				}
			}
			else {
				if ((image_context.max_blue - image_context.max_green < 2) && (image_context.max_blue - image_context.max_red > 2)){
					if ((image_context.mean_blue > image_context.mean_green) && (image_context.count_blue > MIN_COUNT)){
						image_context.color_index = BLUE_IDX;
#ifdef TUNE
						set_leds(BLUE_IDX);
#endif
					}
					else {
						if ((image_context.mean_green > image_context.mean_blue) && (image_context.count_green > MIN_COUNT)){
							image_context.color_index = GREEN_IDX;
#ifdef TUNE
							set_leds(GREEN_IDX);
#endif
						}
					}
				}
			}
		}

	}


	if(image_context.send_data){
		chprintf((BaseSequentialStream *)&SD3, "%R Max =%-7d G Max =%-7d B Max =%-7d \r\n\n",
				image_context.max_red, image_context.max_green, image_context.max_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Mean =%-7d G Mean =%-7d B Mean =%-7d \r\n\n",
				image_context.mean_red, image_context.mean_green, image_context.mean_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Count =%-7d G Count =%-7d B Count =%-7d \r\n\n",
				image_context.count_red, image_context.count_green, image_context.count_blue);
	}

}

void find_color(void){
	switch(image_context.detection){
	case MAX_ONLY:
		find_color_max();
		break;
	case MEAN_ONLY:
		find_color_mean();
		break;
	case MAX_N_MEAN:
		find_color_max_n_mean();
		break;
	case MAX_N_COUNT:
			find_color_max_n_count();
			break;
	case RAINY_DAY:
			fin_color_rainy_day();
			break;
	default:
		find_color_max();
		break;
	}
}

