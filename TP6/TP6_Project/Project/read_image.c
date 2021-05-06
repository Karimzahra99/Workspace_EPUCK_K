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
#define DEAD_ZONE_WIDTH				100

//Shift pour remettre les bits des couleurs dans l'ordre lors de l'extraction du format RGB565
#define SHIFT_2						2
#define SHIFT_3						3
#define SHIFT_6						6



//After tuning adjust to the desired detection mode
static detect_mode_t detection = MAX_ONLY;
//To visualize maxs, means and counts for each color
static visualize_mode_t send_data = NO_VISUALIZE_PARAMS;


static color_index_t color_idx = 0;
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

	//dereferencing structure from void *
	struct tunning_config *tune = (struct tunning_config *)arg;
	uint8_t contr = tune->contrast;
	uint16_t idx = tune->line_idx;

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

	uint8_t *img_buff_ptr;

	bool send_to_computer = true;

	color_index_t color_index = *((color_index_t *)arg);

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

		//To visualize one image on computer with plotImage.py
		if(send_to_computer){
			//sends to the computer the image
			if (color_index == RED_IDX) SendUint8ToComputer(image_red, IMAGE_BUFFER_SIZE);
			if (color_index == GREEN_IDX) SendUint8ToComputer(image_green, IMAGE_BUFFER_SIZE);
			if (color_index == BLUE_IDX) SendUint8ToComputer(image_blue, IMAGE_BUFFER_SIZE);
		}

		//invert the bool
		send_to_computer = !send_to_computer;

	}

}

void tune_image_start(struct tunning_config arg_tune_settings){
	chThdCreateStatic(waTuneProcessImage, sizeof(waTuneProcessImage), NORMALPRIO, TuneProcessImage, &arg_tune_settings);
	chThdCreateStatic(waTuneCaptureImage, sizeof(waTuneCaptureImage), NORMALPRIO, TuneCaptureImage, &(arg_tune_settings.color_idx));
	send_data = arg_tune_settings.send_data_terminal;
	detection = arg_tune_settings.detection_mode;
}

#else
//Uncomment to use plot_image.py for debug
//#define PLOT_ON_COMPUTER

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);
static BSEMAPHORE_DECL(image_ready_sem_2, TRUE);

void calc_line_middle(uint8_t alternator);
uint8_t filter_noise_single(uint8_t couleur);

static uint8_t image_bot[IMAGE_BUFFER_SIZE] = {0};
static uint16_t middle_line_top = IMAGE_BUFFER_SIZE/2; //middle of line
static uint16_t middle_line_bot = IMAGE_BUFFER_SIZE/2;


uint16_t calc_middle(uint8_t *buffer){

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
		if (color_idx == RED_IDX){
			middle_line_top = calc_middle(image_red);
		}
		else {
			if (color_idx == GREEN_IDX){
				middle_line_top = calc_middle(image_green);
			}

			else {
				if (color_idx == BLUE_IDX){
					middle_line_top = calc_middle(image_blue);
				}
			}
		}

	}
	else {
		middle_line_bot = calc_middle(image_bot);
	}
}

uint8_t filter_noise_single(uint8_t couleur){
	if (couleur > threshold_color){
		return couleur;
	}
	else return 0;
}

int16_t get_middle_diff(void) {
	return middle_line_top-middle_line_bot;
}

uint16_t get_middle_top(void) {
	return middle_line_top;
}

uint16_t get_middle_bot(void) {
	return middle_line_bot;
}

static THD_WORKING_AREA(waCaptureImage, 512);
static THD_FUNCTION(CaptureImage, arg) {

	chRegSetThreadName(__FUNCTION__);

	(void)arg;

	while(1){

		//Line index 413 detecting colors goes wrong
		//po8030_advanced_config(FORMAT_RGB565, 0, 413, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);

		po8030_advanced_config(FORMAT_RGB565, 0, LINE_INDEX_TOP, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
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

		po8030_advanced_config(FORMAT_RGB565, 0, LINE_INDEX_BOT, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
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
			if (color_idx == RED_IDX){
				c = ((uint8_t)img_buff_ptr[i]&0xF8) >> SHIFT_3;
			}
			else {
				if (color_idx == GREEN_IDX){
					c = (((uint8_t)img_buff_ptr[i]&0x07) << SHIFT_2) + (((uint8_t)img_buff_ptr[i+1]&0xC0) >> SHIFT_6);
				}
				else {
					if (color_idx == BLUE_IDX){
						c = (uint8_t)img_buff_ptr[i+1]&0x1F;
					}
				}
			}
			image_bot[i/2] = filter_noise_single(c);
		}
		calc_line_middle(BOTTOM);

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

void read_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);

}

#endif


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

	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; ++i){

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


	mean_red = temp_r / count_r;
	mean_green = temp_g / count_g;
	mean_blue = temp_b / count_b;

	max_red = max_r;
	max_green = max_g;
	max_blue = max_b;

}

void max_count(void){

	uint16_t count_r = 0;
	uint16_t count_g = 0;
	uint16_t count_b = 0;

	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; ++i){
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

uint8_t get_color(void){
	return color_idx;
}

void find_color_mean(void){

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
				color_idx = NO_COLOR;
			}
		}
	}

	if(send_data){
		chprintf((BaseSequentialStream *)&SD3, "%R Max =%-7d G Max =%-7d B Max =%-7d \r\n\n",
				max_red, max_green, max_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Mean =%-7d G Mean =%-7d B Mean =%-7d \r\n\n",
				mean_red, mean_green, mean_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Count =%-7d G Count =%-7d B Count =%-7d \r\n\n",
				count_red, count_green, count_blue);
	}

}

void find_color_max(void){

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

	if(send_data){
		chprintf((BaseSequentialStream *)&SD3, "%R Max =%-7d G Max =%-7d B Max =%-7d \r\n\n",
				max_red, max_green, max_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Mean =%-7d G Mean =%-7d B Mean =%-7d \r\n\n",
				mean_red, mean_green, mean_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Count =%-7d G Count =%-7d B Count =%-7d \r\n\n",
				count_red, count_green, count_blue);
	}
}


void find_color_max_n_mean(void){

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
	if(send_data){
		chprintf((BaseSequentialStream *)&SD3, "%R Max =%-7d G Max =%-7d B Max =%-7d \r\n\n",
				max_red, max_green, max_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Mean =%-7d G Mean =%-7d B Mean =%-7d \r\n\n",
				mean_red, mean_green, mean_blue);

		chprintf((BaseSequentialStream *)&SD3, "%R Count =%-7d G Count =%-7d B Count =%-7d \r\n\n",
				count_red, count_green, count_blue);
	}

}

void find_color(void){
	switch(detection){
	case MAX_ONLY:
		find_color_max();
		break;
	case MEAN_ONLY:
		find_color_mean();
		break;
	case MAX_N_MEAN:
		find_color_max_n_mean();
		break;
	default:
		find_color_max();
		break;
	}
}

