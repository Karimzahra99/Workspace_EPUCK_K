#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <string.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>
#include <selector.h>

static float distance_cm = 0;
static uint16_t line_position = IMAGE_BUFFER_SIZE/2; //middle

static uint8_t threshold_green = 61;
static uint8_t threshold_red_blue  = 30;

static uint16_t count_blue = 0;
static uint16_t count_red  = 0;
static uint16_t count_green  = 0;

static uint8_t max_blue = 0;
static uint8_t max_red  = 0;
static uint8_t max_green  = 0;

static uint8_t image_red[IMAGE_BUFFER_SIZE] = {0};
static uint8_t image_green[IMAGE_BUFFER_SIZE] = {0};
static uint8_t image_blue[IMAGE_BUFFER_SIZE] = {0};

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
* Returns the line's width extracted from the image buffer given
* Returns 0 if line not found
*/
uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	static uint16_t last_width = PXTOCM/GOAL_DISTANCE;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){ //on avait mit 32 a check
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{
			//the slope must at least be WIDTH_SLOPE wide and is compared
			//to the mean of the image
			if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
			{
				begin = i;
				stop = 1;
			}
			i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
			stop = 0;

			while(stop == 0 && i < IMAGE_BUFFER_SIZE)
			{
				if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
				{
					end = i;
					stop = 1;
				}
				i++;
			}
			//if an end was not found
			if (i > IMAGE_BUFFER_SIZE || !end)
			{
				line_not_found = 1;
			}
		}
		else//if no begin was found
		{
			line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		width = last_width;
	}else{
		last_width = width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}

	//sets a maximum width or returns the measured width
	if((PXTOCM/width) > MAX_DISTANCE){
		return PXTOCM/MAX_DISTANCE;
	}else{
		return width;
	}
}


static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    //chThdSleepMilliseconds(12);

    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line LINE_INDEX + LINE_INDEX+1 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, LINE_INDEX, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	//systime_t time;//utiliser pour calculer le temps d'execution

	while(1){

    	//time = chVTGetSystemTime();

    	//starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
		//chThdSleepMilliseconds(12);
		//chprintf((BaseSequentialStream *)&SDU1, "capture time = %d\n", chVTGetSystemTime()-time);
    }

}

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {


	chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint16_t lineWidth = 0;

	bool send_to_computer = true;


    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		int selector_value = get_selector();
		switch (selector_value)
		{
			case 0:
				threshold_red_blue = 8;
				threshold_green = 16;
				break;
			case 1:
				threshold_red_blue = 10;
				threshold_green = 19;
				break;
			case 2:
				threshold_red_blue = 11;
				threshold_green = 23;
				break;
			case 3:
				threshold_red_blue = 13;
				threshold_green = 26;
				break;
			case 4:
				threshold_red_blue = 14;
				threshold_green = 29;
				break;
			case 5:
				threshold_red_blue = 16;
				threshold_green = 32;
				break;
			case 6:
				threshold_red_blue = 18;
				threshold_green = 35;
				break;
			case 7:
				threshold_red_blue = 19;
				threshold_green = 38;
				break;
			case 8:
				threshold_red_blue = 21;
				threshold_green = 41;
				break;
			case 9:
				threshold_red_blue = 22;
				threshold_green = 45;
				break;
			case 10:
				threshold_red_blue = 24;
				threshold_green = 48;
				break;
			case 11:
				threshold_red_blue = 25;
				threshold_green = 50;
				break;
			case 12:
				threshold_red_blue = 27;
				threshold_green = 54;
				break;
			case 13:
				threshold_red_blue = 28;
				threshold_green = 57;
				break;
			case 14:
				threshold_red_blue = 29;
				threshold_green = 60;
				break;
			case 15:
				threshold_red_blue = 30;
				threshold_green = 61;
				break;
			default:
				threshold_red_blue = 24;
				threshold_green = 45;
				break;

		}

		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){ //RRRRRGGG (<-img_buff_ptr[i])	 GGGBBBBB (<-img_buff_ptr[i+1])
			// R 0 a 31
			// B 0 a 31
			// V 0 a 63

			//extracting red 5 bits and shifting them right
			uint8_t r = ((uint8_t)img_buff_ptr[i]&0xF8) >> SHIFT_3;

			//extracting green 6 bits and rearranging their order
			uint8_t g = (((uint8_t)img_buff_ptr[i]&0x07) << SHIFT_3) + (((uint8_t)img_buff_ptr[i+1]&0xE0) >> SHIFT_5);

			//extracting blue 5 bits
			uint8_t b = (uint8_t)img_buff_ptr[i+1]&0x1F;

			//filtering noise for each color
			if (r > threshold_red_blue){
				image_red[i/2] = r;
			}
			else image_red[i/2] = 0;
			if (b > threshold_red_blue){
				image_blue[i/2] = b;
			}
			else image_blue[i/2] = 0;

			if (g >threshold_green){
				image_green[i/2] = g;
			}
			else image_green[i/2] = 0;


			check_threshold();
			calc_max();
			uint8_t color_idx = get_color();

			//clear array if condition is not satisfied
//			if (count_red < MIN_COUNT){
//				memset(image_red, 0, sizeof(image_red));
//			}
//			if (count_green < MIN_COUNT){
//				memset(image_green, 0, sizeof(image_green));
//			}
//			if (count_blue < MIN_COUNT){
//				memset(image_blue, 0, sizeof(image_blue));
//			}

//			set_rgb_led(LED4, 255, 0, 0);
//			set_rgb_led(LED2, 0, 255, 0);
//			set_rgb_led(LED8, 0, 0, 255);

//			if (r > 130){
//				image_red[i/2]=r;
//
//			}else image_red[i/2] = 0;
//
//			if (g > 130){
//				image_green[i/2]=g;
//
//			}else image_green[i/2] = 0;
//
//			if (b > 130){
//				image_blue[i/2]=b;
//
//			}else image_blue[i/2] = 0;
//
//			if (image_red[i/2]>image_green[i/2] && image_red[i/2]>image_blue[i/2]){
//				palWritePad(GPIOD, GPIOD_LED1, 0);//led on
//				palWritePad(GPIOD, GPIOD_LED3, 1);//led off
//				palWritePad(GPIOD, GPIOD_LED5, 1);//led off
//
//			}else
//
//			if (image_green[i/2]>image_blue[i/2] && image_green[i/2]>image_red[i/2]){
//				palWritePad(GPIOD, GPIOD_LED1, 1);//led off
//				palWritePad(GPIOD, GPIOD_LED3, 0);//led on
//				palWritePad(GPIOD, GPIOD_LED5, 1);//led off
//			}else
//
//			if (image_blue[i/2]>image_green[i/2] && image_blue[i/2]>image_red[i/2]){
//				palWritePad(GPIOD, GPIOD_LED1, 1);//led off
//				palWritePad(GPIOD, GPIOD_LED3, 1);//led off
//				palWritePad(GPIOD, GPIOD_LED5, 0);//led on
//			}

//		chprintf((BaseSequentialStream *)&SDU1, "Red intensity = %d\n", image_red[i/2]);
//		chprintf((BaseSequentialStream *)&SDU1, "Green intensity = %d\n", image_green[i/2]);
//		chprintf((BaseSequentialStream *)&SDU1, "Blue intensity = %d\n", image_blue[i/2]);

		}


		//search for a line in the image and gets its width in pixels
		lineWidth = extract_line_width(image_green);

		//converts the width into a distance between the robot and the camera
		if(lineWidth){
			distance_cm = PXTOCM/lineWidth;
		}


		//To visualize one image on computer with plotImage.py
		if(send_to_computer){
		//sends to the computer the image
		SendUint8ToComputer(image_blue, IMAGE_BUFFER_SIZE);
		}

		//invert the bool
			send_to_computer = !send_to_computer;
		//chThdSleepMilliseconds(1000);
		}

}

float get_distance_cm(void){
	return distance_cm;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

uint16_t get_line_position(void){
	return line_position;
}

void check_threshold(void){
	count_red = 0;
	for (uint16_t i =0; i < IMAGE_BUFFER_SIZE; i++){
		if(image_red[i] > threshold_red_blue){
			count_red ++;
		}
	}

	count_green = 0;
	for (uint16_t i =0; i < IMAGE_BUFFER_SIZE; i++){
		if(image_green[i] > threshold_green){
			count_green ++;
		}
	}

	count_blue = 0;
	for (uint16_t i =0; i < IMAGE_BUFFER_SIZE; i++){
		if(image_blue[i] > threshold_red_blue){
			count_blue ++;
		}
	}

}

void calc_max(void){

	uint8_t max = 0;
	for (uint16_t i = 0; i <  IMAGE_BUFFER_SIZE; i++){
		if(image_red[i]> max){
			max = image_red[i];
		}
	}
	max_red = max;

	max = 0;
	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++){
		if(image_blue[i]> max){
			max = image_blue[i];
		}
	}
	max_blue = max;

	max = 0;
	for (uint16_t i = 0; i < IMAGE_BUFFER_SIZE; i++){
		if(image_green[i]> max){
			max = image_green[i];
		}
	}
	max_green = max;

}

uint8_t get_color(void){
	uint8_t idx = 0;

//	chprintf((BaseSequentialStream *)&SD3, "%R Max =%-7d G Max =%-7d B Max=%-7d Red Count =%-7d Green Count=%-7d Blue Count =%-7d\r\n\n",
//	              max_red,max_green,max_blue,count_red,count_green,count_blue);
//Add some conditions to find the color off the line

	return idx;
}
