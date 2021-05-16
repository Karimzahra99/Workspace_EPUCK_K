#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <spi_comm.h>
#include <moving.h>
#include <read_image.h>
#include <audio/play_melody.h>
#include <audio/microphone.h>
#include <audio/audio_thread.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
			115200,
			0,
			0,
			0,
	};
	// UART3
	sdStart(&SD3, &ser_cfg);
}

int main(void)
{
	//Initialize hardware abstraction layer
	halInit();

	//Initialize ChibiOs (RTOS : real time operating system)
	chSysInit();

	//Initialize mpu : memory protection unit
	mpu_init();

	//Initialize DAC : digital analog converter (needed to use microphones)
	dac_start();

	//Initialize message bus (needed for proximity thread)
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	//Clear all LEDs at start up
	clear_leds();
	set_body_led(0);
	set_front_led(0);

	//starts the serial communication / can be removed if bluetooth not needed
	serial_start();

	//start the USB communication / can be removed if communication not needed
	usb_start();

	//Initialize DCMI : digital camera interface
	dcmi_start();

	//Initialize camera PO8030D
	po8030_start();
	//For RGB LEDS
	//Initialize SPI : serial peripheral interface, to use RGB LEDs
	spi_comm_start();

	//Tuning / demo configuration, to switch between both modes just uncomment / comment "TUNE" in main.h
	//Tuning procedure at end of file

	/* Tuning parameters for camera :
	 * rgb_gains : [0 255] for each, default : 94, 64, 93
	 * contrast : [0 255], default 64
	 * Brightness : [-128 127], default 0, careful function of po8030.c takes an uint8_t but the format is the following [7]:[6:0] = Sign : Magnitude
	 * line_index_top : [0 489], above 413 image becomes noisy
	 * mode_detect : MAX_ONLY, MEAN_ONLY, MAX_N_MEANS, MAX_N_COUNT, RAINY_DAY, SUPER_RAINY_DAY, ULTRA_RAINY_DAY
	 * plot_pixels_color : RED_IDX, GREEND_IDX, BLUE_IDX
	 * send_params : NO_VISUALIZE_PARAMS, YES_VISUALIZE_PARAMS
	 */

	//Declaration of camera configuration
	rgb_gain_t rgb_gains = {94, 76, 90};
	uint8_t contrast = 75;
	uint8_t brightness = 40;
	uint16_t line_index_top = 10; //tuning uses line_index_top for plot visualization
	detect_mode_t mode_detect = MAX_ONLY;
	visualize_mode_t send_params = YES_VISUALIZE_PARAMS;
#ifdef TUNE
	color_index_t plot_pixels_color = RED_IDX; //chose which color intensity to plot with plot_image.py
#else
	uint16_t line_index_bot = 400;
#endif

#ifdef TUNE
	//Initialize camera with given configuration
	tuning_config_t tunning = {rgb_gains, contrast, brightness, line_index_top, mode_detect, plot_pixels_color, send_params};
	tune_image_start(tunning);
#else

	//Initialize motors
	motors_init();

	//3 seconds to remove hands and obstacles near the robot for IR calibration
	chThdSleepMilliseconds(3000);

	//Initialize IR sensors and calibrate them
	proximity_start();
	calibrate_ir();

	//Initialize camera with given configuration
	config_t config = {rgb_gains, contrast, brightness, line_index_top, line_index_bot, mode_detect, send_params};
	read_image_start(config);

	//Give some time to the camera to find a color if there is one
	chThdSleepMilliseconds(500);

	//Initialize speakers
	mic_start(NULL);

	//Initialize Melody thread to play songs
	playMelodyStart();

	//Initialize Moving thread to follow lines of different color and avoid obstacles
	moving_start();

#endif

	while (1) {
		//waits 1 second
//			chThdSleepMilliseconds(1000);

	}
}

//RGB LEDs setting function
void set_leds(color_index_t color_index){
	if (color_index == RED_IDX){ //red
		set_rgb_led(LED_RGB_2, LED_ON, LED_OFF, LED_OFF);
		set_rgb_led(LED_RGB_4, LED_ON, LED_OFF, LED_OFF);
		set_rgb_led(LED_RGB_6, LED_ON, LED_OFF, LED_OFF);
		set_rgb_led(LED_RGB_8, LED_ON, LED_OFF, LED_OFF);
		return;
	}
	else {
		if (color_index == GREEN_IDX){ //green
			set_rgb_led(LED_RGB_2, LED_OFF, LED_ON, LED_OFF);
			set_rgb_led(LED_RGB_4, LED_OFF, LED_ON, LED_OFF);
			set_rgb_led(LED_RGB_6, LED_OFF, LED_ON, LED_OFF);
			set_rgb_led(LED_RGB_8, LED_OFF, LED_ON, LED_OFF);
			return;
		}
		else {
			if (color_index == BLUE_IDX){ //blue
				set_rgb_led(LED_RGB_2, LED_OFF, LED_OFF, LED_ON);
				set_rgb_led(LED_RGB_4, LED_OFF, LED_OFF, LED_ON);
				set_rgb_led(LED_RGB_6, LED_OFF, LED_OFF, LED_ON);
				set_rgb_led(LED_RGB_8, LED_OFF, LED_OFF, LED_ON);
				return;
			}
			else {
				if (color_index == YELLOW_IDX){ //yellow
					set_rgb_led(LED_RGB_2, LED_ON, LED_ON, LED_OFF);
					set_rgb_led(LED_RGB_4, LED_ON, LED_ON, LED_OFF);
					set_rgb_led(LED_RGB_6, LED_ON, LED_ON, LED_OFF);
					set_rgb_led(LED_RGB_8, LED_ON, LED_ON, LED_OFF);
					return;
				}
				else {
					if (color_index == PURPLE_IDX){ //purple
						set_rgb_led(LED_RGB_2, LED_ON, LED_OFF, LED_ON);
						set_rgb_led(LED_RGB_4, LED_ON, LED_OFF, LED_ON);
						set_rgb_led(LED_RGB_6, LED_ON, LED_OFF, LED_ON);
						set_rgb_led(LED_RGB_8, LED_ON, LED_OFF, LED_ON);
						return;
					}
					else {
						if (color_index == NO_LINE){ //cyan
							set_rgb_led(LED_RGB_2, LED_OFF, LED_ON, LED_ON);
							set_rgb_led(LED_RGB_4, LED_ON, LED_OFF, LED_OFF);
							set_rgb_led(LED_RGB_6, LED_OFF, LED_ON, LED_ON);
							set_rgb_led(LED_RGB_8, LED_ON, LED_OFF, LED_OFF);
							return;
						}
						else {
							if (color_index == FIND_COLOR){ //blue and red
								set_rgb_led(LED_RGB_2, LED_ON, LED_OFF, LED_OFF);
								set_rgb_led(LED_RGB_4, LED_OFF, LED_OFF, LED_ON);
								set_rgb_led(LED_RGB_6, LED_ON, LED_OFF, LED_OFF);
								set_rgb_led(LED_RGB_8, LED_OFF, LED_OFF, LED_ON);
								return;
							}
							else {
								if (color_index == NO_COLOR){//LEDs off
									set_rgb_led(LED_RGB_2, LED_OFF, LED_OFF, LED_OFF);
									set_rgb_led(LED_RGB_4, LED_OFF, LED_OFF, LED_OFF);
									set_rgb_led(LED_RGB_6, LED_OFF, LED_OFF, LED_OFF);
									set_rgb_led(LED_RGB_8, LED_OFF, LED_OFF, LED_OFF);
									return;
								}
							}
						}
					}
				}
			}
		}
	}
}

//Security function : terminates the function in case of stack overflow
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
	chSysHalt("Stack smashing detected");
}

/* Tuning procedure :
 * 1) Start with default parameters, MAX_ONLY as color detection method and YES_VISUALIZE_PARAMETERS.
 * 2) Set Selector on position to 2 to have a noise threshold of 10
 * 3) Show to the camera lines of red, green and blue color on a black background and visualize with a serial terminal the max, mean and count values
 * 4) The goal is that the RGB component associated with the color of the line returns maximum and mean values ​​are bigger compared to the other two RGB components
 * 5) If the max values are low progressively increase the RGB gains
 * 6) If the RGB gains aren't sufficient, increase step by step the contrast up to 100 (more if needed) and each time progressively increase the RGB gains starting from their default values
 * 7) If the environment is dark, increase the brightness
 * 8) If for a specific line color, two RGB components have saturated maximum values but the correct RGB component has a higher mean, change the color detection to RAINY_DAY
 * 9) If for a specific line color, two RGB components have saturated maximum values but the correct RGB component has a higher mean and higher count, change the color detection to SUPER_RAINY_DAY
 * 10) If for a specific line color, two RGB components have saturated maximum values and similar mean values but the correct RGB component has a higher count value,
 *     change the color detection to ULTRA_RAINY_DAY
 * 11) Set set_params to NO_VISUALIZE_PARAMETERS and use plotImage.py to set appropriate threshold value to ensure that only the line appears and no background noise
 */
