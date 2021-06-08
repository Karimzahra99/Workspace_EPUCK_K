#include "follow_line.h"

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <process_image.h>
#include <proximity.h>
#include <leds.h>
#include <spi_comm.h>
#include <audio/play_melody.h>

// constants //

// LED constants
#define LED_ON					50
#define LED_OFF					0

// Proximity sensors constant
#define PROX_THRESH				200

// Regulator constants
#define LOW_SPEED					500 //step/s
#define MEDIUM_SPEED				800
#define HIGH_SPEED					MOTOR_SPEED_LIMIT
#define ERROR_THRESHOLD				3.0f	//the noise of the camera
#define KP_L						1.5f
#define KP_M						2.0f
#define KP_H						3.6f
#define KD_H						36.0f

//sensors names
typedef enum {
	SENSOR_IR1 = 1,
	SENSOR_IR2,
	SENSOR_IR3,
	SENSOR_IR4,
	SENSOR_IR5,
	SENSOR_IR6
} ir_sensors;

//speed states
typedef enum {
	LOW = 0,
	MEDIUM,
	STOP,
	HIGH
} STATE;

static STATE speed_context;

// detects proximity and change mode accordingly
void change_speed(void);

// set led colors for each speed case
void set_mode_color(void);

//simple PI regulator implementation
int16_t pd_regulator(void);

static THD_WORKING_AREA(wafollow_line, 256);
static THD_FUNCTION(follow_line, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();
        clear_leds();
        set_mode_color();

        switch(speed_context){
		case LOW :
        	speed = LOW_SPEED;
			break;

		case MEDIUM :
			speed = MEDIUM_SPEED;
			break;

		case HIGH :
			speed = HIGH_SPEED;
			break;

		case STOP :
        	speed = 0;
			break;
		default:
			speed = 0;
			break;
        }
        change_speed();
        speed_correction = pd_regulator();
		right_motor_set_speed(speed -  speed_correction);
		left_motor_set_speed(speed + speed_correction);
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void change_speed(void){
	if (get_prox(SENSOR_IR2) > PROX_THRESH && get_prox(SENSOR_IR5) > PROX_THRESH ){
		speed_context = STOP;
		playMelody(WE_ARE_THE_CHAMPIONS, ML_SIMPLE_PLAY, NULL);
	}
	else{
		if (get_prox(SENSOR_IR2) > PROX_THRESH){
			speed_context = LOW;
		}
		if (get_prox(SENSOR_IR5) > PROX_THRESH){
			speed_context = MEDIUM;
		}
		if (get_prox(SENSOR_IR3) > PROX_THRESH || get_prox(SENSOR_IR4) > PROX_THRESH){
			speed_context = HIGH;
		}
	}


}

void set_mode_color(void){

	if (speed_context == HIGH){
		set_rgb_led(LED2, LED_OFF, LED_ON, LED_OFF);
		set_rgb_led(LED4, LED_OFF, LED_ON, LED_OFF);
		set_rgb_led(LED6, LED_OFF, LED_ON, LED_OFF);
		set_rgb_led(LED8, LED_OFF, LED_ON, LED_OFF);
		set_led(LED5,LED_ON);
	}
	else if (speed_context == MEDIUM){
		set_rgb_led(LED2, LED_ON, LED_ON, LED_OFF);
		set_rgb_led(LED4, LED_ON, LED_ON, LED_OFF);
		set_rgb_led(LED6, LED_ON, LED_ON, LED_OFF);
		set_rgb_led(LED8, LED_ON, LED_ON, LED_OFF);
		set_led(LED7,LED_ON);
	}
	else if(speed_context == LOW){
		set_rgb_led(LED2, LED_ON, LED_OFF, LED_OFF);
		set_rgb_led(LED4, LED_ON, LED_OFF, LED_OFF);
		set_rgb_led(LED6, LED_ON, LED_OFF, LED_OFF);
		set_rgb_led(LED8, LED_ON, LED_OFF, LED_OFF);
		set_led(LED3,LED_ON);
	}
	else if(speed_context == STOP){
		set_led(LED7,LED_ON);
		set_led(LED3,LED_ON);

	}
}

int16_t pd_regulator(void){

	float error = 0;
	float speed = 0;
	float derivative = 0;
	static float old_error = 0;

	error = get_middle() - IMAGE_BUFFER_SIZE/2;


	//camera is little noisy
	if(fabs(error) < ERROR_THRESHOLD){
		//disables the PI regulator if error is too small
		return 0;
	}

	derivative = error - old_error ;

	old_error = error;
	if (speed_context==MEDIUM){
	speed = KP_M * error;
	}
	else if (speed_context==LOW){
	speed = KP_L * error;
	}
	else if (speed_context==HIGH){
	speed = KP_H * error + KD_H*derivative;
	}
    return (int16_t)speed;
}


void follow_line_start(void){
	speed_context = STOP;
	chThdCreateStatic(wafollow_line, sizeof(wafollow_line), NORMALPRIO, follow_line, NULL);
}

