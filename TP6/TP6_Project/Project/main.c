#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <spi_comm.h>
#include <pid_regulator.h>
#include <process_image.h>



static 	int ir_left_ancien =0;
static 	int ir_avant_ancien =0;
static 	float ir_left_nouvau =0;
static 	float ir_avant_nouvau =0;
//Functions for communication and visualization
void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}
static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //Initialisation bus
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	// Init the peripherals.
	clear_leds();
	set_body_led(0);
	set_front_led(0);

    //starts the serial communication / can be removed if communication not needed
    serial_start();
    //start the USB communication / can be removed if communication not needed
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	//inits the motors
	motors_init();
	//For RGB LEDS
	spi_comm_start();

	//stars the threads for the pid regulator and the processing of the image
//	pid_regulator_start();
//	process_image_start();

	proximity_start();
	rotate_until_irmax();

    /* Infinite loop. */
    while (1) {
    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}


void rotate_until_irmax()
{
//	if ()
		do{
			ir_left_ancien = get_prox(Sensor_IR2);
			ir_avant_ancien = get_prox(Sensor_IR3);
			motor_set_position(PERIMETER_EPUCK/8, PERIMETER_EPUCK/8,  -speedcms_to_speedsteps(1), speedcms_to_speedsteps(1));
			ir_left_nouvau = get_prox(Sensor_IR2);
			ir_avant_nouvau = get_prox(Sensor_IR3);
			chprintf((BaseSequentialStream *)&SD3, "ir2_ancien =%-7d ir3_ancien =%-7d \r\n\n",
					ir_left_ancien, ir_avant_ancien);
		}
		while (1);//ir_left_nouvau > ir_left_ancien + 10 || ir_avant_nouvau < ir_avant_ancien - 10 );
}
