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

#include <pi_regulator.h>
#include <process_image.h>
#include <proximity.h>
#include <leds.h>
#include <spi_comm.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

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

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the camera
    dcmi_start();
	po8030_start();


	//to uncomment if needed
    //starts the serial communication
	//serial_start();
    //start the USB communication
    usb_start();

	//Initialize DAC : digital analog converter (needed to use microphones)
	dac_start();

	//For RGB LEDS
	//Initialize SPI : serial peripheral interface, to use RGB LEDs
	spi_comm_start();

	// bus initialization for proximity sensors
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //Starts the proximity sensors and calibrate them
	proximity_start();

	//Initialize speakers
	mic_start(NULL);

	//Initialize Melody thread to play songs
	playMelodyStart();

	//Initialize the motors
	motors_init();

	//starts the threads for the line following and the processing of the image
	follow_line_start();
	process_image_start();

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
