#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
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
#include <tune_camera.h>
#include "read_image.h"


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

	sdStart(&SD3, &ser_cfg); // UART3.
}

// uncomment to tune the camera settings : detection method and contrast
#define TUNNING_MODE

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

#ifdef TUNNING_MODE
	//Contrast level, camera line index, detect_mode, send_data_to_computer
	struct tunning_config tunning = {85, 400, MEAN_ONLY, false};
	tune_image_start(tunning);
#else
	//inits the motors
	motors_init();
	//For RGB LEDS
	spi_comm_start();


	//stars the threads for the pid regulator and the processing of the image
	pid_regulator_start();

	read_image_start();

	proximity_start();
#endif

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

//Functions for communication and visualization
void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}
