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
	proximity_start();
	calibrate_ir();
//	process_image_start();

	//stars the threads for the pid regulator and the processing of the image
	pid_regulator_start();


//	while(1){
//	int ir_left_ancien = get_prox(Sensor_IR3);
//	chprintf((BaseSequentialStream *)&SD3, "ir3prox =%-7d \r\n\n ir3cprox =%-7d \r\n\n ir3amb =%-7d \r\n\n",
//			get_prox(Sensor_IR3), get_calibrated_prox(Sensor_IR3), get_ambient_light(Sensor_IR3) );
//	}
//	rotate_until_irmax();

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

