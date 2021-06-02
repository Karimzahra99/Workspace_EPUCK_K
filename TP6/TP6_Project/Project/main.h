#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define ERROR_THRESHOLD			3.0f	//[cm] because of the noise of the camera

// SPEED = 500
#define KP_L						1.5f
//SPEED = 800
#define KP_M						2.0f
//SPEED = 1100
#define KP_H						3.6f
#define KD_H						36.0f


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
