#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//Comment after tuning the color detection parameters in read_image.c
#define TUNE

//constants for the differents parts of the project
#define PI                 			3.1415926536f

#define IMAGE_BUFFER_SIZE			640
#define ROTATION_THRESHOLD			10
#define ROTATION_COEFF				2
#define LINEWIDTH					100		//en fonction de la ligne imprimmer, a voir si utile
#define TOLERANCE					3

//PID Parameters
#define ERROR_THRESHOLD				0.1f	//[cm] because of the noise of the camera
#define KP							100.0f
#define KI 							3.5f	//must not be zero
#define KD							0.0f	//a tuner -> utiliser deuxieme methode de ZN avec Ku et Pu
#define MAX_SUM_ERROR 				(MOTOR_SPEED_LIMIT/KI)

//Distances parameters
#define WHEEL_PERIMETER     		13 		//[cm]
#define NSTEP_ONE_TURN      		1000	// number of step for 1 turn of the motor
#define WHEEL_DISTANCE      		5.30f    //cm
#define PERIMETER_EPUCK     		(PI * WHEEL_DISTANCE)

//Threshold des IR
#define	IR_THRESHOLD				250

//Vertical index of line (0 to 480) 0 : highest, 479 :lowest (due to camera library we take two lines)
#define LINE_INDEX_TOP				10
#define LINE_INDEX_BOT				400
#define TOP							0
#define BOTTOM						1

//Contrast of camera
#define CONTRAST					85

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

//Level des leds
#define LED_ON						10
#define LED_OFF						0


typedef enum {
	MAX_ONLY = 0,
	MEAN_ONLY,
	MAX_N_MEAN
} detect_mode_t;

typedef enum {
	NO_COLOR = 0,
	RED_IDX,
	GREEN_IDX,
	BLUE_IDX,
	YELLOW_IDX,
	PURPLE_IDX
} color_index_t;

typedef enum {
	SENSOR_IR1 = 1,
	SENSOR_IR2,
	SENSOR_IR3,
	SENSOR_IR4,
	SENSOR_IR5,
	SENSOR_IR6
} ir_sensors_index_t;

typedef enum {
	LED_RGB_2 = 0,
	LED_RGB_4,
	LED_RGB_6,
	LED_RGB_8
} rgb_leds_index_t;

typedef enum {
	NO_VISUALIZE_PARAMS = 0,
	YES_VISUALIZE_PARAMS,
} visualize_mode_t;

struct tunning_config {
	uint8_t contrast;
	uint16_t line_idx;
	detect_mode_t detection_mode;
	color_index_t color_idx;
	visualize_mode_t send_data_terminal;
};


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
