#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

typedef enum {
	MAX_ONLY = 0,
	MEAN_ONLY,
	MAX_N_MEAN
} detect_mode_t;

struct tunning_config {
	uint8_t contrast;
	uint16_t line_idx;
	detect_mode_t detection_mode;
	bool send_data_terminal;
};


//constants for the differents parts of the project
#define PI                 			3.1415926536f

#define IMAGE_BUFFER_SIZE			640
#define ROTATION_THRESHOLD			10
#define ROTATION_COEFF				2
#define LINEWIDTH					100		//en fonction de la ligne imprimmer, a voir si utile
#define TOLERANCE					3
#define ERROR_THRESHOLD				0.1f	//[cm] because of the noise of the camera
#define KP							100.0f
#define KI 							3.5f	//must not be zero
#define KD							0.0f	//a tuner -> utiliser deuxieme methode de ZN avec Ku et Pu
#define MAX_SUM_ERROR 				(MOTOR_SPEED_LIMIT/KI)


#define WHEEL_PERIMETER     		13 		//[cm]
#define NSTEP_ONE_TURN      		1000	// number of step for 1 turn of the motor
#define WHEEL_DISTANCE      		5.30f    //cm
#define PERIMETER_EPUCK     		(PI * WHEEL_DISTANCE)

#define Sensor_IR1					1
#define Sensor_IR2					2
#define Sensor_IR3					3
#define Sensor_IR4					4
#define Sensor_IR5					5
#define Sensor_IR6					6
#define	IR_THRESHOLD				250

//Vertical index of line (0 to 480) 0 : highest, 479 :lowest (due to camera library we take two lines)
#define LINE_INDEX_TOP				10
#define LINE_INDEX_BOT				400
#define TOP							0
#define BOTTOM						1

//Le nombre minimum de pixel pour valider une detection de ligne pour une certaine couleur
#define MIN_COUNT					5

//Pour trouver le milieu de la ligne, condition sur largeur de ligne et "trous" dans une ligne
#define MIN_LINE_WIDTH				70
#define MIN_HOLE_WIDTH				20
#define DEAD_ZONE_WIDTH				100
//Shift pour remettre les bits des couleurs dans l'ordre lors de l'extraction du format RGB565
#define SHIFT_3						3
#define SHIFT_5						5

//Index des couleurs / utiliser enum ?
#define RED_IDX						1
#define GREEN_IDX					2
#define BLUE_IDX					3
#define YELLOW_IDX					4
#define PURPLE_IDX					5
#define NO_COLOR					0

#define LED_RGB_2					0
#define LED_RGB_4					1
#define LED_RGB_6					2
#define LED_RGB_8					3
#define LED_ON						10
#define LED_OFF						0


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
