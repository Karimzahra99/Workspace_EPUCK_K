#ifndef MAIN_H
#define MAIN_H

/* LEDs that can be used in this TP1
  LED1 			: GPIOD pin 5
  LED3 			: GPIOD pin 6
  LED5 			: GPIOD pin 10
  LED7 			: GPIOD pin 11
  FRONT_LED 	: GPIOD pin 14
WARNING : Not on the same port !!
  BODY_LED		: GPIOB pin 2
*/
#define LED1     	GPIOD, 5
#define LED3     	GPIOD, 6
#define LED5     	GPIOD, 10
#define LED7     	GPIOD, 11
#define FRONT_LED	GPIOD, 14
#define BODY_LED	GPIOB, 2

void delay(unsigned int n);

void mov_circ_right(float vitesse,float rayon,float angle, int mode);
void mov_circ_left(float vitesse,float rayon,float angle, int mode);

void mov_infinity(float vitesse,float rayon);

void mov_square(float vitesse,float cote, char location[2]);



#endif /* MAIN_H_ */
