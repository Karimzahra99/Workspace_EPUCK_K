#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <main.h>
#include <timer.h>
#include <motor.h>
#include <selector.h>
#include <string.h> //added

#define ECART_ROUE 5.3f
#define PI                  3.1415926536f //f a la fin pour float
//TO ADJUST IF NECESSARY. NOT ALL THE E-PUCK2 HAVE EXACTLY THE SAME WHEEL DISTANCE
#define WHEEL_DISTANCE      5.35f    //cm
#define PERIMETER_EPUCK     (PI * WHEEL_DISTANCE)

// Init function required by __libc_init_array
void _init(void) {}

// Simple delay function
void delay(unsigned int n)
{
    while (n--) {
        __asm__ volatile ("nop");
    }
}


int main(void)
{
    SystemClock_Config();

//    // Enable GPIOD and GPIOE peripheral clock
    RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;

    //config the GPIO for PWM (alternate function 2)
    gpio_config_output_af_pushpull(FRONT_LED, 2);

    //config the timer with a given duty_cycle
    timer4_PWM_start(0);

    gpio_config_output_pushpull(BODY_LED);

    init_selector();

    motor_init();
    //waits before moving to let us position the robot before it moves
    delay(SystemCoreClock/4);
    //Sequence du corriger :
    //move 20cm forward at 5cm/s
    //motor_set_position(5, 5, 5, 5);
//    while(motor_position_reached() != POSITION_REACHED);
//    delay(SystemCoreClock/16);
//    //clockwise rotation of 180°
//    motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, -5, 5);
//    while(motor_position_reached() != POSITION_REACHED);
//    delay(SystemCoreClock/8);
//    //move 20cm forward at 5cm/s
//    motor_set_position(20, 20, 5, 5);
//    while(motor_position_reached() != POSITION_REACHED);
//    delay(SystemCoreClock/8);
//    //counterclockwise rotation of 180°
//    motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, 5, -5);
//    while(motor_position_reached() != POSITION_REACHED);
//    delay(SystemCoreClock/8);
    //move 20cm forward at 5cm/s


    float R = 30;
    float v = 10;
     //trajectoire circulaire dans le sens horaire
//    float dist_g = (R-ECART_ROUE/2)*2*PI;
//    float dist_d = (R+ECART_ROUE/2)*2*PI;
//    float vd = v;
//    float vg = dist_g*vd/dist_d;
    ////motor_set_position(d2, d1, v2, v1);//position right/position left/speed right/speed left
//    motor_set_position(dist_d,dist_g,vd,vg);
//    while(motor_position_reached() != POSITION_REACHED);

    //mov_infinity(v,R); //attention boucle infini dans la fonction

    mov_square(v,R,"ne");
    delay(SystemCoreClock/32);
    mov_square(v,R,"nw");
    delay(SystemCoreClock/32);
    mov_square(v,R,"se");
    delay(SystemCoreClock/32);
    mov_square(v,R,"sw");
    delay(SystemCoreClock/8);
    mov_circ_right(v,R,2*PI,0);
    delay(SystemCoreClock/32);
    mov_circ_right(v,R,2*PI,1);
    delay(SystemCoreClock/32);
    mov_circ_left(v,R,2*PI,0);
    delay(SystemCoreClock/32);
    mov_circ_left(v,R,2*PI,1);

    while (1) {
            delay(SystemCoreClock/32);
            gpio_toggle(BODY_LED);
            timer4_set_duty_cycle(get_selector() / (float)MAX_VALUE_SELECTOR);
        }

}

//vitesse en cm/s //rayon en cm //angle en radian //mode = 0 cercle par top //mode = 1 cercle par bot
void mov_circ_right(float vitesse,float rayon,float angle, int mode){
	float dg = (rayon+ECART_ROUE/2)*angle;
	float dd = (rayon-ECART_ROUE/2)*angle;
	float vg = vitesse;
	float vd = dd*vg/dg;
	if (mode == 1){
		vd = -vd;
		vg = -vg;
	}
	motor_set_position(dd,dg,vd,vg);
	while(motor_position_reached() != POSITION_REACHED);
	//delay(SystemCoreClock/8); 16000000/8 = 2000000 = 2s d'attente
	//temps d'attente conseille 0.5s ou 0.25s
}

void mov_circ_left(float vitesse,float rayon,float angle, int mode){
	float dg = (rayon-ECART_ROUE/2)*angle;
	float dd = (rayon+ECART_ROUE/2)*angle;
	float vd = vitesse;
	float vg = dg*vd/dd;
	if (mode == 1){
		vd = -vd;
		vg = -vg;
	}
	motor_set_position(dd,dg,vd,vg);
	while(motor_position_reached() != POSITION_REACHED);
}

void mov_infinity(float vitesse,float rayon){ //boucle infini attention
	while(1){
		mov_circ_right(vitesse,rayon,PI,0);
		mov_circ_left(vitesse,rayon,2*PI,0);
		mov_circ_right(vitesse,rayon,PI,0);
	}

}

//mode 0 = depart N/S & mode 1 = depart E/O
void mov_square(float vitesse,float cote, char location[2]){
	float vlin =  0;
	float vrotg = 0;
	float vrotd = 0;
	if ((!strcmp(location,"ne")) || ((!strcmp(location,"nw")))){
		vlin = vitesse;
	}
	else vlin = -vitesse;

	if ((!strcmp(location,"ne")) || ((!strcmp(location,"sw")))){
			vrotg = vitesse;
			vrotd = -vitesse;
		}
		else {
			vrotd = vitesse;
			vrotg = -vitesse;
		}

		motor_set_position(cote,cote,vlin,vlin);
		while(motor_position_reached() != POSITION_REACHED);

		motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, vrotd, vrotg);
		while(motor_position_reached() != POSITION_REACHED);

		motor_set_position(cote,cote,vlin,vlin);
		while(motor_position_reached() != POSITION_REACHED);

		motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, vrotd, vrotg);
		while(motor_position_reached() != POSITION_REACHED);

		motor_set_position(cote,cote,vlin,vlin);
		while(motor_position_reached() != POSITION_REACHED);

		motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, vrotd, vrotg);
		while(motor_position_reached() != POSITION_REACHED);

		motor_set_position(cote,cote,vlin,vlin);
		while(motor_position_reached() != POSITION_REACHED);

		motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, vrotd, vrotg);
		while(motor_position_reached() != POSITION_REACHED);

}

//motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, -vitesse, vitesse); CW 45deg
//motor_set_position(PERIMETER_EPUCK/4, PERIMETER_EPUCK/4, vitesse, -vitesse); CW -45deg
//motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, -vitesse, vitesse); CW 180deg
//motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, vitesse, -vitesse); CW -180deg
//motor_set_position(PERIMETER_EPUCK, PERIMETER_EPUCK, -vitesse, vitesse); CW 360deg
//motor_set_position(PERIMETER_EPUCK, PERIMETER_EPUCK, vitesse, -vitesse); CW -360deg
