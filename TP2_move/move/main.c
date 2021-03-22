#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <main.h>
#include <timer.h>
#include <motor.h>
#include <selector.h>

#define ECART_ROUE 5.25
#define PI                  3.1415926536f
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

    gpio_config_output_pushpull(BODY_LED);//?????

    init_selector();

    motor_init();
    //waits before moving to let us position the robot before it moves
    delay(SystemCoreClock/4);
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
    float dist = 17;
    float d1 = 2*PI*dist;
    float d2 = 2*PI*(dist-ECART_ROUE);
    float v1 = 10;
    float v2 = v1*d2/d1;
    motor_set_position(d2, d1, v2, v1);//position right/position left/speed right/speed left
    while(motor_position_reached() != POSITION_REACHED);
    delay(SystemCoreClock/8);
    motor_set_position(d2, d1, -v2, -v1);//position right/position left/speed right/speed left
    while(motor_position_reached() != POSITION_REACHED);
    delay(SystemCoreClock/8);
    motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, 5, -5);
    while(motor_position_reached() != POSITION_REACHED);
    delay(SystemCoreClock/8);
    motor_set_position(d1, d2, v1, v2);//position right/position left/speed right/speed left
    while(motor_position_reached() != POSITION_REACHED);
    delay(SystemCoreClock/8);
    motor_set_position(PERIMETER_EPUCK/2, PERIMETER_EPUCK/2, -5, 5);
	while(motor_position_reached() != POSITION_REACHED);
	delay(SystemCoreClock/8);

    while (1) {
            delay(SystemCoreClock/32);
            gpio_toggle(BODY_LED);
            timer4_set_duty_cycle(get_selector() / (float)MAX_VALUE_SELECTOR);
        }

}

