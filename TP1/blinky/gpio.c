#include <stm32f407xx.h>
#include <gpio.h>
#include <main.h>

void gpio_config_output_opendrain(GPIO_TypeDef *port, unsigned int pin)
{
    // Output type open-drain : OTy = 1
    port->OTYPER |= (1 << pin); //Or donc on veut set, masque 0000000100000
    //port->OTYPER &= ~(1 << pin); //works pour mettre en pushpull

    // Output data low : ODRy = 0
    port->ODR &= ~(1 << pin); //and donc on veut clear, masque 111111011111
    //port->ODR &= ~(0 << pin); // dosent work to turn off led, masque 11111111111, donc on clear aucun bit
    //port->ODR = 0b100000000000; // works to turn off led
    //port->ODR |= 1 << pin; // works to turn off led, masque 0000001000000

    // Floating, no pull-up/down : PUPDRy = 00
    port->PUPDR &= ~(3 << (pin * 2));

    // Changed my mind, lets put a pull-up resistor
    port->PUPDR = (port->PUPDR & ~(3 << (pin * 2))) | (1 << (pin * 2));
    //port->PUPDR |= (1 << (pin * 2)); //works to put pullup resistor
    //port->PUPDR |= (1 << 22); //works to put pullup resistor

    // Output speed highest : OSPEEDRy = 11
    port->OSPEEDR |= (3 << (pin * 2)); //or donc on veut set

    // Output mode : MODERy = 01
    port->MODER = (port->MODER & ~(3 << (pin * 2))) | (1 << (pin * 2));
    //port->MODER &= ~(3 << (pin * 2)); //remet les deux bits a 0
}

void gpio_set(GPIO_TypeDef *port, unsigned int pin)
{
    port->BSRR = (1 << pin);
}

void gpio_clear(GPIO_TypeDef *port, unsigned int pin)
{
    port->BSRR = (1 << (pin + 16));
}

void gpio_toggle(GPIO_TypeDef *port, unsigned int pin)
{
	if (port->ODR & (1<<pin)) { //retourne vrai si le bit est a 1
        gpio_clear(port, pin);
    } else {
    	gpio_set(port, pin);
    }
}

void gpio_config_output_pushpull(GPIO_TypeDef *port, unsigned int pin){
	// Output type pushpull : OTy = 0
	port->OTYPER &= ~(1 << pin);
	// Output data low : ODRy = 0
	port->ODR &= ~(1 << pin);

	// Floating, no pull-up/down : PUPDRy = 00
	port->PUPDR &= ~(3 << (pin * 2));
	// Output speed highest : OSPEEDRy = 11
	port->OSPEEDR |= (3 << (pin * 2));

	// Output mode : MODERy = 01
	port->MODER = (port->MODER & ~(3 << (pin * 2))) | (1 << (pin * 2));
}

bool gpio_read(GPIO_TypeDef *port, unsigned int pin)
{
return (port->IDR & (1<<pin));
}

void gpio_config_input_pd(GPIO_TypeDef *port, unsigned int pin){
	// Pull-down : PUPDRy = 10 / par securite
	port->PUPDR = (port->PUPDR & ~(3 << (pin * 2))) | (2 << (pin * 2));
	// Output speed highest : OSPEEDRy = 11
	port->OSPEEDR |= (3 << (pin * 2));
	// Input mode : MODERy = 00
	port->MODER &= ~(3 << (pin * 2));
}

void gpio_clear_front_body(GPIO_TypeDef *port1, unsigned int pin1,GPIO_TypeDef *port2, unsigned int pin2){
	gpio_clear(port1,pin1);
	gpio_clear(port2,pin2);
}

void gpio_config_output_PWM(GPIO_TypeDef *port, unsigned int pin)
{
	// Output mode : MODERy = 02, alternate function
    port->MODER = (port->MODER & ~(2 << (pin * 2))) | (2 << (pin * 2));

    // Alternate function AF2 (AFRH14 = 0010)
    port->AFR[1] |= (2 << ((pin - 8) * 4));

    // Output type open-drain : OTy = 1
    port->OTYPER &= (1 << pin);

    // Floating, no pull-up/down : PUPDRy = 00
    port->PUPDR &= ~(3 << (pin * 2));

	// Output mode : MODERy = 02, alternate function
    port->MODER = (port->MODER & ~(2 << (pin * 2))) | (2 << (pin * 2));
}

