#ifndef GPIO_H
#define GPIO_H

#include <stm32f407xx.h>

void gpio_config_output_opendrain(GPIO_TypeDef *port, unsigned int pin);
void gpio_set(GPIO_TypeDef *port, unsigned int pin);
void gpio_clear(GPIO_TypeDef *port, unsigned int pin);
void gpio_toggle(GPIO_TypeDef *port, unsigned int pin);
void gpio_config_output_pushpull(GPIO_TypeDef *port, unsigned int pin);
_Bool gpio_read(GPIO_TypeDef *port, unsigned int pin);
void gpio_config_input_pd(GPIO_TypeDef *port, unsigned int pin);
void gpio_clear_front_body(GPIO_TypeDef *port1, unsigned int pin1,GPIO_TypeDef *port2, unsigned int pin2);

#endif /* GPIO_H */
