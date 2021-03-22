#include <stm32f4xx.h>
#include <system_clock_config.h>
#include <gpio.h>
#include <main.h>
#include <timer.h>

// Init function required by __libc_init_array
void _init(void) {}

void delay(unsigned int n) //fonction pour faire blink led
{
	while (n--) {
		__asm__ volatile ("nop");
	}
 }

/***************** LED SEQUENCES *****************/
// LEDs sequences with order LED7, LED5, LED3, LED1
static const uint8_t seq1[8][4] = {
	{0, 0, 0, 1}, // ON1
	{0, 0, 0, 0}, // OFF1
	{0, 0, 1, 0}, // ON3
	{0, 0, 0, 0}, // OFF3
	{0, 1, 0, 0}, // ON5
	{0, 0, 0, 0}, // OFF5
	{1, 0, 0, 0}, // ON7
	{0, 0, 0, 0}, // OFF7
};

static const uint8_t seq2[8][4] = {
	{0, 0, 0, 1}, // ON1
	{0, 0, 1, 1}, // ON3
	{0, 1, 1, 1}, // ON5
	{1, 1, 1, 1}, // ON7
	{1, 1, 1, 0}, // OFF1
	{1, 1, 0, 0}, // OFF3
	{1, 0, 0, 0}, // OFF5
	{0, 0, 0, 0}, // OFF7
};

static const uint8_t seq5[8][4] = {
	{0, 0, 0, 0}, // ON1
	{0, 1, 0, 1}, // ON3
	{0, 0, 0, 0}, // ON5
	{1, 0, 1, 0}, // ON7
	{0, 0, 0, 0}, // OFF1
	{0, 1, 0, 1}, // OFF3
	{0, 0, 0, 0}, // OFF5
	{1, 0, 1, 0}, // OFF7
};

static const uint8_t seq6[8][4] = {
	{0, 0, 0, 0}, // ON1
	{0, 0, 1, 1}, // ON3
	{0, 0, 0, 0}, // ON5
	{1, 1, 0, 0}, // ON7
	{0, 0, 0, 0}, // OFF1
	{0, 0, 1, 1}, // OFF3
	{0, 0, 0, 0}, // OFF5
	{1, 1, 0, 0}, // OFF7
};

static const uint8_t seq4[8][6] = {
	{1, 1, 1, 1, 1, 1}, // ON
	{0, 0, 0, 0, 0, 0}, // OFF
	{1, 1, 1, 1, 1, 1}, // ON
	{0, 0, 0, 0, 0, 0}, // OFF
	{1, 1, 1, 1, 1, 1}, // ON
	{0, 0, 0, 0, 0, 0}, // OFF
	{1, 1, 1, 1, 1, 1}, // ON
	{0, 0, 0, 0, 0, 0}, // OFF
};

static const uint8_t seq3[8][4] = {
	{0, 0, 0, 1}, // ON1
	{0, 0, 1, 1}, // ON3
	{0, 1, 1, 1}, // ON5
	{1, 1, 1, 1}, // ON7
	{0, 1, 1, 1}, // OFF7
	{0, 0, 1, 1}, // OFF5
	{0, 0, 0, 1}, // OFF3
	{0, 0, 0, 0}, // OFF7
};

static const uint8_t seq7[8][4] = {
	{0, 0, 0, 1}, // ON1
	{1, 0, 0, 1}, // ON3
	{1, 1, 0, 1}, // ON5
	{1, 1, 1, 1}, // ON7
	{1, 1, 1, 0}, // OFF1
	{0, 1, 1, 0}, // OFF3
	{0, 0, 1, 0}, // OFF5
	{0, 0, 0, 0}, // OFF7
};


static const uint8_t seq8[16][6] = {
	{1, 1, 1, 1, 1, 1}, // ON
	{0, 0, 0, 0, 0, 0}, // OFF
	{1, 1, 0, 0, 0, 0}, // FRONT BODY
	{0, 0, 0, 0, 0, 0}, // OFF
	{0, 0, 1, 1, 1, 1}, // 5 7
	{0, 0, 0, 0, 0, 0}, // OFF
	{0, 0, 0, 1, 0, 1}, // 1 3
	{0, 0, 0, 0, 0, 0}, // OFF
	{0, 0, 1, 0, 1, 0},
	{0, 0, 0, 0, 0, 0}, // OFF
	{0, 0, 1, 0, 1, 1},
	{0, 0, 0, 1, 1, 1},
	{0, 0, 1, 1, 1, 0},
	{0, 0, 1, 1, 0, 1},
	{0, 0, 1, 0, 1, 1},
	{0, 0, 0, 0, 0, 0}, // OFF

};


static void LEDs_update(const uint8_t *out)
{
	/* LEDs */
	out[3] ? gpio_clear(LED1) : gpio_set(LED1);
	out[2] ? gpio_clear(LED3) : gpio_set(LED3);
	out[1] ? gpio_clear(LED5) : gpio_set(LED5);
	out[0] ? gpio_clear(LED7) : gpio_set(LED7);
}

static void LEDs_update2(const uint8_t *out)
{
	/* LEDs */
	out[5] ? gpio_clear(LED1) : gpio_set(LED1);
	out[4] ? gpio_clear(LED3) : gpio_set(LED3);
	out[3] ? gpio_clear(LED5) : gpio_set(LED5);
	out[2] ? gpio_clear(LED7) : gpio_set(LED7);
	out[1] ? gpio_set(FRONT_LED) : gpio_clear(FRONT_LED);
	out[0] ? gpio_set(BODY_LED) : gpio_clear(BODY_LED);
}


int main(void)
{



	SystemClock_Config();
	int selector, old_selector = 0;
	int sequence_pos = 0;
    // Enable GPIOD peripheral clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;
	// RCC_AHB1ENR_GPIOBEN = 0b10
	// RCC_AHB1ENR_GPIODEN = 0b1000

	timer7_start();//comment to use selector


    // LED used init
	gpio_config_output_opendrain(LED_USED);
	gpio_config_output_opendrain(LED1);
	gpio_config_output_opendrain(LED3);
	gpio_config_output_opendrain(LED5);
	gpio_config_output_pushpull(FRONT_LED);
	gpio_config_output_pushpull(BODY_LED);

	while(1){//main for timer interrupt //comment to use selector
		;
	}
//	gpio_set(FRONT_LED); //pour synchroniser les LED car front led s'allume a 1 et non a 0
//    gpio_set(BODY_LED);
	//GPIOD->BSRR = (1 << 11); //LED eteinte car on met un 1 sur la ligne
    //GPIOD->BSRR = (1 << 11+16);// LED allume car on met un 0 sur la ligne
	//gpio_clear(LED_USED);

//    init_selector(); //active les clocks des pins du selector et config les pins en entreer
//
//    while (1) {
//		delay(SystemCoreClock/32);
//		selector = get_selector();
//		if (selector != old_selector) {
//			sequence_pos = 0;
//			gpio_set(LED1);
//			gpio_set(LED3);
//			gpio_set(LED5);
//			gpio_set(LED7);
//		}
//		old_selector = selector;
//		switch (selector)
//		{
//			case 0: // All LEDs off
//				gpio_clear_front_body(BODY_LED,FRONT_LED);
//				gpio_set(LED1);
//				gpio_set(LED3);
//				gpio_set(LED5);
//				gpio_set(LED7);
//				break;
//			case 1:
//				gpio_clear_front_body(BODY_LED,FRONT_LED);
//				gpio_toggle(LED1);
//				break;
//			case 2:
//				gpio_clear_front_body(BODY_LED,FRONT_LED);
//				gpio_toggle(LED3);
//				break;
//			case 3:
//				gpio_clear_front_body(BODY_LED,FRONT_LED);
//				LEDs_update(seq5[sequence_pos]);
//				sequence_pos++;
//				sequence_pos %= 8;
//				break;
//			case 4:
//				gpio_clear_front_body(BODY_LED,FRONT_LED);
//				gpio_toggle(LED5);
//				break;
//			case 5:
//				gpio_clear(BODY_LED);
//				gpio_toggle(FRONT_LED);
//				break;
//			case 6:
//				gpio_clear(FRONT_LED);
//				gpio_toggle(BODY_LED);
//				break;
//			case 7:
//				gpio_clear_front_body(BODY_LED,FRONT_LED);
//				LEDs_update(seq6[sequence_pos]);
//				sequence_pos++;
//				sequence_pos %= 8;
//				break;
//			case 13:
//				gpio_toggle(BODY_LED);
//				gpio_toggle(FRONT_LED);
//				break;
//			case 8:
//				gpio_clear_front_body(BODY_LED,FRONT_LED);
//				gpio_toggle(LED7);
//				break;
//			case 9: // Use Sequence 1
//				gpio_clear_front_body(BODY_LED,FRONT_LED);
//				LEDs_update(seq1[sequence_pos]);
//				sequence_pos++;
//				sequence_pos %= 8;
//				break;
//			case 10: // Use Sequence 2
//				gpio_clear_front_body(BODY_LED,FRONT_LED);
//				LEDs_update(seq2[sequence_pos]);
//				sequence_pos++;
//				sequence_pos %= 8;
//				break;
//			case 11: // Use Sequence 3
//				gpio_clear_front_body(BODY_LED,FRONT_LED);
//				LEDs_update(seq3[sequence_pos]);
//				sequence_pos++;
//				sequence_pos %= 8;
//				break;
//			case 12:
//				gpio_clear_front_body(BODY_LED,FRONT_LED);
//				LEDs_update2(seq4[sequence_pos]);
//				sequence_pos++;
//				sequence_pos %= 8;
//				break;
//			case 14:
//				gpio_clear_front_body(BODY_LED,FRONT_LED);
//				LEDs_update(seq7[sequence_pos]);
//				sequence_pos++;
//				sequence_pos %= 8;
//				break;
//			case 15:
//				gpio_clear_front_body(BODY_LED,FRONT_LED);
//				LEDs_update2(seq8[sequence_pos]);
//				sequence_pos++;
//				sequence_pos %= 16; // sequence_pos = sequence_pos % 16 // % : operateur modulo / passe de 8 a 16 car cette sequence est plus longue
//				break;
//
//
//			default:
//			gpio_clear_front_body(BODY_LED,FRONT_LED);
//			break;
//		}
//	}



}

//	while (1) { //toutes les led clignote de facon synchrone
//		delay(SystemCoreClock/32);//SystemCoreClock = 16MHz
//		gpio_toggle(LED7);
//		gpio_toggle(LED1);
//		gpio_toggle(LED3);
//		gpio_toggle(LED5);
//		gpio_toggle(FRONT_LED);
//		gpio_toggle(BODY_LED);
//    }
