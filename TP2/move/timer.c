#include <stm32f4xx.h>
#include <gpio.h>
#include <main.h>

#define TIMER_CLOCK         84000000    // APB1 clock car le prescaler de APB1 est 4 donc 168/4 *2 <- mult par 2 car prescaler different de 1
#define PRESCALER_TIM4      2100        // timer frequency: 40kHz
#define COUNTER_MAX_TIM4    500       // timer max counter -> 80Hz = 40000/500 = 80 //Normalement c'est 500 mais 40000 ca donne 1Hz et on peut voir l'effet visuellement sans passer oscilloscope

#define PRESCALER_TIM7      8400        // timer frequency: 10kHz
#define COUNTER_MAX_TIM7    10000       // timer max counter -> 100Hz

void timer4_PWM_start(uint16_t valuePulse)
{
	if(valuePulse > COUNTER_MAX_TIM4){
		valuePulse = COUNTER_MAX_TIM4;
	}

	// enable TIM4 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //p231 RM //0x0000004 == 0b100

	// configure TIM4 PSC and CNT
	TIM4->PSC = PRESCALER_TIM4 - 1; // Note: final timer clock = timer clock / (prescaler + 1)
	TIM4->ARR = COUNTER_MAX_TIM4 - 1; // Note: timer reload takes 1 cycle, thus -1

	// configure Output Compare
	TIM4->CCMR2 |=(0x6U << TIM_CCMR2_OC3M_Pos); //Output Compare 3 Mode PWM1 // bit 4 a 6 sont mit a 110 pour PWM Mode 1 //6U = 0b110 et TIM_CCMR2_OC3M_Pos = 4U donc on decale 0b110 de 4 et on obtient 0b1100000 on a bien 110 pour les bits 4 a 6 (on commence a compté par le bit0)
	TIM4->CCMR2 |= TIM_CCMR2_OC3PE; // Output Compare 3 Preload enable //le bit OC3PE est mit a 1 pour utiliser le PWM
	TIM4->CCR3 = valuePulse; //Value of the pulse (related to the Duty-Cycle) //la valeur de comparaison pour faire le duty cycle
	TIM4->CCER |= TIM_CCER_CC3E; //Output state enable //0x00000100 == 0b000100000000 //on met a 1 au bit8 (CC3E)

	//Enable Timer
	TIM4->CR1 |= TIM_CR1_CEN; // enable timer
}


//percentage between 0 and 1
void timer4_set_duty_cycle(float percentage)
{
	if(percentage > 1){
		percentage = 1;
	}else if(percentage < 0){
		percentage = 0;
	}
	TIM4->CCR3 = (uint16_t)(percentage * COUNTER_MAX_TIM4);//Value of the pulse (related to the Duty-Cycle)
}



void timer7_start(void)
{
    // Enable TIM7 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    // Enable TIM7 interrupt vector
    NVIC_EnableIRQ(TIM7_IRQn);

    // Configure TIM7
    TIM7->PSC = PRESCALER_TIM7 - 1;      // Note: final timer clock  = timer clock / (prescaler + 1)
    TIM7->ARR = COUNTER_MAX_TIM7 - 1;    // Note: timer reload takes 1 cycle, thus -1
    TIM7->DIER |= TIM_DIER_UIE;          // Enable update interrupt
    TIM7->CR1 |= TIM_CR1_CEN;            // Enable timer
}

/*
*   Commented because used for the motors
*/

// // Timer 7 Interrupt Service Routine
// void TIM7_IRQHandler(void)
// {
	/*
	*
	*   BEWARE !!
	*   Based on STM32F40x and STM32F41x Errata sheet - 2.1.13 Delay after an RCC peripheral clock enabling
	*
	*   As there can be a delay between the instruction of clearing of the IF (Interrupt Flag) of corresponding register (named here CR) and
	*   the effective peripheral IF clearing bit there is a risk to enter again in the interrupt if the clearing is done at the end of ISR.
	*
	*   As tested, only the workaround 3 is working well, then read back of CR must be done before leaving the ISR
	*
	*/

//     /* do something ... */
//     gpio_toggle(BODY_LED);

//     // Clear interrupt flag
//     TIM7->SR &= ~TIM_SR_UIF;
//     TIM7->SR;	// Read back in order to ensure the effective IF clearing
// }
