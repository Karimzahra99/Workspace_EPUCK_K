#include <stm32f4xx.h>
#include <gpio.h>
#include <main.h>

#define TIMER_CLOCK 84000000    //Dans SystemCoreConfig le prescaler de APB1 est 4 donc 168/4 = 42MHz et si le prescaler de APB1 est different de 1 alors pour les timers on multiplie cette frequence par 2
#define PRESCALER   8400    //p216 Datasheet if Prescaler different de 1 alors la clock est divise par 2*Prescaler donc 84000000/8400 = 10000
#define COUNTER_MAX 10000      //on creer une frequence de 1Hz

void timer7_start(void)
{
    // Enable TIM7 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    // Enable TIM7 interrupt vector
    NVIC_EnableIRQ(TIM7_IRQn);

    // Configure TIM7
    TIM7->PSC = PRESCALER - 1;      // Note: final timer clock  = timer clock / (prescaler + 1)
    TIM7->ARR = COUNTER_MAX - 1;	// Note: timer reload takes 1 cycle, thus -1
    TIM7->DIER |= TIM_DIER_UIE;  	// Enable update interrupt
    TIM7->CR1 |= TIM_CR1_CEN;    	// Enable timer
}

// Timer 7 Interrupt Service Routine
void TIM7_IRQHandler(void)
{
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

    /* do something ... */
	gpio_toggle(LED7);

    // Clear interrupt flag
    TIM7->SR &= ~TIM_SR_UIF;
    TIM7->SR;	// Read back in order to ensure the effective IF clearing
}
