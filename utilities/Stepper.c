#include "ch.h"
#include "Stepper.h"

void Setup_Stepper_PWM(void) {/* Note that this is hardcoded to timer8 chan2 - used to drive the stepper on the F4Discovery*/
	/* Clk and Reset tim8 */
	RCC->APB1ENR|=RCC_APB1ENR_TIM8EN;
	RCC->APB1RSTR|=RCC_APB1RSTR_TIM8;
	RCC->APB1RSTR&=~(RCC_APB1RSTR_TIM8RST);
	/* Configure the timers */
	//Channel 2
	TIM8->CR1|=TIM_CR1_ARPE;	//Preload enable
	TIM8->ARR=1000;			//Reload value sets the pwm period
	TIM8->CCMR1|=0x0030|TIM_CCMR1_OC2PE;//Use OC2 in Toggle mode with preload as the stepper signal
	TIM8->CCR2=1;			//OC2 Toggle - lowest possible value to still get toggling
	TIM8->CR1|=TIM_CR1_CEN;		//Start the timer - note it needs to have been clocked first, edit the chibios config file to ensure clocking
}

void GPIO_Stepper_Enable(uint8_t En) {
	palWritePad(PORTE, 10, En?PAL_HIGH:PAL_LOW);	
}

void GPIO_Stepper_Dir(uint8_t Dir) {
	palWritePad(PORTE, 13, Dir?PAL_HIGH:PAL_LOW);
}
