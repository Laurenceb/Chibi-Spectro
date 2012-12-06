#include "ch.h"
#include "hal.h"
#include "Stepper.h"
#include "Hardware_Conf.h"

void Setup_Stepper_PWM(void) {/* Note that this is hardcoded to timer1 chan2 - used to drive the stepper on the F4Discovery*/
	/* Clk and Reset tim8 */
	RCC->APB2ENR|=RCC_APB2ENR_TIM1EN;
	RCC->APB2RSTR|=RCC_APB2RSTR_TIM1RST;
	RCC->APB2RSTR&=~(RCC_APB2RSTR_TIM1RST);
	/* Configure the timers */
	//Channel 2
	TIM1->CR1|=TIM_CR1_ARPE;	//Preload enable
	TIM1->PSC=84;			//Gives 1MHz timer clock - gives a range from 0.15mm/s to 10m/s (~25x faster than motor)
	TIM1->ARR=5000;			//Reload value sets the pwm period - gives 100steps/second initialised
	TIM1->CCMR1|=0x0030|TIM_CCMR1_OC2PE;//Use OC2 in Toggle mode with preload as the stepper signal
	TIM1->CCR2=1;			//OC2 Toggle - lowest possible value to still get toggling
	TIM1->CR1|=TIM_CR1_CEN;		//Start the timer - note it needs to have been clocked first, edit the chibios config file to ensure clocking
}

void GPIO_Stepper_Enable(uint8_t En) {
	palWritePad( STEPPER_PORT, STEPPER_EN_PIN, En?PAL_LOW:PAL_HIGH);/* Note that Allegro uses inverted logic levels */	
}

void GPIO_Stepper_Dir(uint8_t Dir) {
	#ifdef STEPPER_INVERTED_DIR
		Dir=!Dir;
	#endif
	palWritePad( STEPPER_PORT, STEPPER_DIR_PIN, Dir?PAL_HIGH:PAL_LOW);
}
