#include <math.h>

#include "ch.h"
#include "hal.h"
#include "Stepper.h"
#include "Hardware_Conf.h"
#include "EKF_Pressure.h"

void Setup_Stepper_PWM(void) {/* Note that this is hardcoded to timer1 chan2 - used to drive the stepper on the F4Discovery*/
	/* Clk and Reset tim1 */
	RCC->APB2ENR|=RCC_APB2ENR_TIM1EN;
	RCC->APB2RSTR|=RCC_APB2RSTR_TIM1RST;
	RCC->APB2RSTR&=~(RCC_APB2RSTR_TIM1RST);
	/* Setup debug mode to disable timer in halt */
	DBGMCU->APB2FZ|=DBGMCU_APB1_FZ_DBG_TIM1_STOP;
	/* Configure the timers */
	//Channel 2
	TIM1->CR1|=TIM_CR1_ARPE;	//Preload enable
	TIM1->PSC=83;			//Gives 1MHz timer clock - gives a range from 0.15mm/s to 10m/s (~25x faster than motor)
	TIM1->ARR=4999;			//Reload value sets the pwm period - gives 100steps/second initialised
	TIM1->CCMR1|=0x3000|TIM_CCMR1_OC2PE;//Use OC2 in Toggle mode with preload as the stepper signal
	TIM1->CCR2=4998;		//OC2 Toggle - highest possible value to still get toggling
	TIM1->CCER|=TIM_CCER_CC2E;	//Enable the PWM output
	TIM1->BDTR|=TIM_BDTR_MOE;
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

//Note this should only be called from inside an ISR or where it cannot be interrupted
void Set_Stepper_Period(uint16_t period) {
	TIM1->CR1&=~TIM_CR1_ARPE;	//Disable the Preload
	TIM1->ARR = TIM1->CNT+2;	//Create a reload as soon as possible
	TIM1->CCR2 = period-1;		//Stepper signal flips at end of period, this is buffered and updates at timer overflow
	TIM1->CR1|=TIM_CR1_ARPE;	//Enable the Preload
	TIM1->ARR = period;		//Set the new timer period	
}

//This function returns the velocity to use in the motor controller, and takes arguments of pointer to produced velocity and input request velocity
float Set_Stepper_Velocity(float *ret_v, float v) {
	float periods = roundf(v*(PRESSURE_TIME_SECONDS/4.0)*STEPS_PER_ROTATION/LEADSCREW_PITCH);/* Round to integer number of stepper steps */
	periods *= 2.0;			/* As we are using toggle mode pwm, we have to double the number of timer periods */
	float periods_actual = periods;/* Round to the nearest half integer number of timer periods, to give isr some timing clearance */
	periods_actual += signbit(periods_actual)?-0.5:0.5;
	if(ret_v)
		*ret_v = periods/((PRESSURE_TIME_SECONDS/2.0)*STEPS_PER_ROTATION/LEADSCREW_PITCH);/* Finally convert back to a rounded velocity -actual v*/
	return periods_actual/((PRESSURE_TIME_SECONDS/2.0)*STEPS_PER_ROTATION/LEADSCREW_PITCH);/* Return the value to use in the stepper */
}
