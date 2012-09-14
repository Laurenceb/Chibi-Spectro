#include "Timer.h"

void Setup_PPG_PWM(void) {
	CORE_TIMER->CR1=TIM_CR1_ARPE;	//Preload enable
	CORE_TIMER->CR2=TIM_CR2_MMS_2|INTERNAL_OUTPUT_CORE<<4;//The internal trigger output from this timer
	CORE_TIMER->ARR=PWM_PERIOD_CENTER;
	




	CORE_TIMER->CR1=TIM_CR1_CEN;	//Enable the timer
}
