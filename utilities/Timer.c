#include "Timer.h"

void Setup_PPG_PWM(void) {
	/* Configure the timers */
	//Spare
	SPARE_TIMER->CR1=TIM_CR1_ARPE;	//Preload enable
	SPARE_TIMER->CR2=TIM_CR2_MMS_2|SPARE_GATE_OUTPUT<<4;//The internal trigger output from this timer
	SPARE_TIMER->ARR=PWM_PERIOD_CENTER;
	//Channel 1
	TIM2->CR1=TIM_CR1_ARPE;		//Preload enable
	TIM2->CR2=TIM_CR2_MMS_2|CHAN1_GATE_OUTPUT<<4;//The internal trigger output from this timer
	TIM2->ARR=PWM_PERIOD_CENTER;
	//Channel 2	
	TIM4->CR1=TIM_CR1_ARPE;		//Preload enable
	TIM4->CR2=TIM_CR2_MMS_2|CHAN2_GATE_OUTPUT<<4;//The internal trigger output from this timer
	TIM4->ARR=PWM_PERIOD_TWO;
	//Channel 3
	TIM5->CR1=TIM_CR1_ARPE;		//Preload enable
	TIM5->CR2=TIM_CR2_MMS_2;	//No internal trigger output from this timer
	TIM5->ARR=PWM_PERIOD_THREE;
	//Channel 4
	TIM3->CR1=TIM_CR1_ARPE;		//Preload enable
	TIM3->CR2=TIM_CR2_MMS_2|CHAN4_GATE_OUTPUT<<4;//The internal trigger output from this timer
	TIM3->ARR=PWM_PERIOD_FOUR;
	//Channel 5
	TIM9->CR1=TIM_CR1_ARPE;		//Preload enable
	TIM9->CR2=TIM_CR2_MMS_2;	//No internal trigger output from this timer
	TIM9->ARR=PWM_PERIOD_FIVE;
	/* Configure the OC units */

	//Enable timers atomically - careful where this is called from
	SPARE_TIMER->CR1=TIM_CR1_CEN;	//Enable the timer
	TIM2->CR1=TIM_CR1_CEN;
	TIM3->CR1=TIM_CR1_CEN;
	TIM4->CR1=TIM_CR1_CEN;
	TIM5->CR1=TIM_CR1_CEN;
	TIM9->CR1=TIM_CR1_CEN;
}
