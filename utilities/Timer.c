#include "Timer.h"

/**
  * @brief  Setup the timers and start them running to generate orthogonal pwm frequencies
  * @param  None
  * @retval None
  * This does not turn on the PWM output, call atomically
  */
void Setup_PPG_PWM(void) {
	/* Clk and Reset the timers */
	RCC->APB1ENR|=RCC_APB1ENR_TIM2EN|RCC_APB1ENR_TIM3EN|RCC_APB1ENR_TIM4EN|RCC_APB1ENR_TIM5EN;
	RCC->APB2ENR|=RCC_APB2ENR_TIM10EN|RCC_APB2ENR_TIM9EN; 
	RCC->APB1RSTR|=RCC_APB1RSTR_TIM2RST|RCC_APB1RSTR_TIM3RST|RCC_APB1RSTR_TIM4RST|RCC_APB1RSTR_TIM5RST;
	RCC->APB2RSTR|=RCC_APB2RSTR_TIM10RST|RCC_APB2RSTR_TIM9RST;
	RCC->APB1RSTR&=~(RCC_APB1RSTR_TIM2RST|RCC_APB1RSTR_TIM3RST|RCC_APB1RSTR_TIM4RST|RCC_APB1RSTR_TIM5RST);
	RCC->APB2RSTR&=~(RCC_APB2RSTR_TIM10RST|RCC_APB2RSTR_TIM9RST);
	/* Configure the timers */
	//Spare
	//SPARE_TIMER->CR1|=TIM_CR1_ARPE;	//Preload enable
	SPARE_TIMER->CR2|=TIM_CR2_MMS_2|(SPARE_GATE_OUTPUT-1)<<4;//The internal trigger output from this timer
	SPARE_TIMER->ARR=PWM_PERIOD_CENTER-1;
	SPARE_TIMER->SMCR|=TIM_SMCR_MSM;//MSM mode on first master, intermediate timers, but not final slave
	SPARE_TIMER->CCMR1|=0x0060|TIM_CCMR1_OC1PE;//Use OC1 in PWM1 mode with preload as the internal signal
	SPARE_TIMER->CCR1=PWM_PERIOD_CENTER-PWM_CLK_TRIM_SPARE;
	//Channel 1
	//TIM2->CR1|=TIM_CR1_ARPE;	//Preload enable
	TIM2->CR2|=TIM_CR2_MMS_2|(CHAN1_GATE_OUTPUT-1)<<4;//The internal trigger output from this timer
	TIM2->ARR=PWM_PERIOD_CENTER-1;
	TIM2->SMCR|=TIM_SMCR_MSM;	//MSM mode on first master, intermediate timers, but not final slave
	TIM2->CCMR2|=0x0060|TIM_CCMR2_OC3PE;//Use OC3 in PWM1 mode with preload as the internal signal
	TIM2->CCR3=PWM_PERIOD_CENTER-PWM_CLK_TRIM_ONE;//OC3 Gate output
	TIM2->CCMR2|=0x7000|TIM_CCMR2_OC4PE;//Use OC4 in PWM2 mode with preload as the LED signal
	TIM2->CCR4=DIM_LED;		//OC4 initial output - not enabled at this point
	//Channel 2	
	//TIM4->CR1|=TIM_CR1_ARPE;		//Preload enable
	TIM4->CR2|=TIM_CR2_MMS_2|(CHAN2_GATE_OUTPUT-1)<<4;//The internal trigger output from this timer
	TIM4->ARR=PWM_PERIOD_TWO-1;
	TIM4->SMCR|=TIM_SMCR_MSM|0x0005|TIM4_TS<<4;//MSM mode, Gated, Trigger source
	TIM4->CCMR1|=0x6000|TIM_CCMR1_OC2PE;//Use OC2 in PWM1 mode with preload as the internal signal
	TIM4->CCR2=PWM_PERIOD_TWO-PWM_CLK_TRIM_TWO;//OC2 Gate output
	TIM4->CCMR2|=0x0070|TIM_CCMR2_OC3PE;//Use OC3 in PWM2 mode with preload as the LED signal
	TIM4->CCR3=DIM_LED;		//OC3 initial output - not enabled at this point
	//Channel 3
	//TIM5->CR1=TIM_CR1_ARPE;		//Preload enable
	TIM5->CR2=TIM_CR2_MMS_2;	//No internal trigger output from this timer
	TIM5->ARR=PWM_PERIOD_THREE;
	TIM5->SMCR=0x0005|TIM5_TS<<4;	//Gated, Trigger source
	TIM5->CCMR1=0x7000|TIM_CCMR1_OC2PE;//Use OC2 in PWM2 mode with preload as the LED signal
	TIM5->CCR2=DIM_LED;		//OC2 initial output - not enabled at this point
	//Channel 4
	//TIM3->CR1|=TIM_CR1_ARPE;		//Preload enable
	TIM3->CR2|=TIM_CR2_MMS_2|(CHAN4_GATE_OUTPUT-1)<<4;//The internal trigger output from this timer
	TIM3->ARR=PWM_PERIOD_FOUR-1;
	TIM3->SMCR|=TIM_SMCR_MSM|0x0005|TIM3_TS<<4;//MSM mode, Gated, Trigger source
	TIM3->CCMR1|=0x6000|TIM_CCMR1_OC2PE;//Use OC2 in PWM1 mode with preload as the internal signal
	TIM3->CCR2=PWM_PERIOD_FOUR-PWM_CLK_TRIM_FOUR;//OC2 Gate output
	TIM3->CCMR2|=0x0070|TIM_CCMR2_OC3PE;//Use OC3 in PWM2 mode with preload as the LED signal
	TIM3->CCR3=DIM_LED;		//OC3 initial output - not enabled at this point
	//Channel 5
	//TIM9->CR1=TIM_CR1_ARPE;		//Preload enable
	TIM9->CR2|=TIM_CR2_MMS_2;	//No internal trigger output from this timer
	TIM9->ARR=PWM_PERIOD_FIVE-1;
	TIM9->SMCR|=0x0005|TIM9_TS<<4;	//Gated, Trigger source
	TIM9->CCMR1|=0x0070|TIM_CCMR1_OC1PE;//Use OC1 in PWM2 mode with preload as the LED signal
	TIM9->CCR1=DIM_LED;		//OC1 initial output - not enabled at this point
	/* Do not Configure the OC units to generate PWM to the LEDs yet here */
	/* Set Timers to differing phases to reduce the Peak to average ratio - uses matlab script in ./phasing*/
	TIM2->CNT=PWM_INIT_ONE;
	TIM4->CNT=PWM_INIT_TWO;
	TIM5->CNT=PWM_INIT_THREE;
	TIM3->CNT=PWM_INIT_FOUR;
	TIM9->CNT=PWM_INIT_FIVE;
	//Enable timers atomically - careful where this is called from
	SPARE_TIMER->CR1|=TIM_CR1_CEN;	//Enable the timer
	TIM2->CR1|=TIM_CR1_CEN;
	TIM3->CR1|=TIM_CR1_CEN;
	TIM4->CR1|=TIM_CR1_CEN;
	TIM5->CR1|=TIM_CR1_CEN;
	TIM9->CR1|=TIM_CR1_CEN;
}

/**
  * @brief  Enable PPG PWM outputs to LEDS
  * @param  None
  * @retval None
  */
void Enable_PPG_PWM(void) {
	TIM2->CCER|=TIM_CCER_CC4E;
	TIM3->CCER|=TIM_CCER_CC3E;
	TIM4->CCER|=TIM_CCER_CC3E;
	TIM5->CCER|=TIM_CCER_CC2E;
	TIM9->CCER|=TIM_CCER_CC1E;
}

/**
  * @brief  Disable PPG PWM outputs to LEDS
  * @param  None
  * @retval None
  */
void Disable_PPG_PWM(void) {
	TIM2->CCER&=~TIM_CCER_CC4E;
	TIM3->CCER&=~TIM_CCER_CC3E;
	TIM4->CCER&=~TIM_CCER_CC3E;
	TIM5->CCER&=~TIM_CCER_CC2E;
	TIM9->CCER&=~TIM_CCER_CC1E;
}



