#pragma once
#include "stm32f4xx.h"

void Setup_Stepper_PWM(void);
void GPIO_Stepper_Enable(uint8_t En);
void GPIO_Stepper_Dir(uint8_t Dir);
void Set_Stepper_Period(uint16_t period);
float Set_Stepper_Velocity(float *ret_v, float v);

#define SET_STEPPER_PERIOD(Timer_Period) TIM1->ARR=((uint16_t)Timer_Period-1)
