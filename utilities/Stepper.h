#pragma once

void Setup_Stepper_PWM(void);
void GPIO_Stepper_Enable(uint8_t En);
void GPIO_Stepper_Dir(uint8_t Dir);

#define SET_STEPPER_PERIOD(Timer_Period) TIM8->ARR=Timer_Period
