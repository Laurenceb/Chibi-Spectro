#pragma once
#include "ch.h"

extern Mailbox Pressures_Setpoint;
extern Mailbox Pressures_Reported;

Thread* Spawn_Pressure_Thread(Actuator_TypeDef* *arg);

#ifndef PRESSURE_TIME_INTERVAL
	/* 100Hz pressure control loop */
	#define PRESSURE_TIME_INTERVAL 10
#endif

#define PRESSURE_SAMPLE_BUFF_SIZE 200
#define POT_SAMPLE_BUFF_SIZE 1

#define ACTUATOR_LIMIT_MINUS	(0.1*ACTUATOR_LENGTH)
#define ACTUATOR_LIMIT_PLUS	(0.9*ACTUATOR_LENGTH)

#define ACTUATOR_STEP_CONSTANT	((LEADSCREW_PITCH/STEPS_PER_ROTATION)*TIMER8_CLK*PRESSURE_TIME_INTERVAL/(2.0*1000.0))/* Note 2.0 due to toggle mode */
