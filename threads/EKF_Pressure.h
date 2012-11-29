#pragma once
#include "ch.h"
#include "hal.h"
#include "Hardware_Conf.h"

extern Mailbox Pressures_Setpoint;
extern Mailbox Pressures_Reported;

typedef struct{
	float MaxAcc;
	float MaxVel;
	float LimitMinus;
	float LimitPlus;
	float DeadVel;
	float DeadPos;
} Actuator_TypeDef;

static void GPT_Stepper_Callback(GPTDriver *gptp);
Thread* Spawn_Pressure_Thread(Actuator_TypeDef* arg);
msg_t Pressure_Thread(void *arg);

static const ADCConversionGroup adcgrpcfg2_pot;
static const ADCConversionGroup adcgrpcfg2_pressure;

#ifndef PRESSURE_TIME_INTERVAL
	/* 100Hz pressure control loop */
	#define PRESSURE_TIME_INTERVAL 10
#endif

#define PRESSURE_SAMPLE_BUFF_SIZE 200
#define POT_SAMPLE_BUFF_SIZE 1

#define ACTUATOR_LIMIT_MINUS	(0.1*ACTUATOR_LENGTH)
#define ACTUATOR_LIMIT_PLUS	(0.9*ACTUATOR_LENGTH)

#define ACTUATOR_STEP_CONSTANT	((LEADSCREW_PITCH/STEPS_PER_ROTATION)*TIMER8_CLK*PRESSURE_TIME_INTERVAL/(2.0*1000.0))/* Note 2.0 due to toggle mode */

#define CONVERT_POT(adc)	(30.0*adc[0]/4096.0)

/* Hand position and properties estimator EKF related macros */

#define MODULUS	1.5		/* Estimated from existing tests, Units are PSI/mm */

#define MEASUREMENT_COVAR ((PRESSURE_MARGIN/3)*(PRESSURE_MARGIN/3))/* 3 sigma margin */

#define PROCESS_NOISE {3*3,MODULUS*MODULUS/100.0}/* EV of 3mm hand drift per second, 10% shift in modulus */

#define INITIAL_COVAR {{MODULUS*MODULUS/9.0,0},{ACTUATOR_LENGTH*ACTUATOR_LENGTH/20.0,0}}/* EKF covar intialisation */

#define INITIAL_STATE {MODULUS,ACTUATOR_LENGTH*0.8}/* Initial position towards end of travel, to maximise probability of a hand hit at initialisation */
