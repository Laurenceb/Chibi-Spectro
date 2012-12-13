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

#define PRESSURE_TIME_SECONDS ((float)PRESSURE_TIME_INTERVAL/1000.0)

/* gives a sample period of 2.96 GPT intervals - enough for the potentiometer read to have time to run afterwards and fit into 3 GPT intervals */
/* the thread then has chance to run anywhere in the fourth interval */
#define PRESSURE_SAMPLE_BUFF_SIZE ((42*PRESSURE_TIME_INTERVAL)*3/4)
#define POT_SAMPLE_BUFF_SIZE 1

#define ACTUATOR_LIMIT_MINUS	(0.1*ACTUATOR_LENGTH)
#define ACTUATOR_LIMIT_PLUS	(0.9*ACTUATOR_LENGTH)

#define ACTUATOR_STEP_CONSTANT	((LEADSCREW_PITCH/STEPS_PER_ROTATION)*TIMER1_CLK/2.0)/* Note 2.0 due to toggle mode */

#ifdef INVERSE_POT
	#define CONVERT_POT(adc)	(ACTUATOR_LENGTH*(float)(4096-adc[0])/4096.0)
#else
	#define CONVERT_POT(adc)	(ACTUATOR_LENGTH*(float)adc[0]/4096.0)
#endif

/* Hand position and properties estimator EKF related macros */

#define MODULUS	1.0		/* Estimated from existing tests, Units are PSI/mm */

#define	MODULUS_LIMIT	(MODULUS*MODULUS/4.0)

#define POS_LIMIT	(ACTUATOR_LENGTH*ACTUATOR_LENGTH/11.0)/* Limit on the size of the position error covar */

#define MEASUREMENT_COVAR 1/*((PRESSURE_MARGIN/3)*(PRESSURE_MARGIN/3))/* 3 sigma margin */

#define PROCESS_NOISE {0.001,(0.1*0.1)}/* EV of 0.1% shift in modulus, 0.1mm hand drift per second,  */

#define INITIAL_COVAR {{MODULUS*MODULUS/9.0,0},{0,ACTUATOR_LENGTH*ACTUATOR_LENGTH/20.0}}/* EKF covar intialisation */

#define INITIAL_STATE {MODULUS,ACTUATOR_LENGTH*0.8}/* Initial position towards end of travel, to maximise probability of a hand hit at initialisation */
