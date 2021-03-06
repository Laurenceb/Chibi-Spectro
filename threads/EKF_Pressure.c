#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "EKF_Pressure.h"
#include "Stepper.h"
#include "EKF_Pressure.h"
#ifndef EKF_NONLINEAR
 #include "EKF_Estimator.h"
#else
 #include "EKF_Estimator_Nonlin.h"
#endif
#include "Quickselect.h"
#include "Quicksort.h"
#include "Pressure.h"
#define EKF_PRESSURE
#ifdef EKF_PRESSURE
/*
 * Mailboxes and buffers for the setpoint pressure input into this thread
 */
Mailbox Pressures_Setpoint;
static msg_t Pressures_Setpoint_Buff[MAILBOX_SIZE];

/*
 * Mailboxes and buffers for the reported pressure output from this thread
 */
Mailbox Pressures_Reported;
static msg_t Pressures_Reported_Buff[MAILBOX_SIZE];

/*
 * Mailboxes and buffers for the actuator velocities passed to the GPT ISR
 */
static Mailbox Actuator_Velocities;
static msg_t Actuator_Velocities_Buff[MAILBOX_SIZE];

/*
 * Working area for this thread
*/
static WORKING_AREA(waThreadPressure, 8192);

/*
 * Thread pointer used for thread wakeup processing
*/
static Thread *tp = NULL;

/*
*  Samples buffer for ADC2
*/
static adcsample_t Pot_sample[POT_SAMPLE_BUFF_SIZE];
static adcsample_t Pressure_Samples[PRESSURE_SAMPLE_BUFF_SIZE];

/**
  * @brief  This function spawns the pressure control thread
  * @param  void* to a PID Loop configuration
  * @retval thread pointer to the spawned thread
  */
Thread* Spawn_Pressure_Thread(Actuator_TypeDef *arg) {
	/*
	* Creates the mailbox buffers associated with this thread.
	*/
	chMBInit(&Pressures_Setpoint, Pressures_Setpoint_Buff, MAILBOX_SIZE);//In
	chMBInit(&Pressures_Reported, Pressures_Reported_Buff, MAILBOX_SIZE);//Out
	chMBInit(&Actuator_Velocities, Actuator_Velocities_Buff, MAILBOX_SIZE);//Internal
	/* Start the Allegro stepper driver pwm */
	GPIO_Stepper_Enable(0);/* Ensure the driver is off first */
	Setup_Stepper_PWM();
	/*
	* Creates the thread. Thread has priority slightly below normal and takes no argument
	*/
	return chThdCreateStatic(waThreadPressure, sizeof(waThreadPressure), NORMALPRIO+1, Pressure_Thread, (void*)arg);
}

//ADC callback functions, no final adccallback as GPT wakes up the thread
static void adc2errorcallback(ADCDriver *adcp, adcerror_t err) {
	(void)adcp;
	(void)err;
}

//This is the callback from the pressure convertion completing, it triggers the pos convertion
static void adc2callback_pressure(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
	if(buffer!=Pressure_Samples)
		adcStartConversion(&ADCD2, &adcgrpcfg2_pot, Pot_sample, POT_SAMPLE_BUFF_SIZE);/* Fire off the ADC2 samples - very fast */
}

/*
 * ADC conversion group for the pressure monitoring
 * Mode:        Linear buffer, multiple sample of 1 channel, SW triggered.
 * Channels:    ADC_PRESSURE_CHANNEL
 */
static const ADCConversionGroup adcgrpcfg2_pressure = {
  FALSE,
  PRESSURE_ADC_NUM_CHANNELS,
  adc2callback_pressure,
  adc2errorcallback,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  ADC_SMPR1_SMP_AN14(ADC_SAMPLE_480),
  0,                        /* SMPR2 */
  ADC_SQR1_NUM_CH(PRESSURE_ADC_NUM_CHANNELS),
  0,                        /* SQR2 */
  ADC_SQR3_SQ1_N(ADC_PRESSURE_CHANNEL)
};

/*
 * ADC conversion group for the pressure calibration
 * Mode:        Linear buffer, 1 sample of 1 channel, SW triggered.
 * Channels:    ADC_PRESSURE_CHANNEL
 */
static const ADCConversionGroup adcgrpcfg2_pressure_calibrate = {
  FALSE,
  PRESSURE_ADC_NUM_CHANNELS,
  NULL,
  adc2errorcallback,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  ADC_SMPR1_SMP_AN14(ADC_SAMPLE_480),
  0,                        /* SMPR2 */
  ADC_SQR1_NUM_CH(1),
  0,                        /* SQR2 */
  ADC_SQR3_SQ1_N(ADC_PRESSURE_CHANNEL)
};

/*
 * ADC conversion group for the pot monitoring
 * Mode:        Linear buffer, multiple sample of 1 channel, SW triggered.
 * Channels:    ADC_POT_CHANNEL
 */
static const ADCConversionGroup adcgrpcfg2_pot = {
  FALSE,
  POT_ADC_NUM_CHANNELS,
  NULL,
  adc2errorcallback,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  ADC_SMPR1_SMP_AN11(ADC_SAMPLE_480),
  0,                        /* SMPR2 */
  ADC_SQR1_NUM_CH(POT_ADC_NUM_CHANNELS),
  0,                        /* SQR2 */
  ADC_SQR3_SQ1_N(ADC_POT_CHANNEL)
};

static GPTConfig gpt8cfg =
{
    200000,					/* Timer clock.*/
    GPT_Stepper_Callback			/* Timer callback.*/
};

/* Global(s) for passing motor info */
volatile float Target;

/*
 * GPT8 callback.
 *      Every time the timer fires setup a new pwm period for the Stepper driver
 */
static void GPT_Stepper_Callback(GPTDriver *gptp){
	float Motor_Velocity;
	chSysLockFromIsr();			/* Use lock to access the mailbox fifo */
	if(chMBFetchI(&Actuator_Velocities, (msg_t*)&Motor_Velocity) == RDY_OK) {/* If we have some data */
		chSysUnlockFromIsr();
		if(!Motor_Velocity) {
			GPIO_Stepper_Enable(0);	/* Disable the stepper driver if zero velocity */
		}
		else {
			GPIO_Stepper_Enable(1);	/* Enable the stepper motor driver */
			GPIO_Stepper_Dir(Motor_Velocity>0);/* Set the direction line to the motor */
			Motor_Velocity=fabs(Motor_Velocity);/* Make sure this is positive */
		}
		uint16_t Timer_Period=Motor_Velocity?(uint16_t)(ACTUATOR_STEP_CONSTANT/Motor_Velocity):0xFFFF;
		Set_Stepper_Period(Timer_Period);/* Set the timer ARR register to control pwm period */	
		chSysLockFromIsr();
	}
	uint8_t freeslots = chMBGetUsedCountI( &Actuator_Velocities );
	chSysUnlockFromIsr();			/* Wakeup the pressure controller thread, enter lock mode to allow read of slots in mailbox */
	if( !freeslots ) {			/* There are no more messages : we are entering final time interval */
		chSysLockFromIsr();
		if (tp != NULL) {
			tp->p_u.rdymsg = (msg_t)NULL;/* Sending a message, e.g. buffer index.*/
			chSchReadyI(tp);
			tp = NULL;
			chSysUnlockFromIsr();
		}
		else {
			chSysUnlockFromIsr();
			GPIO_Stepper_Enable(0);	/* Disable the stepper driver driver if there is no data and we cannot awake the thread */
		}
	}
	else if( freeslots == 3)		/* The control thread just ran, so we are entering the first time interval */
		adcStartConversion(&ADCD2, &adcgrpcfg2_pressure, Pressure_Samples, PRESSURE_SAMPLE_BUFF_SIZE);/* Start ADC2 samples - takes just < 3 GPT */
}

/**
  * @brief  This is the Pressure control thread
  * @param  void* arg - pointer to an actuator tydef holding actuator capability information
  * @retval msg_t status
  */
msg_t Pressure_Thread(void *arg) {		/* Initialise as zeros */
	msg_t msg;				/* Used to read the setpoint buffer and messages from GPT */
	uint8_t index=0,Direction=0;
	uint16_t sindex=0,holdoff=0;
	float velocities[4]={},velocity,prior_velocity=0,position,delta,actuator_midway_position_est=0,actuator_position,actuator_velocity,\
	actuator_midway_position=0,actuator_midway_position_efk_beadband=0,pot_position,end_position,pressure=0,target=0,Setpoint=0,\
	real_position=0,old_actuator_midway_position=0;
	float State[STATE_SIZE]=INITIAL_STATE,Covar[STATE_SIZE][STATE_SIZE]=INITIAL_COVAR;/* Initialisation for the EKF */
	float Process_Noise[STATE_SIZE]=PROCESS_NOISE,Measurement_Covar=MEASUREMENT_COVAR;
	uint16_t Pressure_Sample;
	Actuator_TypeDef* Actuator=arg;		/* Pointer to actuator definition - MaxAcc and MaxVel defined as per GPT timebin */
	chRegSetThreadName("EKF Pressure");
	/*
	* Activates the ADC2 driver
	*/
	adcStart(&ADCD2, NULL);
	/*
	* ADC2 runs single DMA transactions of multiple conversions.
	*/
	/* Enable the GPT8 timer (TIM8) */	
	gptStart(&GPTD8, &gpt8cfg);
	/* 
	* Start the GPT in continuous mode, use TIM8.  dT is the time between triggers
	* Here, we have set the timer clock to 200,000Hz, and we want
	* to call the callback function every 500 GPT clock cycles.  This
	* means we call the callback function every 2500uS or 400 times per second
	 */
    	gptStartContinuous(&GPTD8, (200*PRESSURE_TIME_INTERVAL)/4 ); // dT = 200,000 / 500 = 400Hz
	/*
	* Linear Actuator parking loop
	* park the linear actuator near to the top of the run
	*/
	do {
		velocity=Set_Stepper_Velocity(NULL, -4.0);//a slow retraction speed means that motor can be turned on/off freely
		for(index=0;index<4;index++)	/* Post 4 velocities to the stepper GPT */
			chMBPost(&Actuator_Velocities, *((msg_t*)&velocity), TIME_IMMEDIATE);/* Non blocking write attempt to GPT motor driver */
		/* Sleep until we are awoken by the GPT ISR - meaning the 4 GPT intervals have been read */
		/* Waiting for the IRQ to happen.*/
		chSysLock();
		tp = chThdSelf();
		chSchGoSleepS(THD_STATE_SUSPENDED);
		msg = chThdSelf()->p_u.rdymsg;	/* Retrieving the message, gives us the correct buffer index*/
		chSysUnlock();
	}while(CONVERT_POT(Pot_sample)>ACTUATOR_LENGTH/6);/* Wait for the pot feedback to indicate that we are at end of travel */
	/* Set the actuator in sleep mode (stator with no current ) */
	velocity=0;
	chMBPost(&Actuator_Velocities, *((msg_t*)&velocity), TIME_IMMEDIATE);/* This will force the stepper driver to off state */
	/* Reset these after the actuator is positioned */
	actuator_position=0;
	actuator_velocity=0;
	/* At this point when starting up we need to calibrate the force sensor offset, so fire off ADC group convertions and calibrate until ready*/
	holdoff=0;
	do {
		adcConvert(&ADCD2, &adcgrpcfg2_pressure_calibrate, &Pressure_Sample, 1);/* This function blocks until it has one sample*/
		if(holdoff++ & 0x0010)	
			chThdSleepMilliseconds(10);
	} while(Calibrate_Sensor((uint16_t)Pressure_Sample));
	/* Wait for data */	
	do {
		chThdSleepMilliseconds(20);
	} while(chMBFetch(&Pressures_Setpoint, (msg_t*)&Setpoint, TIME_IMMEDIATE) != RDY_OK);/* Loop until we get some Setpoints send to the thread */
	/* Loop for the Pressure thread */
	while(TRUE) {
		/* Sleep until we are awoken by the GPT ISR */
		/* Waiting for the IRQ to happen.*/
		chSysLock();
		tp = chThdSelf();
		chSchGoSleepS(THD_STATE_SUSPENDED);
		msg = chThdSelf()->p_u.rdymsg;	  /* Retrieving the message, gives us the correct buffer index*/
		chSysUnlock();
		/* Run the hand properties estimator EKF and find optimal position for the Actuator */
		/*
		/ First we process the data - we use a median filter to take out the non guassian noise
		*/
		QuickSort(Pressure_Samples, PRESSURE_SAMPLE_BUFF_SIZE-1);/* Use the quick sort algorythm - from numerical recepies*/
		pressure=0;
		for(sindex = (PRESSURE_SAMPLE_BUFF_SIZE/4); sindex < ((PRESSURE_SAMPLE_BUFF_SIZE*3)/4); sindex++)/* Interquartile mean */
			pressure += Pressure_Samples[sindex];
		pressure = Convert_Pressure(pressure/(PRESSURE_SAMPLE_BUFF_SIZE/2));/* Converts to PSI as a float */
		/* Get the potentiometer value from the actuator */
		pot_position = CONVERT_POT(Pot_sample);/* The membrane pot used in the Firgelli L12 linear actuator is very poor, use for endstops only */
		/* Generate the actual actuator position (end of actuator) at sampling midpoint using estimated position */
		/* Only update if the end of the actuator isnt idling in deadband */
		if( fabs(actuator_midway_position-actuator_midway_position_est)>Actuator->BackLash/2.0 )/* This is a backlash corrected position for the EKF */
			actuator_midway_position=actuator_midway_position_est+Actuator->BackLash*((actuator_midway_position_est\
									>actuator_midway_position)?-0.5:0.5);
		/* Run the EKF */
		#ifndef EKF_NONLINEAR
		Predict_State(State, Covar, PRESSURE_TIME_SECONDS, Process_Noise);
		if(pressure>PRESSURE_MARGIN)	/* Only run the Update set if the pressure sensor indicates we are in contact */
			Update_State(State, Covar, pressure, actuator_midway_position, 0.1*(velocity*velocity*State[0]*State[0])+1); 
			//Measurement_Covar);/*Use the previously stored midway position */
		else if(actuator_midway_position>State[1] && !fabs(velocity))
			State[1]=actuator_midway_position;/* Adjust the State position if there is no contact */
		/* Now that the EKF has been run, we can use the current EKF state to solve for a target position, given our setpoint pressure */
		if(chMBFetch(&Pressures_Setpoint, (msg_t*)&Setpoint, TIME_IMMEDIATE) == RDY_OK)
			#ifdef GAIN_FACTOR
			target = actuator_midway_position + (  ( (Setpoint-pressure) * GAIN_FACTOR) / State[0] ) ;
			#endif
			//target = 5.0+(float)Setpoint;/* Debug test code for testing lower level motor control code */
		else
			target = State[1];	/* If we arent getting any data, set the Target to the point where we are just touching the target */
		#else
		if(pressure>PRESSURE_MARGIN) {
			Predict_State(State, Covar, PRESSURE_TIME_SECONDS, Process_Noise, actuator_midway_position-old_actuator_midway_position);
			if(fabs(actuator_midway_position - actuator_midway_position_efk_beadband) > Actuator->EKF_DeadBand/2.0) {/* Dedicated deadband for EKF */
				Update_State(State, Covar, pressure, 2.5e-4*fabs(velocity)+0.0004 );/* Velocity can be used as a surrogate for unmodeled thixotropy */
				actuator_midway_position_efk_beadband = actuator_midway_position+Actuator->EKF_DeadBand*((actuator_midway_position>\
							actuator_midway_position_efk_beadband)?-0.5:0.5);
			} 
		}
		else
			State[0]=pressure;
		State[0]=(State[0]<0)?0.0001:State[0];
		State[1]=(State[1]<0.1)?0.1:State[1];
		State[2]=(State[2]<0)?0.0001:State[2];
		State[2]=(State[2]>0.5)?0.5:State[2];
		pressure = (pressure<0)?0.0001:pressure;/* Force everything to keep sane values */
		old_actuator_midway_position = actuator_midway_position;
		/* Now that the EKF has been run, we can use the current EKF state to solve for a target position, given our setpoint pressure */
		if(chMBFetch(&Pressures_Setpoint, (msg_t*)&Setpoint, TIME_IMMEDIATE) == RDY_OK) {
			#ifdef GAIN_FACTOR
			Setpoint = pressure + ( Setpoint - pressure ) * GAIN_FACTOR;/* Reduce gain down from 1 for sub-optimal but more stable control*/
			#endif
			target = actuator_midway_position + logf((Setpoint+(State[1]/State[2]))/(pressure+(State[1]/State[2])))/State[2];
		}
		#endif
		Target=target;			/* Debug test output from thread */
		/* Perform motor driver processing, places results into mailbox fifo */
		position=actuator_position;	/* Store the position variable from the previous 4th loop */
		velocity=actuator_velocity;	/* Same for the velocity - this will be valid data only at the END of the current GPT bin */
		actuator_midway_position_est=position;/* Initialise this - estimated with no backlash correction */
		/* Apply software end stops */
		/* Note that the EKF position is not in real space - it can drift, so we use pot */
		end_position = ( pot_position + target - position + (velocities[3]*PRESSURE_TIME_SECONDS/4.0) );/* Pot is measured at end of 3rd GPT */
		if( end_position > Actuator->LimitPlus )/* To extropolate the end position after moving to position Target */
			target -= end_position - Actuator->LimitPlus;/* Adjust the Target - this may mean we never reach correct pressure */
		else if( end_position < Actuator->LimitMinus )
			target += Actuator->LimitMinus - end_position;/* But at least we dont hit the end stop */
		/* Loop through the GPT bins, issuing the next 4 instructions (GPT callback at 400hz) */
		for(index=0;index<5;index++) {	/* Note that there are 5 iterations - we cover the 4th timebin twice allowing backcorrection scheduling*/
			if( fabs(position-real_position)>Actuator->BackLash/2.0 )/* Here the targetted stepper rotation is corrected for backlash */
				real_position=position+Actuator->BackLash*((position>real_position)?-0.5:0.5);/* Real target is a corrected one*/
			delta=target-real_position;/* The travel distance to target - defined starting from next GPT callback */
			if( fabs(delta)<Actuator->DeadPos && fabs(velocity)<Actuator->DeadVel ) {/* Position,Veloctiy deadband for our hardware */
				prior_velocity=velocity;/* The initial velocity is stored for reference */
				velocity=0;	/* If the GPT callback reads zero velocity it knows to de-energize the motor */
			}
			else {			/* We cannot enter the deadband */
				if(signbit(velocity)==signbit(delta)) {/* If we are moving towards the target */
					Direction=0;
					float stop_distance=(velocity*velocity)/(2*Actuator->MaxAcc);/* The minimum stopping distance for the hardware */
					if( fabs(delta) > (stop_distance/*-Actuator->DeadPos*/) ) {/* We have chance to stop with some margin */
						prior_velocity=velocity;/* The initial velocity is stored for reference */
						if(!signbit(delta))/* Apply maximum acceleration in the correct direction */
							velocity+=Actuator->MaxAcc*PRESSURE_TIME_SECONDS/4.0;
						else/* This is the velocity for the next GPT bin */
							velocity-=Actuator->MaxAcc*PRESSURE_TIME_SECONDS/4.0;
						if( fabs(velocity)>Actuator->MaxVel )/* We are above the max speed - enforce limits on speed */
							velocity=signbit(delta)?-Actuator->MaxVel:Actuator->MaxVel;
					}
					else {	/* We are unable to stop without overshooting the target - try to backcorrect if scheduling allows */
						if(index) {/* If the current bin is not the first, we may be able to backcorrect*/
							float overshoot_velocity =sqrtf( 2*Actuator->MaxAcc*(stop_distance-fabs(delta)) );/*Exs Vel*/
							float first_acc = velocity-prior_velocity;/* The acceleration over previous GPT timestep */
							first_acc += signbit(delta)?overshoot_velocity:-overshoot_velocity;/* Correct this accel */
							if( fabs(first_acc)>Actuator->MaxAcc*PRESSURE_TIME_SECONDS/4.0 ) {/* Adjusted acc exceeds limit */
								first_acc = signbit(first_acc)?-Actuator->MaxAcc:Actuator->MaxAcc;/* Apply range limit */
								first_acc*= PRESSURE_TIME_SECONDS/4.0;/* Scale for delta time, ready to add to velocity */
							}
							velocity = prior_velocity + first_acc;/* Backapply the acceleration */
							velocities[index-1] = velocity;/* Backapply velocity */
						}/* Try our best to slow down in this interval */
						prior_velocity = velocity;/* The initial velocity is stored for reference */
						velocity += signbit(delta)?Actuator->MaxAcc*PRESSURE_TIME_SECONDS/4.0:\
									  -Actuator->MaxAcc*PRESSURE_TIME_SECONDS/4.0;
					}
				}
				else {		/* If we are moving away from the target */
					if(!Direction && index ) {/* We were heading towards the target */
						float first_acc = (velocity-prior_velocity);/* Acc over previous GPT timestep */
						first_acc -= velocity;/* Correct this accel */
						if( fabs(first_acc)>Actuator->MaxAcc*PRESSURE_TIME_SECONDS/4.0 ) {/* Adjusted acceleration exceeds limit */
							first_acc = signbit(first_acc)?-Actuator->MaxAcc:Actuator->MaxAcc;/* Apply range limit */
							first_acc*= PRESSURE_TIME_SECONDS/4.0;/* Scale for delta time, ready to add to velocity */
						}
						velocity = prior_velocity + first_acc;/* Backapply the acceleration */
						velocities[index-1] = velocity;/* Backapply velocity */
					}
					prior_velocity=velocity;
					velocity-=signbit(delta)?Actuator->MaxAcc*PRESSURE_TIME_SECONDS/4.0:-Actuator->MaxAcc*PRESSURE_TIME_SECONDS/4.0;
					Direction=1;
				}		/* Try our best to head towards target*/
			}
			/* Apply some rounding of the velocity at this point to give a half integer number of timer periods */
			if(index<4) {
				velocities[index] = Set_Stepper_Velocity(&velocity, velocity);
				if(!index)	/* Store the midway position to eliminate lag from the EKF */
					actuator_midway_position_est += velocity*PRESSURE_TIME_SECONDS/4.0;
				else if(index==1)/* The ADC sampling interval lasts just under three GPT intervals (3/2=1.5)*/
					actuator_midway_position_est += velocity*PRESSURE_TIME_SECONDS/8.0;
			}
			position += velocity*PRESSURE_TIME_SECONDS/4.0;/* The new reference position for the next loop */
			if(index==3) {		/* Store these for reference in next thread iteration */
				actuator_position = position;
				actuator_velocity = velocity;
			}
		}
		for(index=0;index<4;index++)
			chMBPost(&Actuator_Velocities, *((msg_t*)&velocities[index]), TIME_IMMEDIATE);/* Non blocking write attempt to GPT motor driver */
		chMBPost(&Pressures_Reported, *((msg_t*)&pressure), TIME_IMMEDIATE);/* Non blocking write attempt to the Reported Pressure mailbox FIFO */
	}
}
#endif

