#include "EKF_Pressure.h"
#include "Stepper.h"
#include "EKF_Pressure.h"
#include "Hardware_Conf.h"

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
static WORKING_AREA(waThreadPressure, 2048);

/**
  * @brief  This function spawns the pressure control thread
  * @param  void* to a PID Loop configuration
  * @retval thread pointer to the spawned thread
  */
Thread* Spawn_Pressure_Thread(Actuator_TypeDef* *arg) {
	/*
	* Creates the mailbox buffers associated with this thread.
	*/
	chMBInit(&Pressures_Setpoint, Pressures_Setpoint_Buff, MAILBOX_SIZE);//In
	chMBInit(&Pressures_Reported, Pressures_Reported_Buff, MAILBOX_SIZE);//Out
	chMBInit(&Actuator_Velocities, Actuator_Velocities_Buff, MAILBOX_SIZE);//Internal
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

/*
 * GPT8 callback.
 *      Every time the timer fires setup a new pwm period for the Stepper driver
 */
static void GPT_Stepper_Callback(GPTDriver *gptp){
	float Motor_Velocity;
	if(chMBFetch(&Actuator_Velocities, (msg_t*)&Motor_Velocity, TIME_IMMEDIATE) == RDY_OK) {/* If we have some data */
		Actuator_Position+=Motor_Velocity;/* This is the position at the end of the current time bin */
		Actuator_Velocity=Motor_Velocity;
		if(!Motor_Velocity)
			GPIO_Stepper_Enable(0);	/* Disable the stepper driver if zero velocity */
		else {
			GPIO_Stepper_Enable(1);	/* Enable the stepper motor driver */
			GPIO_Stepper_Dir(Motor_Velocity>0);/* Set the direction line to the motor */
			uint32_t Timer_Period=(uint32_t)(ACTUATOR_STEP_CONSTANT/Motor_Velocity);
			SET_STEPPER_PERIOD(Timer_Period);/* Set the timer ARR register to control pwm period */	
		}
	}
	if( chMBGetUsedCountI(&Actuator_Velocities) <= 1) {/* There is only one more message : we are entering final time interval */
		chSysLockFromIsr();		/* Wakeup the pressure controller thread */
		if (tp != NULL) {
			tp->p_u.rdymsg = /*(buffer==PPG_Sample_Buffer? (msg_t)1 : (msg_t)0 );*//* Sending the message, gives buffer index.*/
			chSchReadyI(tp);
			tp = NULL;
		}
		chSysUnlockFromIsr();
	}
	else if( chMBGetUsedCountI(&Actuator_Velocities) >= 4)/* The control thread just ran, so we are entering the first time interval */
		adcStartConversion(&ADCD2, &adcgrpcfg2, Pressure_Samples, PRESSURE_SAMPLE_BUFF_SIZE);/* Fire off the ADC2 samples - takes just < 3 GPT */
}

/**
  * @brief  This is the Pressure control thread
  * @param  void* arg - pointer to an actuator tydef holding actuator capability information
  * @retval msg_t status
  */
msg_t Pressure_Thread(void *arg) {		/* Initialise as zeros */
	msg_t Setpoint,msg;			/* Used to read the setpoint buffer and messages from GPT */
	uint8_t index=0;
	float velocities[4],velocity,prior_velocity,position,delta,actuator_midway_position,pot_position,end_position;
	float State=INITIAL_STATE,Covar=INITIAL_COVAR;/* Initialisation for the EKF */
	float Process_Noise=PROCESS_NOISE,Measurement_Covar=MEASUREMENT_COVAR;
	Actuator_TypeDef* Actuator=arg;		/* Pointer to actuator definition - MaxAcc and MaxVel defined as per GPT timebin */
	chRegSetThreadName("EKF Pressure");
	/*
	* Activates the ADC2 driver
	*/
	adcStart(&ADCD2, NULL);
	/*
	* ADC2 runs single DMA transactions of multiple conversions.
	*/
	gptStart(&GPTD8, &gpt8cfg);
	/* 
	* Start the GPT in continuous mode.  dT is the time between triggers
	* Here, we have set the timer clock to 200,000Hz, and we want
	* to call the callback function every 500 GPT clock cycles.  This
	* means we call the callback function every 2500uS or 400 times per second
	 */
    	gptStartContinuous(&GPTD8, (200*PRESSURE_TIME_INTERVAL)/4 ); // dT = 200,000 / 500 = 400Hz
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
		Pressure_Sample = quick_select(Pressure_Samples, PRESSURE_SAMPLES);/* Use the quick select algorythm - from numerical recepies*/
		Pressure = Convert_Pressure((uint16_t)Pressure_Sample);/* Converts to PSI as a float */
		pot_position = Convert_Pot(Pot_sample);/* The membrane pot used in the Firgelli L12 linear actuator is very poor, use for endstops only */
		/* Run the EKF */
		Predict_State(State, Covar, PRESSURE_TIME_INTERVAL/1000.0, Process_Noise);
		if(Pressure>PRESSURE_MARGIN)	/* Only run the Update set if the pressure sensor indicates we are in contact */
			Update_State(State, Covar, Pressure, actuator_midway_position, Measurement_Covar);/*Use the previously stored midway position */
		/* Now that the EKF has been run, we can use the current EKF state to solve for a target position, given our setpoint pressure */
		if(chMBFetch(&Pressures_Setpoint, (msg_t*)&Setpoint, TIME_IMMEDIATE) == RDY_OK)
			Target = State[1] + (Setpoint / State[0]) ;
		else
			Target = State[1];	/* If we arent getting any data, set the Target to the point where we are just touching the target */
		/* Perform motor driver processing, places results into mailbox fifo */
		position=Actuator_Position;	/* Store the position variable from the GPT callback (thread safe on 32bit architectures) */
		velocity=Actuator_Velocity;	/* Same for the velocity - this will be valid data only at the END of the current GPT bin */
		prior_velocity=velocity;	/* The initial velocity is stored for reference */
		actuator_midway_position=position;/* Initialise this */
		/* Apply software end stops */
		/* Note that the EKF position is not in real space - it can drift, so we use pot */
		end_position = ( pot_position + Target - position + velocities[3] );/* We add on the final velocity as pot is measured at end of 3rd GPT */
		if( end_position > ACTUATOR_LIMIT_PLUS )/* To extropolate the end position after moving to position Target */
			Target -= end_position - ACTUATOR_LIMIT_PLUS;/* Adjust the Target - this may mean we never reach correct pressure */
		else if( end_position < ACTUATOR_LIMIT_MINUS )
			Target += ACTUATOR_LIMIT_MINUS - end_position;/* But at least we dont hit the end stop */
		/* Loop through the GPT bins, issuing the next 4 instructions (GPT callback at 400hz) */
		for(index=0;index<5;index++) {	/* Note that there are 5 iterations - we cover the 4th timebin twice allowing backcorrection scheduling*/
			delta=Target-position;	/* The travel distance to target - defined starting from next GPT callback */
			if( abs(delta)<Actuator->DeadPos && abs(velocity)<Actuator->DeadVel ) {/* Position,Veloctiy deadband for our hardware */
				velocity=0;	/* If the GPT callback reads zero velocity it knows to de-energize the motor */
			}
			else {			/* We cannot enter the deadband */
				if(signbit(velocity)==signbit(delta)) {/* If we are moving towards the target */
					float stop_distance=(velocity^2)/(2*Actuator->MaxAcc);/* This is the minimum stopping distance for the hardware */
					if( abs(delta) > (stop_distance+Actuator->StopMargin){/* We have chance to stop with some margin */
						if(!signbit(delta))/* Apply maximum acceleration in the correct direction */
							velocity+=Actuator->MaxAcc;
						else
							velocity-=Actuator->MaxAcc;/* This is the velocity for the next GPT bin */
						if( abs(velocity)>Actuator->MaxVel )/* We are above the max speed - enforce limits on speed */
							velocity=signbit(delta)?-Actuator->MaxVel:Actuator->MaxVel;
					}
					else {	/* We are unable to stop without overshooting the target - try to backcorrect if scheduling allows */
						if(index) {/* If the current bin is not the first, we may be able to backcorrect*/
							float overshoot_velocity = sqrtf( 2*Actuator->MaxAcc*( abs(delta)-stop_distance ) );/* Excess Vel*/
							float first_acc = velocity-prior_velocity;/* The acceleration over previous GPT timestep */
							first_acc -= signbit(delta)?overshoot_velocity:-overshoot_velocity;/* Correct this accel */
							if( abs(first_acc)>Actuator->MaxAcc ) {/* Our adjusted acceleration exceeds limited */
								first_acc = signbit(delta)?Actuator->MaxAcc:-Actuator->MaxAcc;
							}
							velocity = prior_velocity + first_acc;/* Backapply the acceleration */
							velocities[index-1] = velocity;/* Backapply velocity */
						}
						velocity-=signbit(delta)?Actuator->MaxAcc:-Actuator->MaxAcc;/* Try our best to slow down*/
					}
				}
				else {		/* If we are moving away from the target */
					velocity+=signbit(delta)?Actuator->MaxAcc:-Actuator->MaxAcc;/* Try our best to head towards target*/
				}
			}
			if(index<4) {
				velocities[index]=velocity;
				if(!index)
					actuator_midway_position+=velocity;/* Store the midway position to eliminate lag from the EKF */
				else if(index==1)
					actuator_midway_position+=velocity/2;/* The ADC sampling interval lasts just under three GPT intervals (3/2=1.5)*/
			}
			prior_velocity=velocity;/* Store this for reference */
		}
		for(index=0;index<4;index++)
			chMBPost(&Actuator_Velocities, *((msg_t*)&velocities[index]), TIME_IMMEDIATE);/* Non blocking write attempt to GPT motor driver */
		chMBPost(&Pressures_Reported, *((msg_t*)&Pressure), TIME_IMMEDIATE);/* Non blocking write attempt to the Reported Pressure mailbox FIFO */
	}
}

