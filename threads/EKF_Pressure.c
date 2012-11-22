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
 * Working area for this thread
*/
static WORKING_AREA(waThreadPressure, 1024);

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
	/*
	* Creates the thread. Thread has priority slightly below normal and takes no argument
	*/
	return chThdCreateStatic(waThreadPressure, sizeof(waThreadPressure), NORMALPRIO+1, Pressure_Thread, (void*)arg);
}

//ADC callback functions, no adccallback as adc Convert wakes up the thread
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {
  (void)adcp;
  (void)err;
}

//The ADC2 configuration for the pressure monitoring
/*
 * ADC conversion group.
 * Mode:        Linear buffer, multiple sample of 1 channel, SW triggered.
 * Channels:    IN11.
 */
static const ADCConversionGroup adcgrpcfg2 = {
  FALSE,
  PRESSURE_ADC_NUM_CHANNELS,
  NULL,
  adcerrorcallback,
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  ADC_SMPR1_SMP_AN14(ADC_SAMPLE_480),
  0,                        /* SMPR2 */
  ADC_SQR1_NUM_CH(PRESSURE_ADC_NUM_CHANNELS),
  0,                        /* SQR2 */
  ADC_SQR3_SQ1_N(ADC_PRESSURE_CHANNEL)
};

static GPTConfig gpt8cfg =
{
    200000,					/* Timer clock.*/
    GPT_Stepper_Callback			/* Timer callback.*/
};


/**
  * @brief  This is the Pressure control thread
  * @param  void* arg - pointer to an actuator tydef holding actuator capability information
  * @retval msg_t status
  */
msg_t Pressure_Thread(void *arg) {		/* Initialise as zeros */
	msg_t Setpoint,msg;			/* Used to read the setpoint buffer and messages from GPT */
	uint8_t index=0;
	float velocities[4],velocity,prior_velocity,position,delta;
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
    	gptStartContinuous(&GPTD8, 500); // dT = 200,000 / 25 = 400Hz
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

		/* Perform motor driver processing, places results into mailbox fifo */
		position=Actuator_Position;	/* Store the position variable from the GPT callback (thread safe on 32bit architectures) */
		velocity=Actuator_Velocity;	/* Same for the velocity - this will be valid data only at the END of the current GPT bin */
		prior_velocity=velocity;	/* The initial velocity is stored for reference */
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
			if(index<4)
				velocities[index]=velocity;
			prior_velocity=velocity;/* Store this for reference */
		}
		for(index=0;index<4;index++)
			chMBPost(&Actuator_Velocities, *((msg_t*)&velocities[index]), TIME_IMMEDIATE);/* Non blocking write attempt to GPT motor driver */
	}
}

