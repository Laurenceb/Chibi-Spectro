#include <math.h>

#include "PID_Pressure.h"
#include "Hardware_Conf.h"
#include "Pressure_Filter.h"
#include "Quickselect.h"
#include "PID_Control.h"
#include "Pressure.h"

#include "hal.h"

//The mailboxes for inter thread communication, and their associated buffers

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
static WORKING_AREA(waThreadPressure, 512);

/**
  * @brief  This function spawns the pressure control thread
  * @param  void* to a PID Loop configuration
  * @retval thread pointer to the spawned thread
  */
Thread* Spawn_Pressure_Thread(PID_Config *arg) {
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
 * Mode:        Linear buffer, 1 sample of 1 channel, SW triggered.
 * Channels:    IN11.
 */
static const ADCConversionGroup adcgrpcfg1 = {
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

//The PWM configuration for the solenoid control
static PWMConfig PWM_Config_Solenoid = {
  1000000,                                  			/* 1MHz PWM clock frequency. */
  250,                                  			/* Initial PWM period 4KHz. */
  NULL,								/* No cyclic callback */
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},				/* Have to define the channel to enable here */
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL},
   {PWM_OUTPUT_DISABLED, NULL}
  },
  0,
};


/**
  * @brief  This is the pressure control thread
  * @param  void* to a PID Loops configuration
  * @retval msg_t status
  */
msg_t Pressure_Thread(void *This_Config) {
	/* This thread is passed a pointer to a PID loop configuration */
	PID_State Pressure_PID_Controllers[((Pressure_Config_Type*)This_Config)->Number_Setpoints];//={};/* Initialise as zeros */
	float* Last_PID_Out=(float*)chHeapAlloc(NULL,sizeof(float)*((Pressure_Config_Type*)This_Config)->Number_Setpoints);/* PID output for interpol */
	adcsample_t Pressure_Samples[PRESSURE_SAMPLES],Pressure_Sample;/* Use multiple pressure samples to drive down the noise */
	float PID_Out,Pressure;
	uint32_t Setpoint=0;
	static uint8_t Old_Setpoint, Previous_Setpoint;
	chRegSetThreadName("PID_Pressure");
	//palSetGroupMode(GPIOC, PAL_PORT_BIT(5) | PAL_PORT_BIT(4), 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOE, 9, PAL_MODE_ALTERNATE(1));		/* Only set the pin as AF output here, so as to avoid solenoid getting driven earlier*/
	/*
	* Activates the PWM driver
	*/
	pwmStart(&PWM_Driver_Solenoid, &PWM_Config_Solenoid);	/* Have to define the timer to use for PWM_Driver in hardware config */
	/*
	* Set the solenoid PWM to off
	*/
	pwmEnableChannel(&PWM_Driver_Solenoid, (pwmchannel_t)PWM_CHANNEL_SOLENOID, (pwmcnt_t)0);
	/*
	* Activates the ADC2 driver *and the thermal sensor*.
	*/
	adcStart(&ADCD2, NULL);
	//adcSTM32EnableTSVREFE();
	/*
	/ Now we run the sensor offset calibration loop
	*/
	do {
		adcConvert(&ADCD2, &adcgrpcfg1, &Pressure_Sample, 1);/* This function blocks until it has one sample*/
	} while(Calibrate_Sensor((uint16_t)Pressure_Sample));
	systime_t time = chTimeNow();				/* T0 */
	systime_t Interpolation_Timeout = time;			/* Set to T0 to show there is no current interpolation */
	/* Loop for the pressure control thread */
	while(TRUE) {
		/*
		* Linear conversion.
		*/
		adcConvert(&ADCD2, &adcgrpcfg1, Pressure_Samples, PRESSURE_SAMPLES);/* This function blocks until it has the samples via DMA*/
		/*
		/ Now we process the data and apply the PID controller - we use a median filter to take out the non guassian noise
		*/
		Pressure_Sample = quick_select(Pressure_Samples, PRESSURE_SAMPLES);
		Pressure = Convert_Pressure((uint16_t)Pressure_Sample);/* Converts to PSI as a float */
		/* Retrieve a new setpoint from the setpoint mailbox, only continue if we get it*/
		if(chMBFetch(&Pressures_Setpoint, (msg_t*)&Setpoint, TIME_IMMEDIATE) == RDY_OK) {
			//Pressure=Run_Pressure_Filter(Pressure);/* Square root raised cosine filter for low pass with minimal lag */
			Pressure = Pressure<0?0.0:Pressure;	/* A negative pressure is impossible with current hardware setup - disregard*/
			Setpoint &= 0x000000FF;
			/* The controller is built around an interpolated array of independant PID controllers with seperate setpoints */
			if(Setpoint != Old_Setpoint) {		/* The setpoint has changed */
				Previous_Setpoint = Old_Setpoint;/* This is for use by the interpolator */
				Old_Setpoint = Setpoint;	/* Store the setpoint */
				/* Store the time at which the interpolation to new setpoint completes*/
				Interpolation_Timeout = time + (systime_t)( 4.0 / ((Pressure_Config_Type*)This_Config)->Interpolation_Base );
			}
			if(Interpolation_Timeout > time) {	/* If we have an ongoing interpolation  - note, operates in tick units */
				/* Value goes from 1 to -1 */
				float interpol = erff( (float)(Interpolation_Timeout - time) *\
						 ((Pressure_Config_Type*)This_Config)->Interpolation_Base - 2.0 );/* erf function interpolator */
				interpol = ( (-interpol + 1.0) / 2.0);/* Interpolation value goes from 0 to 1 */
				PID_Out = ( Last_PID_Out[Previous_Setpoint] * (1.0 - interpol) ) + ( Last_PID_Out[Setpoint] * interpol );
				Pressure_PID_Controllers[Setpoint].Last_Input = Pressure;/* Make sure the input to next PID controller is continuous */
			}
			else {
				PID_Out = Run_PID_Loop( ((Pressure_Config_Type*)This_Config)->PID_Loop_Config, &Pressure_PID_Controllers[Setpoint],\
						 (((Pressure_Config_Type*)This_Config)->Setpoints)[Setpoint], \
						 Pressure, (float)PRESSURE_TIME_INTERVAL/1000.0);/* Run PID */
				Last_PID_Out[Setpoint] = PID_Out;/* Store for use by the interpolator */
			}
		}
		else
			PID_Out=0;				/* So we can turn off the solenoid simply by failing to send Setpoints */
		PID_Out=PID_Out>1.0?1.0:PID_Out;
		PID_Out=PID_Out<0.0?0.0:PID_Out;		/* Enforce range limits on the PID output */
		/*
		/ Now we apply the PID output to the PWM based solenoid controller, and feed data into the mailbox output - Note fractional input
		*/
		pwmEnableChannel(&PWM_Driver_Solenoid, (pwmchannel_t)PWM_CHANNEL_SOLENOID, (pwmcnt_t)PWM_FRACTION_TO_WIDTH(&PWM_Driver_Solenoid, 1000\
														, (uint32_t)(1000.0*PID_Out)));	
		chMBPost(&Pressures_Reported, *((msg_t*)&Pressure), TIME_IMMEDIATE);/* Non blocking write attempt to the Reported Pressure mailbox FIFO */
		/*
		/ The Thread is syncronised to system time
		*/	
		time += MS2ST(PRESSURE_TIME_INTERVAL);		/* Next deadline */
		chThdSleepUntil(time);				/* Gives us a thread with regular timing */
	}
}
