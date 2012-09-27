#include "PPG_Demod.h"
#include "Hardware_Conf.h"
#include "PID_Pressure.h"

#include "ch.h"
#include "hal.h"

/* The mailboxes for inter thread communication, and their associated buffers */

/*
 * Mailboxes and buffers for the PPG output from thread
 */
Mailbox PPG_Demod[PPG_CHANNELS];
static msg_t PPG_Demod_Buff[PPG_CHANNELS][MAILBOX_SIZE];

/*
 * Mailboxes and buffers for the reported pressure output from this thread - syncronised with PPG data
 */
Mailbox Pressures_Output;
static msg_t Pressures_Output_Buff[MAILBOX_SIZE];

/*
 * Working area for this thread
*/
static WORKING_AREA(waThreadPPG, 1024);

/*
 * Thread pointer used for thread wakeup processing
*/
static Thread *tp = NULL;

/*
*  Samples buffer for photodiode ADC
*/
static adcsample_t PPG_Sample_Buffer[ADC_BUFF_SIZE];

/**
  * @brief  This function spawns the pressure control thread
  * @param  void* to a PID Loop configuration
  * @retval thread pointer to the spawned thread
  */
Thread* Spawn_PPG_Thread(void) {
	/*
	* Creates the mailbox buffers associated with this thread.
	*/
	for(uint8_t n=0; n<PPG_CHANNELS; n++)
		chMBInit(&PPG_Demod[n], &PPG_Demod_Buff[n][0], MAILBOX_SIZE);//PPG output
	chMBInit(&Pressures_Output, Pressures_Output_Buff, MAILBOX_SIZE);//Pressure data Out
	/*
	* Creates the thread. Thread has priority slightly above normal and takes no argument
	*/
	return chThdCreateStatic(waThreadPPG, sizeof(waThreadPPG), NORMALPRIO+2, PPG_Thread, (void*)NULL);
}

//ADC callback functions, adccallback as adc Convert wakes up the thread

static void adc1callback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
	/* Wakes up the thread.*/
	chSysLockFromIsr();
	if (tp != NULL) {
		tp->p_u.rdymsg = (buffer==PPG_Sample_Buffer? (msg_t)1 : (msg_t)0 );/* Sending the message, gives buffer index.*/
		chSchReadyI(tp);
		tp = NULL;
	}
	chSysUnlockFromIsr()
}


static void adc1errorcallback(ADCDriver *adcp, adcerror_t err) {
  (void)adcp;
  (void)err;
}

//The ADC1 configuration for the PPG monitoring
/*
 * ADC conversion group.
 * Mode:        Circular buffer, ADC_BUFFER_SIZE/2 samples of 1 channel, continuous triggered.
 * Channels:    IN11.
 */
static const ADCConversionGroup adcgrpcfg2 = {
  TRUE,
  1,
  adc1callback,
  adc1errorcallback,
  0,                        /* CR1 */
  ADC_CR2_CONT|ADC_CR2_SWSTART,/* CR2 */
  ADC_SMPR1_SMP_AN11(ADC_SAMPLE_3),
  0,                        /* SMPR2 */
  ADC_SQR1_NUM_CH(1),
  0,                        /* SQR2 */
  ADC_SQR3_SQ1_N(ADC_PPG_CHANNEL)
};

/**
  * @brief  This is the PPG thread
  * @param  void* unused
  * @retval msg_t status
  */
msg_t PPG_Thread(void *arg) {			/* Initialise as zeros */
	msg_t Pressure_Read, Pressure_Read_, msg;/* Used to read the pressure buffer */
	chRegSetThreadName("PPG");
	/*
	* Activates the ADC1 driver
	*/
	adcStart(&ADCD1, NULL);
	/*
	* Starts an ADC1 continuous conversion.
	*/
	adcStartConversion(&ADCD1, &adcgrpcfg2, PPG_Sample_Buffer, ADC_BUFF_SIZE);
	/* Loop for the PPG thread */
	while(TRUE) {
		/* Sleep until we are awoken by the ADC DMA ISR */
		/* Waiting for the IRQ to happen.*/
		chSysLock();
		tp = chThdSelf();
		chSchGoSleepS(THD_STATE_SUSPENDED);
		msg = chThdSelf()->p_u.rdymsg;  /* Retrieving the message, optional.*/
		chSysUnlock();
		/* Perform processing, places results into mailbox fifo */
		PPG_LO_Filter( msg?(volatile uint16_t*) PPG_Sample_Buffer:(volatile uint16_t*) &PPG_Sample_Buffer[ADC_BUFF_SIZE/2], PPG_Demod );
		/*
		/ Now we spit out the pressure data into the output mailbox fifos
		*/
		//The pressure is at 100Hz - slightly slower than the PPG samples, so we dont always get a sample
		if(chMBFetch(&Pressures_Reported, (msg_t*)&Pressure_Read_, TIME_IMMEDIATE) == RDY_OK)
			Pressure_Read=Pressure_Read_;
		chMBPost(&Pressures_Output, (msg_t)Pressure_Read, TIME_IMMEDIATE);/* Non blocking write attempt to the Reported Pressure mailbox FIFO */
	}
}
