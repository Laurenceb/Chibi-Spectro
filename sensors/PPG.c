#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "ch.h"

#include "PPG.h"

/* This is used for direct read of the output data - should be thread safe as seperate 32bit variables */
volatile float Last_PPG_Values[PPG_CHANNELS];

/**
  * @brief  Run the baseband LO on the quadrature samples, then integrate and dump the baseband into indivdual LED bins
  * @param  Pointer to raw ADC data, Pointer array to the output data buffers
  * @retval None
  * This will be called at 2800Hz
  */
void PPG_LO_Filter(volatile uint16_t* Buff, Mailbox Output_Mailbox[PPG_CHANNELS]) {
	static uint8_t bindex;			//Baseband decimation index
	static int32_t Frequency_Bin[PPG_CHANNELS][2];//All Frequencies in use - consisting of and I and Q component
	static const int8_t sinusoid[120]=STEP_SIN,cosinusoid[120]=STEP_COS;//Lookup tables
	static uint8_t m=0;
	int32_t I=0,Q=0;			//I and Q integration bins
	for(uint16_t n=0;n<ADC_BUFF_SIZE/2;n++) {//Loop through multiplying by the LO
		I+=(int16_t)Buff[n]*(int16_t)cosinusoid[m];
		Q+=(int16_t)Buff[n]*(int16_t)sinusoid[m];
		if(m++>=120)			//There are 120 samples
			m=0;
	}
	//Now run the "baseband" decimating filter(s)
	//Positive frequencies
	#if PPG_CHANNELS>=4
	PPG_Frequency_Bin_Rotate(&Frequency_Bin[3][0],1);
	#endif
	#if PPG_CHANNELS>=2
	if(bindex&0x01) {			//This happens at half the rate of the outpermost bins
		PPG_Frequency_Bin_Rotate(&Frequency_Bin[1][0],1);
	}
	#endif
	//Zero frequency - i.e. directly on quadrature so nothing to do to this bin
	//Negative frequencie(s) go here, need to get to 0hz, so multiply bin by a +ive complex exponential
	#if PPG_CHANNELS>=3
	if(bindex&0x01) {			//This happens at half the rate of the outpermost bins
		PPG_Frequency_Bin_Rotate(&Frequency_Bin[2][0],0);
	}
	#endif
	#if PPG_CHANNELS>=5
	PPG_Frequency_Bin_Rotate(&Frequency_Bin[4][0],0);
	#endif
	#if PPG_CHANNELS>5 || !PPG_CHANNELS
	#error "Unsupported number of channels - decoder error"
	#endif
	//Add the I and Q directly into the bins
	for(uint8_t n=0;n<PPG_CHANNELS;n++) {
		Frequency_Bin[n][0]+=I;Frequency_Bin[n][1]+=Q;//I,Q is real,imaginary
	}
	//End of decimating filters
	if(++bindex==PPG_NO_SUBSAMPLES) {	//Decimation factor of 12 - 62.004Hz data output
		for(uint8_t n=0;n<PPG_CHANNELS;n++) {
			Last_PPG_Values[n]=sqrtf(((float)Frequency_Bin[n][0]*(float)Frequency_Bin[n][0])+((float)Frequency_Bin[n][1]*(float)Frequency_Bin[n][1]));
			chMBPostI(&Output_Mailbox[n], (msg_t)((uint32_t)Last_PPG_Values[n])>>8);//There will always be at least 8 bits on noise, so shift out the mess
		}				//Fill the array of buffers
		memset(Frequency_Bin,0,sizeof(Frequency_Bin));//Zero everything
		bindex=0;			//Reset this
	}
}

/**
  * @brief  Rotates a phasor by approximately 45 degrees using integer maths
  * @param  Pointer to the phasor array
  * @retval None - rotated in place
*/
void PPG_Frequency_Bin_Rotate(float Bin[2],uint8_t Direction) {
	int32_t a=Bin[0];
	if(Direction) {				// Positive bins
		Bin[0]=Bin[0]*7+Bin[1]*7;	//Rotate the phasor in the bin - real here (45degree rotation)
		Bin[1]=Bin[1]*9-a*4;		//complex here
		Bin[1]/=10;Bin[0]/=10;		//divide by 10
	}
	else {					// Negative bins
		Bin[0]=Bin[0]*7-Bin[1]*7;	//Rotate the phasor in the bin - real here (45degree rotation)
		Bin[1]=Bin[1]*9+a*4;		//complex here
		Bin[1]/=10;Bin[0]/=10;		//divide by 10
	}
}

/**
  * @brief  Corrects PWM values to get the ADC input in the correct range
  * @param  none
  * @retval none
*/
void PPG_Automatic_Brightness_Control(void) {
	uint8_t channel;
	uint16_t vals[3]={0,0,0};
	uint16_t old_vals[3];			//This function iterates until the PWM duty correction falls below a limit set in header
	do {
		memcpy(old_vals,vals,sizeof(old_vals));//Copy over to the old values
		for(channel=0;channel<PPG_CHANNELS;channel++) {	//Loop through the channels
			uint16_t pwm=Get_PWM_0();
			if(channel==1)
				pwm=Get_PWM_1();
			else if(channel==2)
				pwm=Get_PWM_2();//Retreives the set pwm for this channel
			vals[channel]=PPG_correct_brightness((uint32_t)Last_PPG_Values[channel], pwm);
		}
		//Apply the pwm duty correction here
		Set_PWM_2(vals[2]);
		Set_PWM_1(vals[1]);
		Set_PWM_0(vals[0]);
		chThdSleepMilliseconds((uint32_t)(4000.0/PPG_SAMPLE_RATE));//Delay for a period of 4 PPG samples to let the analogue stabilise	
	}while((abs(vals[0]-old_vals[0])>old_vals[0]/PWM_STEP_LIM)||(abs(vals[1]-old_vals[1])>old_vals[1]/PWM_STEP_LIM)||\
		(abs(vals[2]-old_vals[2])>old_vals[2]/PWM_STEP_LIM));
}




/**
  * @brief  Output a corrected PWM value to get the ADC input in the correct range
  * @param  Output sample from the decimator, present PWM duty cycle value
  * @retval A new corrected duty cycle value
  * This will be called from the main code between pressure applications and timed refills
  * If more leds are added at different pwm frequencies, then we need to take the sum of Decimated values and scale
  * To avoid clipping of the frontend
  */
uint16_t PPG_correct_brightness(uint32_t Decimated_value, uint16_t PWM_value) {
	//2^adc_bits*samples_in_half_buffer/4*baseband_decimator
	//(2^12)*(64/4)*21 == 1376256 == 2*target_decimated_value TODO impliment this with macros full - atm just TARGET_ADC
	float corrected_pwm=PWM_Linear(PWM_value);
	corrected_pwm*=(float)(TARGET_ADC)/(float)Decimated_value;//This is the linearised pwm value required to get target amplitude
	corrected_pwm=(corrected_pwm>1.0)?1.0:corrected_pwm;//Enforce limit on range to 100%
	return ((asinf(corrected_pwm)/M_PI)*PWM_PERIOD_CENTER);//Convert back to a PWM period value
}

/**
  * @brief  Output a linearised value in range 0 to 1 from a PWM duty cycle
  * @param  PWM duty cycle
  * @retval A linearised value as a float in the range 0 to 1
  */
float PWM_Linear(uint16_t PWM_value) {
	return sinf(((float)PWM_value/(float)PWM_PERIOD_CENTER)*M_PI);//returns the effecive sinusoidal amplitude in range 0-1
}
