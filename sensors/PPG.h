#include "ch.h"

#include "Timer.h"

#define PPG_CHANNELS 5

#define PPG_NO_SUBSAMPLES 16
//#define TARGET_ADC 1376256/(3*PPG_CHANNELS)/*Target 67% of ADC range used by the pwm led signal*/
#if PPG_CHANNELS==1
	#define TARGET_ADC 231612662UL*2/(3)/*Target 67% of ADC range used by the pwm led signal*/
#else
	#define TARGET_ADC 231612662UL*2/(3*3/*sqrt(PPG_CHANNELS+2)*/)/*This equation is true for arbitrary number of channels with correct phasing*/
#endif

#define ADC_BUFF_SIZE 1500		/* 1500 sample buffer for DMA (120*100/(8*2)) */

#define PWM_STEP_LIM  100		/* A 1% or less shift per iteration causes termination */

#define PPG_SAMPLE_RATE (float)116.66667/* 168000000/2/4/15/120/100 */

#define STEP_SIN {1,2,2,3,4,5,5,6,7,7,8,9,9,10,11,11,12,12,13,13,13,14,14,14,14,15,15,15,15,15,15,15,15,\
		  15,14,14,14,14,13,13,13,12,12,11,11,10,9,9,8,8,7,6,5,5,4,3,2,2,1,0,-1,-2,-2,-3,-4,-5,\
		  -5,-6,-7,-7,-8,-9,-9,-10,-11,-11,-12,-12,-13,-13,-13,-14,-14,-14,-14,-15,-15,-15,-15,-15,\
		  -15,-15,-15,-15,-14,-14,-14,-14,-13,-13,-13,-12,-12,-11,-11,-10,-9,-9,-8,-8,-7,-6,-5,-5,\
		  -4,-3,-2,-2,-1,-0,1,2,2,3,4,5,5,6,7,7,8,9,9,10,11,11,12,12,13,13,13,14,14,14,\
		  14,15,15,15,15,15}

extern volatile float Last_PPG_Values[PPG_CHANNELS];/* Last values from the PPG decoders, useful for brightness control */

void PPG_LO_Filter(volatile uint16_t* buff, Mailbox *Output_Mailbox) __attribute__ ((section (".ccmram")));;
void PPG_Frequency_Bin_Rotate(int32_t Bin[2], uint8_t Direction) __attribute__ ((section (".ccmram")));;
void PPG_Automatic_Brightness_Control(void);
uint16_t PPG_correct_brightness(uint32_t Decimated_value, uint16_t PWM_value);
float PWM_Linear(uint16_t PWM_value);
