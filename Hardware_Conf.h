


/* Pressure sensor related */

#define DIFF_GAIN (1.0/(1.2412*-5.7767*56.556))/*pressure sensor 2SMPP with 56.6 times instrumentation amp gain but negative factor */

/* ADC2 - pressure sensor related */

#define PRESSURE_ADC_NUM_CHANNELS 1
#define ADC_PRESSURE_CHANNEL ADC_CHANNEL_IN11

/* Solenoid PWM control related */

#define PWM_CHANNEL_SOLENOID	2
#define PWM_Driver_Solenoid	PWMD2

/* PPG related */

#define ADC_PPG_CHANNEL ADC_CHANNEL_IN6

/* General thread related */

#define MAILBOX_SIZE 128
