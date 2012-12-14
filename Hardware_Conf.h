/* Hardware configuration */


/* Pressure sensor related */
#define SENSOR_DIAMETER 14	/* 14mm diameter sensor */
#define SENSOR_GAIN (-1.0/68.01)/* Honeywell sensor calibrated to Newtons of force */

#define PASCALS_2_PSI (1.0/6894.75729)/* Convertion constant to convert pascals to PSI */

#define SENSOR_DIAMETER_ (SENSOR_DIAMETER/1000.0)/* Units origonally in mm */ 
#define SENSOR_AREA (SENSOR_DIAMETER_*SENSOR_DIAMETER_*M_PI/4.0)/* Area in square meters */
#define DIFF_GAIN_PASCALS (SENSOR_GAIN/SENSOR_AREA)/* Sensor area needs to be taken into account when calculating pressure */
#define DIFF_GAIN (PASCALS_2_PSI*DIFF_GAIN_PASCALS)/* Need to multiply be this constant to convert to PSI from ADC delta value */

#define PRESSURE_MARGIN 0.25/*0.075*/	/* An estimate of 3 sigma noise, ie rms noise * 3 */

/* ADC2 - pressure sensor related */

#define PRESSURE_ADC_NUM_CHANNELS 1
#define ADC_PRESSURE_CHANNEL ADC_CHANNEL_IN14

/* ADC2 - pot readout from linear actuator related*/

#define POT_ADC_NUM_CHANNELS 1
#define ADC_POT_CHANNEL ADC_CHANNEL_IN11

/* Solenoid PWM control related */

#define PWM_CHANNEL_SOLENOID	0
#define PWM_Driver_Solenoid	PWMD1/* Solenoid uses Timer1 channel1 */

/* Experimental Servo control related */

#define PWM_Driver_Servo	PWMD1/* Servo uses Timer1 channel2 */
#define PWM_CHANNEL_SERVO	1
#define USE_SERVO		/* Comment out to disable this */

/* Stepper motor + leadscrew based linear actuator related */

#define LEADSCREW_PITCH		4.0/* Units of mm in all the linear actuator code */
#define STEPS_PER_ROTATION	(360.0/1.8)
#define ACTUATOR_LENGTH		30.0/* A haxored firgelli L12 actuator */
#define INVERSE_POT		/* Define this if high voltage means retracted */

#define TIMER1_CLK		1000000.0/* Tim1 is clocked at f_cpu/2/84 */

#define STEPPER_PORT		GPIOE
#define STEPPER_DIR_PIN		13
#define STEPPER_EN_PIN		10

#define STEPPER_INVERTED_DIR	/* Use this to reverse stepper direction */

/* PPG related */

/* PPG ADC */
#define ADC_PPG_CHANNEL ADC_CHANNEL_IN15

/* PPG LED timer config - maps to the channels as numbered on the sensor */
//sensor channel, connector channel, OC, Tim
//	1		3	     4	 2	
//	2		4	     3	 4
//	3		1	     2	 5
//	4		5	     3	 3
//	5		2	     1	 9

#define Set_PWM_0(compare) TIM2->CCR4=compare
#define Set_PWM_1(compare) TIM4->CCR3=compare
#define Set_PWM_2(compare) TIM5->CCR2=compare
#define Set_PWM_3(compare) TIM3->CCR3=compare
#define Set_PWM_4(compare) TIM9->CCR1=compare

#define Get_PWM_0()  TIM2->CCR4
#define Get_PWM_1()  TIM4->CCR3
#define Get_PWM_2()  TIM5->CCR2
#define Get_PWM_3()  TIM3->CCR3
#define Get_PWM_4()  TIM9->CCR1

#define SPARE_TIMER TIM10	/* Extra timer used as spare gating source */

/* Gate output config - this is all internal */

#define CHAN1_GATE_OUTPUT	3
#define CHAN2_GATE_OUTPUT	2
#define CHAN3_GATE_OUTPUT 	1
#define CHAN4_GATE_OUTPUT	2

/* TS slave mode internal trigger source config for the slave timers */

#define TIM4_TS		1
#define TIM3_TS		2
#define TIM5_TS		2
#define TIM9_TS		1

/* General thread related */

#define MAILBOX_SIZE 128

#define PRESSURE_PROFILE_LENGTH_MS 15000
#define NUMBER_SETPOINTS 2
