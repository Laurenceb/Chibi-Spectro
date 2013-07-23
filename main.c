/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011 Giovanni Di Sirio.
 
    This file is part of ChibiOS/RT.
 
    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.
 
    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
 
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <stdio.h>
#include <string.h>
#include <math.h>
 
#include "ch.h"
#include "hal.h"

#include "usb_cdc.h"
#include "chprintf.h"

#include "usbcfg.h"
#include "Hardware_Conf.h"
#include "EKF_Pressure.h"
#include "PPG_Demod.h"
//#include "Scanf.h"
#include "Timer.h"
#include "Stepper.h"

#define ORANGE_LED 6

/* Virtual serial port over USB.*/
static SerialUSBDriver SDU1;

/*
 * This is a periodic thread that does absolutely nothing except flashing
 * a LED.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {
 
  (void)arg;
  chRegSetThreadName("Blinker");
  while (TRUE) {
    palSetPad(GPIOD, GPIOD_LED3);       /* Orange.  */
    chThdSleepMilliseconds(500);
    palClearPad(GPIOD, GPIOD_LED3);     /* Orange.  */
    chThdSleepMilliseconds(500);
  }
}

/*
 * Application entry point.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  SYSCFG->PMC = 0;
  usbDisconnectBus(serusbcfg.usbp);
  halInit();
  /* The LED PWM - do this here so the config is atomic */
  Setup_PPG_PWM();

  chSysInit();

  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  //usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1000);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);
  /* Wait for USB to connect */
  while(SDU1.config->usbp->state != USB_ACTIVE) {chThdSleepMilliseconds(20);}

  BaseSequentialStream *USBout = (BaseSequentialStream *)&SDU1;
  BaseChannel *USBin = (BaseChannel *)&SDU1;

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /* The pressure control structure with default actuator setup - approx Hardware limits*/
  Actuator_TypeDef Our_Config = { .MaxAcc=400, .MaxVel=50, .LimitPlus=(ACTUATOR_LENGTH*5)/6,\
   .LimitMinus=ACTUATOR_LENGTH/6, .DeadPos=0.005, .DeadVel=16, .BackLash=0.05 };
  /* Explanatory text for terminal interface */
  const uint8_t instruction_string[]="\r\nFormat is profile: (Triangle=1, Pulse=2, Dual Pulse=3, Const pressure=4, PPG Cal=5),\r\n Period:\
  time in seconds,\r\n Peak pressure: PSI\r\n";
  /* Variables for dumping data */
  //For pressure setting
  //float pressure_setpoints[NUMBER_SETPOINTS];
  float pressure_set_array[PRESSURE_PROFILE_LENGTH_MS/PRESSURE_TIME_INTERVAL]={};
  //end pressure
  float pressure,target;
  uint32_t ppg[PPG_CHANNELS],iterations=0,n=0;
  /* A bit of debug info here */
  chprintf(USBout, "Firmware compiled %s, running ChibiOS\r\n",__DATE__);
  chprintf(USBout, "core free memory : %u bytes\r\n", chCoreStatus());
  chprintf(USBout, "Data format: Time, Pressure (PSI), PPG channels 1,2,...\r\n");
  chprintf(USBout, "\r\n\r\nEnter config (whitespace separated) or press enter to use default (10s timeout)\r\n");
  chprintf(USBout, instruction_string);
  /* Try and read input over usb */
  uint8_t scanbuff[255]={};//Buffer for input data
  uint8_t numchars=0, timeout=1, valid_string=1;
  int test=-1;
  float per=30,peak=1.0;
  uint8_t current_ppg_channel=0;	//Used for the PPG Test/Calibrate mode
  do {
	  do {
	  	uint8_t a=chnReadTimeout(USBin, &scanbuff[numchars], sizeof(scanbuff), MS2ST(100));//100ms second timeout
		if(a) {
			timeout=0;
			chprintf(USBout, "%s", &scanbuff[numchars]);//Echo the typed characters
		}
		numchars+=a;
		if(timeout)		//Only continue with timeout if we haven't got any data
			timeout++;
	  } while(timeout<100 && scanbuff[numchars-1]!='\r' && scanbuff[numchars-1]!='\n');//Loop until newline or timeout with nothing
	  if(timeout!=100) {
	  	valid_string=sscanf((const char*)scanbuff, "%d %f %f", &test, &per, &peak);//scanf for setpoints input
	  	chprintf(USBout, "\r\n Read:%d", test);
	  	chprintf(USBout, ", %3f", (float)per);
	  	chprintf(USBout, ", %3f\r\n", (float)peak);
	  	//TODO:  autobrightness config ?
	 	if(valid_string!=3 || (test>5 || test<-1)) {
			chprintf(USBout,"Invalid config, format is: \r\n");//Message to user
  			chprintf(USBout, instruction_string);
			valid_string=0;
			numchars=0;
		}
  	}
  } while(valid_string==0);		//We loop here until string is valid
  if((test==2 && per>15.0) || (test!=2 && per>30.0) ) {
  	chprintf(USBout,"Period is invalid, using default period\r\n");
	per=(test==2)?15.0:30.0;
  }
  uint16_t itr = ((float)per*1000.0/(float)PRESSURE_TIME_INTERVAL);//Number of pressure setpoints sent to the control loop
  if(itr*sizeof(float)>sizeof(pressure_set_array)) {
  	chprintf(USBout,"Setpoint length too long, using default\r\n");
	itr=sizeof(pressure_set_array)/sizeof(float);
  }
  /* At present we just have a 1/3 pulse at end of each period */
  for(uint16_t n=0;n<itr;n++) {
	if(test>=4) {
		pressure_set_array[n]=peak;//The PPG Cal command just uses a constant pressure
	}
	if(test==3) {
		if((itr-n)<itr/6)
			pressure_set_array[n]=peak;
		else if((itr-n)<itr/2)
			pressure_set_array[n]=0.5;//At present these two release pressures are hardcoded
		else if((itr-n)<2*itr/3)
			pressure_set_array[n]=peak;
		else
			pressure_set_array[n]=0.2;
	}
	if(test==2) {
		if((itr-n)<itr/3)
			pressure_set_array[n]=peak;
		else
			pressure_set_array[n]=0.2;
	}
	if(test==-1 || test==1) {
		if(n<(itr/2))
			pressure_set_array[n]=2.0*peak*(float)n/(float)itr;//sweep from 0psi to setpoint
		else
			pressure_set_array[n]=2.0*peak*(1.0-(float)n/(float)itr);
	}
  }
  /* Populate our pressure control struct*/
  //Done in declaration atm
  /* Create the Pressure thread */
  Spawn_Pressure_Thread((void*)&Our_Config);
  /* Create the PPG thread */
  Spawn_PPG_Thread();
  /* Turn on the PPG LEDs here */
  Enable_PPG_PWM(0xFF);
  /* Loop to park the sensor on target with a 0.2PSI application pressure */
  peak=0.2;
  do {
	while(chMBPost(&Pressures_Setpoint, *(msg_t*) &peak, TIME_IMMEDIATE) == RDY_OK);/* Post to the pressure controller - fill its buffer*/
	chMBFetch(&Pressures_Output, (msg_t*) &pressure, TIME_INFINITE);/* Reads back the pressure - blocks waiting for PPG thread */
  } while(fabs(pressure-0.2)>0.05 || !pressure);/* Pressure is returned as zero when pressure controller is booting up */
  /* Set the brightness once on start up - TODO impliment it later on command */
  PPG_Automatic_Brightness_Control();	//Whilst this is running, the filled pressure setpoint buffer will supply the pressure controller
  /* After the brightness control has completed, print out the PWM fractions for reference */
  chprintf(USBout, "Autobrightness:\r\n");
  chprintf(USBout, "%3f",Get_PWM_0());
#if PPG_CHANNELS>1
  chprintf(USBout, ", %3f",Get_PWM_1());
#if PPG_CHANNELS>2
  chprintf(USBout, ", %3f",Get_PWM_2());
#if PPG_CHANNELS>3
  chprintf(USBout, ", %3f",Get_PWM_3());
#if PPG_CHANNELS>4
  chprintf(USBout, ", %3f",Get_PWM_4());
#endif
#endif
#endif
#endif
  chprintf(USBout, "\r\n");
  /* Flush the buffers to align the data (pressure fifo from pressure controller to ppg thread is emptied by itself) */
  chMBReset(&Pressures_Output);		
  for(uint8_t n=0; n<PPG_CHANNELS; n++)
	chMBReset( &PPG_Demod[n]);	//Flush PPG data output
  /*
   * main() thread activity;
   * wait for mailbox data in a loop and dump it to usb
   */
  while (TRUE) {
	while(1) {
		if(chMBPost(&Pressures_Setpoint, *(msg_t*)&pressure_set_array[n], TIME_IMMEDIATE)==RDY_OK) {
			n++;
			if(n==itr) {
				n=0;	//Loop around to start of buffer
			}
		}
		else
			break;		//Break once the mailbox fifo is filled
		chThdSleepMilliseconds(1);
	}
	chMBFetch( &Pressures_Output, (msg_t*) &pressure, TIME_INFINITE);//Waits for data to be posted
	chMBFetch( &Targets_Reported, (msg_t*) &target, TIME_INFINITE);
	for(uint8_t n=0; n<PPG_CHANNELS; n++)
		chMBFetch( &PPG_Demod[n], (msg_t*) &ppg[n], TIME_INFINITE);//Waits for data to be posted
	chprintf(USBout, "%3f,", (float)iterations/PPG_SAMPLE_RATE);
	chprintf(USBout, "%3f,", pressure);
	chprintf(USBout, "%3f", target);
	for(uint8_t n=0; n<PPG_CHANNELS; n++)
		chprintf(USBout, ",%lu", ppg[n]);
	chprintf(USBout, "\r\n");
	chThdSleepMilliseconds(2);
	iterations++;
	if(test==5) {			//If we have PPG Cal mode, we need to swap between individual PPG channels so that they can be tested
		if(!(iterations%200)) {	//This runs every 200 iterations
			if(++current_ppg_channel>=PPG_CHANNELS)
				current_ppg_channel=0;
			Enable_PPG_PWM(0x01<<current_ppg_channel);//Only one channel is enabled at any given time
		}
	}	
  }
}


