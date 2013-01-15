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
 
#include "ch.h"
#include "hal.h"

#include "usb_cdc.h"
#include "chprintf.h"

#include "usbcfg.h"
#include "Hardware_Conf.h"
#include "EKF_Pressure.h"
#include "PPG_Demod.h"
#include "Timer.h"
#include "Scanf.h"
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
  Actuator_TypeDef Our_Config = { .MaxAcc=400, .MaxVel=50, .LimitPlus=(ACTUATOR_LENGTH*5)/6, .LimitMinus=ACTUATOR_LENGTH/6, .DeadPos=0.005, .DeadVel=16 };
  /* Variables for dumping data */
  //For pressure setting
  //float pressure_setpoints[NUMBER_SETPOINTS];
  float pressure_set_array[PRESSURE_PROFILE_LENGTH_MS/PRESSURE_TIME_INTERVAL]={};
  //end pressure
  float pressure,target;
  uint32_t ppg[PPG_CHANNELS],iterations=0,loaded_setpoints=0,n=0;
  /* A bit of debug info here */
  chprintf(USBout, "Firmware compiled %s, running ChibiOS\r\n",__DATE__);
  chprintf(USBout, "core free memory : %u bytes\r\n", chCoreStatus());
  chprintf(USBout, "Data format: Time, Pressure (PSI), PPG channels 1,2,...\r\n");
  chprintf(USBout, "\r\n\r\nEnter config or press enter to use default (10s timeout)\r\n");
  //debug code
  /*Setup_Stepper_PWM();
  GPIO_Stepper_Enable(1);
  while(1){
	GPIO_Stepper_Dir(1);
	chThdSleepMilliseconds(500);
	SET_STEPPER_PERIOD(1667);
	chThdSleepMilliseconds(500);
	SET_STEPPER_PERIOD(4999);
	chThdSleepMilliseconds(500);
	GPIO_Stepper_Dir(0);
	chThdSleepMilliseconds(500);
	SET_STEPPER_PERIOD(1667);
	chThdSleepMilliseconds(500);
	SET_STEPPER_PERIOD(4999);
	chThdSleepMilliseconds(500);
  }*/
  /* Try and read input over usb */
  uint8_t scanbuff[255]={};//Buffer for input data
  uint8_t numchars=0, timeout=0, valid_string=1;
  double test;
  do {
	  do {
	  	uint8_t a=chnReadTimeout(USBin, &scanbuff[numchars], sizeof(scanbuff), MS2ST(100));//100ms second timeout
		if(a) {
			timeout=0;
			chprintf(USBout, "%s", &scanbuff[numchars]);//Echo the typed characters
		}
		numchars+=a;
		timeout++;
	  } while(timeout<100 && scanbuff[numchars-1]!='\r' && scanbuff[numchars-1]!='\n');//Loop until newline or timeout with nothing
	  sscanf(scanbuff, "%D", &test );//scanf will exentually allow setpoints input - TODO
	  chprintf(USBout, "\r\n Read:%f\r\n", (float)test );
	  //TODO: PID setpoints, pressure pulse sequences, autobrightness config
	  if(!valid_string)
		chprintf(USBout,"Invalid config, format is: \r\n");//Message to user
  } while(valid_string==0);		//We loop here until string is valid
  /* At present we just have a 5s pulse at end of each period */
  for(uint16_t n=0;n<sizeof(pressure_set_array)/sizeof(float);n++) {
	if((sizeof(pressure_set_array)/sizeof(float)-n)<500)
		pressure_set_array[n]=3.0;
	else if((sizeof(pressure_set_array)/sizeof(float)-n)<1500)
		pressure_set_array[n]=0.5;
	else if((sizeof(pressure_set_array)/sizeof(float)-n)<2000)
		pressure_set_array[n]=3.0;
	else
		pressure_set_array[n]=0.2;
  }
  //pressure_setpoints[0]=0.3;
  //pressure_setpoints[1]=2.5;
  /* Populate our pressure control struct*/
	//Done in declaration atm
  /* Create the Pressure thread */
  Spawn_Pressure_Thread((void*)&Our_Config);
  /* Create the PPG thread */
  Spawn_PPG_Thread();
  /* Turn on the PPG LEDs here */
  Enable_PPG_PWM();
  /* Wait for front end to stabilise and get some data */
  chThdSleepMilliseconds(50);
  /* Set the brightness once on start up - TODO inpliment it later */
  PPG_Automatic_Brightness_Control();
  /*
   * main() thread activity;
   * wait for mailbox data in a loop and dump it to usb
   */
  while (TRUE) {
	//TODO impliment pressure cycles using config data supplied over USB
	while(1) {
		if(chMBPost(&Pressures_Setpoint, *(msg_t*)&pressure_set_array[n], TIME_IMMEDIATE)==RDY_OK) {
			n++;
			if(n==sizeof(pressure_set_array)/sizeof(float)) {
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
  }
}


