#pragma once

#include "ch.h"
#include "PID_Control.h"

/* 100Hz pressure control loop */
#define PRESSURE_TIME_INTERVAL 10
#define PRESSURE_SAMPLES 200

extern Mailbox Pressures_Setpoint;
extern Mailbox Pressures_Reported;

Thread* Spawn_Pressure_Thread(PID_Config *arg);
msg_t Pressure_Thread(void *Loop_Config);
