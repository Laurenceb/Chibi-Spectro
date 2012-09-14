#pragma once

#define PID_BLANK {0.0,0.0}

typedef struct {
	float P_Const;
	float I_Const;
	float D_Const;
	float I_Limit;
} PID_Config;

typedef struct {
	float I;
	float Last_Input;
} PID_State;

float Run_PID_Loop(PID_Config *Loop_Config, PID_State *PID, float Setpoint, float Input, float Delta_Time);
