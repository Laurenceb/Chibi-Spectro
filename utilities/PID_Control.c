#include "PID_Control.h"

/**
  * @brief  This function runs an iteration of a PID controller and returns the result
  * @param  PID Loop configuration, PID Loop state, loop setpoint, input to loop, time step
  * @retval Float PID output
  */
float Run_PID_Loop(PID_Config *Loop_Config, PID_State *PID, float Setpoint, float Input, float Delta_Time) {
	float Error=Setpoint-Input;		/* Pressure_Setpoint is a global containing the target diff press */
	PID->I += Error*Loop_Config->I_Const*Delta_Time;/* Constants from the loop struct */
	if(PID->I > Loop_Config->I_Limit)	/* Enforce limits */
		PID->I = Loop_Config->I_Limit;
	if(PID->I < -Loop_Config->I_Limit)	/* Enforce limits */
		PID->I = -Loop_Config->I_Limit;
	float a=Loop_Config->P_Const*Error + PID->I + Loop_Config->D_Const*(Input-PID->Last_Input)/Delta_Time;
	PID->Last_Input=Input;
	return a;
}
