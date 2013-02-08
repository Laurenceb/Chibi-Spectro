#include <math.h>
#include <stdint.h>
#include <string.h>
#include "EKF_Pressure.h"
#ifdef EKF_NONLINEAR
#include "EKF_Estimator_Nonlin.h"

//The state vector is: Pressure, K_1, K_2
//where Delta_P = K_1*Delta_X + K_2*Delta_X*Pressure
//Note that we define matrices as being in Row, Column order

/**
  * @brief  This function runs an update on the EKF
  * @param  EKF state and coveriance, measured force (PSI), measurement mean squared error (PSI^2)
  * @retval None, EKF updated in place
  */
void Update_State(float State[3], float Covar[3][3], float Measurement, float Measurement_Covar) {
	float y = Measurement - State[0];
	float s = Covar[0][0] + Measurement_Covar;
	y/=s;
	State[0] += Covar[0][0]*y; State[1] += Covar[1][0]*y; State[2] += Covar[2][0]*y;//State update
	float Covar_Old[3][3];
	memcpy(Covar_Old, Covar, sizeof(Covar_Old) );//Copy over the old Covar
	Covar[0][0] -= Covar_Old[0][0]*Covar_Old[0][0]/s;	//Now we can update the Covar in place
	Covar[0][1] -= Covar_Old[0][0]*Covar_Old[0][1]/s;	
	Covar[0][2] -= Covar_Old[0][0]*Covar_Old[0][2]/s;	
	Covar[1][0] -= Covar_Old[1][0]*Covar_Old[0][0]/s;	
	Covar[1][1] -= Covar_Old[1][0]*Covar_Old[0][1]/s;	
	Covar[1][2] -= Covar_Old[1][0]*Covar_Old[0][2]/s;	
	Covar[2][0] -= Covar_Old[2][0]*Covar_Old[0][0]/s;	
	Covar[2][1] -= Covar_Old[2][0]*Covar_Old[0][1]/s;	
	Covar[2][2] -= Covar_Old[2][0]*Covar_Old[0][2]/s;	
}

/**
  * @brief  This function runs a predict on the EKF
  * @param  EKF state and covariance
  * @retval None, EKF updated in place
  */
void Predict_State(float State[3], float Covar[3][3], float Delta_Time, float Process_Noise[3], float Delta_Pos) {
	//Fist we predict the new state -State[1] and State[2] are unchanged
	State[0] += (State[1] + State[0]*State[2])*Delta_Pos;//Nonlinear elastic model
	float F[3];
	F[0] = 1+State[2]*Delta_Pos; F[1] = Delta_Pos; F[2] = State[0]*Delta_Pos;
	float PF[3];		//This is actuall the first column of the PF' product, the rest of PF' is just a copy of P
	PF[0] = Covar[0][0]*F[0] + Covar[0][1]*F[1] + Covar[0][2]*F[2];
	PF[1] = Covar[1][0]*F[0] + Covar[1][1]*F[1] + Covar[1][2]*F[2];
	PF[2] = Covar[2][0]*F[0] + Covar[2][1]*F[1] + Covar[2][2]*F[2];
	float Covar_Old[3][3];
	memcpy(Covar_Old, Covar, sizeof(Covar_Old) );//Copy over the old Covar
	Covar[0][0] = F[0]*PF[0] + F[1]*PF[1] + F[2]*PF[2] + Process_Noise[0]*Delta_Time;//Now we update the Covar in place
	Covar[0][1] = F[0]*Covar_Old[0][1] + F[1]*Covar_Old[1][1] + F[2]*Covar_Old[2][1];
	Covar[0][2] = F[0]*Covar_Old[0][2] + F[1]*Covar_Old[1][2] + F[2]*Covar_Old[2][2];
	Covar[1][0] = PF[1];
	Covar[1][1] += Process_Noise[1]*Delta_Time;
	Covar[2][0] = PF[2];	//Other Covar indices are left unchanged
	Covar[2][2] += Process_Noise[2]*Delta_Time;
}
#endif
