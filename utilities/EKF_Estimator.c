#include <math.h>
#include <stdint.h>
#include "EKF_Pressure.h"
#ifndef EKF_NONLINEAR
#include "EKF_Estimator.h"

static void by2multiply(float out[2][2], float inone[2][2], float intwo[2][2]) {
	uint8_t n,m;
	for(n=0;n<2;n++) {
		for(m=0;m<2;m++)
			out[n][m]=inone[n][0]*intwo[0][m]+inone[n][1]*intwo[1][m];
	}
}

/**
  * @brief  This function runs an update on the EKF
  * @param  EKF state and coveriance, measured force (PSI), position (mm), measurement mean squared error (PSI^2)
  * @retval None, EKF updated in place
  */
void Update_State(float State[2], float Covar[2][2], float Measurement, float Position, float Measurement_Covar) {
	float p_ = ( Position - State[1] );			//y=z-h(x)
	float s_ = -State[0];					//H=(p_,s_)
	float y = Measurement - State[0] * p_;
	float s = p_ * ( Covar[0][0]*p_ + s_*Covar[0][1] ) + s_ * ( Covar[1][0]*p_ + s_*Covar[1][1] ) + Measurement_Covar;//S=HPH'+R
	y/=s;							//For the state update
	State[0] += (Covar[0][0]*p_ + Covar[0][1]*s_)*y;	//x+=PH'y/S
	State[1] += (Covar[1][0]*p_ + Covar[1][1]*s_)*y;
	float Covar_New[2][2];
	float HH[2][2];
	HH[0][0]=p_*p_/s;HH[0][1]=p_*s_/s;HH[1][0]=HH[0][1];HH[1][1]=s_*s_/s;//HH=H'H/s
	by2multiply(Covar_New,Covar,HH);			//Covar_New=PH'(H/s)==KH
	by2multiply(HH,Covar_New,Covar);			//HH=(PH'(H/s))P
	Covar[0][0] -= HH[0][0];				//P-=(P(H'/S)H)P
	Covar[0][1] -= HH[0][1];
	Covar[1][0] -= HH[1][0];
	Covar[1][1] -= HH[1][1];
}

/**
  * @brief  This function runs a predict on the EKF
  * @param  EKF state and covariance
  * @retval None, EKF updated in place
  */
void Predict_State(float State[2], float Covar[2][2], float Delta_time, float Process_Noise[2]) {
	Covar[0][0] += ((State[0]*Process_Noise[0])*(State[0]*Process_Noise[0]))*Delta_time;/* The first noise component is fractional shift/second */
	Covar[0][0] = Covar[0][0]>MODULUS_LIMIT?MODULUS_LIMIT:Covar[0][0];/* Apply limit on size of the covar */
	Covar[1][1] += Delta_time*Process_Noise[1];
	Covar[1][1] = Covar[1][1]>POS_LIMIT?POS_LIMIT:Covar[1][1];/* Apply limits */
}
#endif
