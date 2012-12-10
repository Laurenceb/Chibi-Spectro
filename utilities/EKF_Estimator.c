#include <math.h>

#include "EKF_Estimator.h"

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
	State[0] += p_*(Covar[0][0]*p_ + Covar[0][1]*s_)*y;	//x+=PH'y/S
	State[1] += s_*(Covar[1][0]*p_ + Covar[1][1]*s_)*y;
	y = (p_*p_ + s_*s_) / s;
	Covar[0][0] -= (Covar[0][0]*Covar[0][0] + Covar[0][1]*Covar[1][0]) * y;//P-=P(H'/S)HP
	Covar[0][1] -= (Covar[0][0]*Covar[0][1] + Covar[0][1]*Covar[1][1]) * y;
	Covar[1][0] -= (Covar[1][0]*Covar[0][0] + Covar[1][1]*Covar[1][0]) * y;
	Covar[1][1] -= (Covar[1][0]*Covar[0][1] + Covar[1][1]*Covar[1][1]) * y;
}

/**
  * @brief  This function runs a predict on the EKF
  * @param  EKF state and covariance
  * @retval None, EKF updated in place
  */
void Predict_State(float State[2], float Covar[2][2], float Delta_time, float Process_Noise[2]) {
	Covar[0][0] += Delta_time*Process_Noise[0];
	Covar[1][1] += Delta_time*Process_Noise[1];
}
