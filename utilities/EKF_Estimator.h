#pragma once

void Update_State(float State[2], float Covar[2][2], float Measurement, float Position, float Measurement_Covar);
void Predict_State(float State[2], float Covar[2][2], float Delay_time, float Process_Noise[2]);
