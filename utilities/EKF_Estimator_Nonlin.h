#pragma once

void Update_State(float State[3], float Covar[3][3], float Measurement, float Measurement_Covar);
void Predict_State(float State[3], float Covar[3][3], float Delta_Time, float Process_Noise[3], float Delta_Pos);
