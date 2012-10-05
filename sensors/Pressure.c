#include "Pressure.h"
#include "Hardware_Conf.h"			/* Location of the sensor sensitivity data */

static float Pressure_Offset;			/* Stores the sensor offset */

/**
  * @brief  This function calibrates the adc offset on the pressure sensor
  * @param  uint16_t ADc value
  * @retval bool to indicate when enough sensor readings have been obtained
  */
bool Calibrate_Sensor(uint16_t diff) {
	static uint8_t n=0;
	if(++n) {				/* Take 256 samples from the pressure sensor */
		Pressure_Offset+=diff;
		return TRUE;
	}
	else {
		Pressure_Offset=(float)Pressure_Offset/(float)255.0;
		return FALSE;
	}
}


/**
  * @brief  This function returns the converted pressure from an unsigned integer
  * @param  uint16_t from ADC
  * @retval Pressure in PSI
  */
float Convert_Pressure(uint16_t diff) {
	return 	(DIFF_GAIN)*((float)diff-Pressure_Offset);
}

