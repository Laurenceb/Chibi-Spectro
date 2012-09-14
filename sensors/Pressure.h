#pragma once

#include "ch.h"
#include <stdbool.h>

bool Calibrate_Sensor(uint16_t diff);
float Convert_Pressure(uint16_t diff);
