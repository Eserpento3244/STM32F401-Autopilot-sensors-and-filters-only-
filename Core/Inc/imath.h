#ifndef IMATH_H
#define IMATH_H

#include <math.h>
#include "MPU.h"

#define Rd 287.05f
#define Rv 461.495f
#define Psea_level 101325.0f
#define AlphaSpFilt 0.8
#define BetaSpFilt 0.2
#define G 9.80665f 
#define DEG2RAD(x) ((x) * PI / 180.0f)

float air_density(float temperature_C, float humidity_RH, float pressure_Pa); 
float air_speed(float total_pressure, float static_pressure, float air_density); 
float altitude_from_pressure(float pressure_Pa, float sea_level_pressure_Pa);
float update_forward_speed(MPU9250_t *data, float dt);
void speedfilter(float *s, float ns, float fns);

#define DEG2PWM(x) ((x) * 11.11f)

#endif