#include "imath.h"

float air_density(float temperature_C, float humidity_RH, float pressure_Pa) 
{

    float T = temperature_C + 273.15f;

    float p_sat = 610.94f * expf((17.625f * temperature_C) / (temperature_C + 243.04f));

    float p_v = humidity_RH * 0.01f * p_sat;

    float p_d = pressure_Pa - p_v;

    return (p_d / (Rd * T)) + (p_v / (Rv * T));
}

float air_speed(float total_pressure, float static_pressure, float air_density) 
{
    float q = total_pressure - static_pressure; 
    if (q < 0) q = 0; 
    return sqrtf((2.0f * q) / air_density);
}

float altitude_from_pressure(float pressure_Pa, float sea_level_pressure_Pa) 
{
    return 44330.0f * (1.0f - powf(pressure_Pa / sea_level_pressure_Pa, 0.1903f));
}

float update_forward_speed(MPU9250_t *data, float dt)
{
    float pitch = DEG2RAD((int)data->pitch);
    float roll  = DEG2RAD((int)data->roll);

    float gx = G * sinf(pitch);
    float gy = -G * sinf(roll) * cosf(pitch);
    float gz = G * cosf(roll) * cosf(pitch);

    float lin_ax = data->Ax - gx;
    float lin_ay = data->Ay - gy;
    float lin_az = data->Az - gz;

    float forward_acc = lin_ay * cosf(roll) + lin_az * sinf(roll);
    
    return forward_acc * dt;
}


void speedfilter(float *s, float ns, float fns)
{
  *s = AlphaSpFilt * (*s + fns) + BetaSpFilt * ns;
}
