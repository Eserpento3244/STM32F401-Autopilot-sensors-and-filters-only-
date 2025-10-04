#include "PID.h"

inline float limit(float min, float max, float value) 
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

int PID_init(PidData *pd, float Kpv, float Kiv, float Kdv, float outMin, float outMax, float intMin, float intMax, float filtAlpha) 
{
    if(pd == 0) return 1;
    pd->Kp = Kpv;
    pd->Ki= Kiv;
    pd->Kd = Kdv;
    pd->prevError = 0;
    pd->integral = 0;
    pd->integralMin = intMin;
    pd->integralMax = intMax;
    pd->outputMin = outMin;
    pd->outputMax = outMax;
    pd->alpha = (filtAlpha >= 0.0f && filtAlpha <= 1.0f) ? filtAlpha : 0.1f;
    pd->derivativeFiltered = 0;
    pd->prevOutput = 0;
    return 0;
}

float calculate(PidData *pd, float current_value, float setpoint, float dt)
{
    if (dt < 0.001) return pd->prevOutput;

    float error = setpoint - current_value;

    float Pout = pd->Kp * error;

    pd->integral += error * dt;
    pd->integral = limit(pd->integralMin, pd->integralMax, pd->integral);
    float Iout = pd->Ki * pd->integral;

    float derivative = (error - pd->prevError) / dt;
    pd->derivativeFiltered = pd->alpha * pd->derivativeFiltered + (1 - pd->alpha) * derivative;
    float Dout = pd->Kd * pd->derivativeFiltered;

    pd->prevError = error;

    float output = Pout + Iout + Dout;
    output = limit(pd->outputMin, pd->outputMax, output);

    pd->prevOutput = output;
    return -output;
}