#ifndef PID_INC_H
#define PID_INC_H

typedef struct
{
    float prevError;
    float integral;
    float integralMin, integralMax;
    float outputMin, outputMax;
    float derivativeFiltered;
    float alpha;
    float prevOutput;
    float Kp, Ki, Kd;
} PidData;

int PID_init(PidData *pd,float Kpv, float Kiv, float Kdv, float outMin, float outMax, float intMin, float intMax, float filtAlpha);

inline float limit(float min, float max, float value);

float calculate(PidData *pd, float current_value, float setpoint, float dt);

#endif /* PID_INC_H */