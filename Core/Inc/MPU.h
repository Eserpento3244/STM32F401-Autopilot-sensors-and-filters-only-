#ifndef MPU_INC_H
#define MPU_INC_H

#include <stdio.h>
#include <stm32f4xx_hal.h>
#include <math.h>


#define TWO_KP (2.0f * 0.5f) 
#define TWO_KI (2.0f * 0.0f) 


#define MPU9250_ADDR      0xD0 
#define AK8963_ADDR       0x18 

#define alpha             0.95f

#define WHO_AM_I_REG      0x75
#define PWR_MGMT_1_REG    0x6B
#define SMPLRT_DIV_REG    0x19
#define ACCEL_CONFIG_REG  0x1C
#define GYRO_CONFIG_REG   0x1B
#define ACCEL_XOUT_H_REG  0x3B
#define GYRO_XOUT_H_REG   0x43
#define INT_PIN_CFG_REG   0x37 

#define MAG_RESOLUTION_UT 0.15f
#define AK8963_WHO_AM_I   0x00 
#define AK8963_HXL        0x03 
#define AK8963_CNTL1      0x0A 
#define AK8963_ASAX       0x10 

#define PI                3.1415f

typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    float Ax, Ay, Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    float Gx, Gy, Gz;
    
    int16_t Mag_X_RAW;
    int16_t Mag_Y_RAW;
    int16_t Mag_Z_RAW;
    float Mx, My, Mz;
    float mag_sens_adj[3]; 
    
    float roll,pitch,yaw;
} MPU9250_t;

float radiansToDegrees(float);
uint8_t MPU9250_Init(I2C_HandleTypeDef *hi2c, MPU9250_t *MPU9250);
void MPU9250_Read_Accel(I2C_HandleTypeDef *hi2c, MPU9250_t *data);
void MPU9250_Read_Gyro(I2C_HandleTypeDef *hi2c, MPU9250_t *data);
void MPU9250_Read_Mag(I2C_HandleTypeDef *hi2c, MPU9250_t *data);
void MPU9250_Filter_Data(MPU9250_t *data, float dt);

#endif /* MPU_INC_H */