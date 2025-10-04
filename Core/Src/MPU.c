#include "MPU.h"

uint8_t MPU9250_Init(I2C_HandleTypeDef *hi2c, MPU9250_t *MPU9250) 
{
    uint8_t check;
    uint8_t data;

    HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 0x71) 
    { 
        data = 0; 
        HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
        data = 0x07;
        HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
        data = 0x00; 
        HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);
        data = 0x00; 
        HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);

        
        data = 0x02;
        HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR, INT_PIN_CFG_REG, 1, &data, 1, 1000);
        HAL_Delay(10);

       
        HAL_I2C_Mem_Read(hi2c, AK8963_ADDR, AK8963_WHO_AM_I, 1, &check, 1, 1000);
        if (check == 0x48) 
        { 
            uint8_t calib_data[3];
            
          
            data = 0x00;
            HAL_I2C_Mem_Write(hi2c, AK8963_ADDR, AK8963_CNTL1, 1, &data, 1, 1000);
            HAL_Delay(10);
            data = 0x0F; 
            HAL_I2C_Mem_Write(hi2c, AK8963_ADDR, AK8963_CNTL1, 1, &data, 1, 1000);
            HAL_Delay(10);
            
            
            HAL_I2C_Mem_Read(hi2c, AK8963_ADDR, AK8963_ASAX, 1, calib_data, 3, 1000);

            
            for(int i=0; i<3; i++) 
            {
                MPU9250->mag_sens_adj[i] = (float)(calib_data[i] - 128)/256.0f + 1.0f;
            }

            
            data = 0x00;
            HAL_I2C_Mem_Write(hi2c, AK8963_ADDR, AK8963_CNTL1, 1, &data, 1, 1000);
            HAL_Delay(10);

            
            data = 0x16;
            HAL_I2C_Mem_Write(hi2c, AK8963_ADDR, AK8963_CNTL1, 1, &data, 1, 1000);
            HAL_Delay(10);
            
            return 1; 
        }
    }
    return 0; 
}

void MPU9250_Read_Accel(I2C_HandleTypeDef *hi2c, MPU9250_t *data) {
    uint8_t buffer[6];

    // ????????? 6 ???? ?????? ?????????????
    HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, ACCEL_XOUT_H_REG, 1, buffer, 6, 1000);

    data->Accel_X_RAW = (int16_t)(buffer[0] << 8 | buffer[1]);
    data->Accel_Y_RAW = (int16_t)(buffer[2] << 8 | buffer[3]);
    data->Accel_Z_RAW = (int16_t)(buffer[4] << 8 | buffer[5]);

    // ???????????? ? g (??? ?2g)
    data->Ax = data->Accel_X_RAW / 16384.0;
    data->Ay = data->Accel_Y_RAW / 16384.0;
    data->Az = (data->Accel_Z_RAW - 7000) / 16384.0;
}

void MPU9250_Read_Gyro(I2C_HandleTypeDef *hi2c, MPU9250_t *data) {
    uint8_t buffer[6];

    // ????????? 6 ???? ?????? ?????????
    HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR, 0x43, 1, buffer, 6, 1000);

    data->Gyro_X_RAW = (int16_t)(buffer[0] << 8 | buffer[1]);
    data->Gyro_Y_RAW = (int16_t)(buffer[2] << 8 | buffer[3]);
    data->Gyro_Z_RAW = (int16_t)(buffer[4] << 8 | buffer[5]);

    // ???????????? ? ?/? (??? ?250?/s)
    data->Gx = (data->Gyro_X_RAW / 131.0);
    data->Gy = (data->Gyro_Y_RAW / 131.0);
    data->Gz = (data->Gyro_Z_RAW / 131.0);
}

float radiansToDegrees(float d)
{
  return d * 180.0f / PI;
}

void MPU9250_Read_Mag(I2C_HandleTypeDef *hi2c, MPU9250_t *data) 
{
    uint8_t buffer[7]; // 6 ???? ?????? + 1 ???? ???????

    // ?????? ??????, ?????? ???? ??? ?????? (????????? DRDY ??? ? ???????? ST1)
    uint8_t status;
    HAL_I2C_Mem_Read(hi2c, AK8963_ADDR, 0x02, 1, &status, 1, 100);

    if (status & 0x01) 
    {
        // ????????? 7 ???? ??????, ??????? ? HXL
        // ?????? ???????? ST2 ? ????? ?????????? ???? ?????????? ??????
        HAL_I2C_Mem_Read(hi2c, AK8963_ADDR, AK8963_HXL, 1, buffer, 7, 1000);

        // ????????? ?? ???????????? (overflow bit ? ST2)
        if (!(buffer[6] & 0x08)) 
        {
            // ??????? ????: Little Endian (??????? ???????)
            data->Mag_X_RAW = (int16_t)(buffer[1] << 8 | buffer[0]);
            data->Mag_Y_RAW = (int16_t)(buffer[3] << 8 | buffer[2]);
            data->Mag_Z_RAW = (int16_t)(buffer[5] << 8 | buffer[4]);

            // ???????????? ? ?????????? (????), ???????? ??????????
            data->Mx = (float)data->Mag_X_RAW * data->mag_sens_adj[0] * MAG_RESOLUTION_UT;
            data->My = (float)data->Mag_Y_RAW * data->mag_sens_adj[1] * MAG_RESOLUTION_UT;
            data->Mz = (float)data->Mag_Z_RAW * data->mag_sens_adj[2] * MAG_RESOLUTION_UT;
        }
    }
}

void MPU9250_Filter_Data(MPU9250_t *data, float dt)
{
      float accRoll = atan2(data->Ay, data->Az);
      float accPitch = atan2(-data->Ax, sqrt(data->Ay * data->Ay + data->Az * data->Az));
   


float cosRoll = cos(accRoll);
float sinRoll = sin(accRoll);
float cosPitch = cos(accPitch);
float sinPitch = sin(accPitch);

// ??????????? ????? (Roll)
float Mx_prime = data->Mx;
float My_prime = data->My * cosRoll + data->Mz * sinRoll;
float Mz_prime = -data->My * sinRoll + data->Mz * cosRoll;

// ??????????? ??????? (Pitch)
float M_horizontal_x = Mx_prime * cosPitch - Mz_prime * sinPitch;
float M_horizontal_y = My_prime;




if((int)(data->Gz * dt * 1000.f) != 0)
{
   
  data->yaw = alpha * (data->yaw + data->Gz * dt) + (1.0 - alpha)  * radiansToDegrees(atan2(-M_horizontal_y, M_horizontal_x));
}
//if (data->yaw > 180) data->yaw -= 360;
//if (data->yaw < 0) data->yaw += 360;

      data->roll = alpha * (data->roll + (data->Gx * dt)) + (1.0 - alpha) * radiansToDegrees(accRoll);
      data->pitch = alpha * (data->pitch + (data->Gy * dt)) + (1.0 - alpha) * radiansToDegrees(accPitch);
      

}



