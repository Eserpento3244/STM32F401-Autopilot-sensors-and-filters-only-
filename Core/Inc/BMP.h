#ifndef BMP280_H
#define BMP280_H

#include <stdio.h>
#include <stm32f4xx_hal.h>

#define BMP280_I2C_ADDR 0x76 << 1  

#define BMP280_REG_ID        0xD0
#define BMP280_REG_RESET     0xE0
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG    0xF5
#define BMP280_REG_PRESS_MSB 0xF7
#define BMP280_REG_TEMP_MSB  0xFA

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    int32_t t_fine;
} BMP280_t;

typedef struct {
  float temp;
  float pr;
} BMPdata_t;

uint8_t BMP280_Init(BMP280_t *bmp, I2C_HandleTypeDef *hi2c);
float BMP280_ReadTemperature(BMP280_t *bmp);
float BMP280_ReadPressure(BMP280_t *bmp);

#endif
