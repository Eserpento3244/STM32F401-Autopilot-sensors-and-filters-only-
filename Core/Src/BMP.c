#include "BMP.h"
#include "math.h"

static uint8_t BMP280_Read8(BMP280_t *bmp, uint8_t reg) 
{
    uint8_t value;
    HAL_I2C_Mem_Read(bmp->hi2c, BMP280_I2C_ADDR, reg, 1, &value, 1, HAL_MAX_DELAY);
    return value;
}

static uint16_t BMP280_Read16(BMP280_t *bmp, uint8_t reg) 
{
    uint8_t data[2];
    HAL_I2C_Mem_Read(bmp->hi2c, BMP280_I2C_ADDR, reg, 1, data, 2, HAL_MAX_DELAY);
    return (data[1] << 8) | data[0];
}

static int16_t BMP280_ReadS16(BMP280_t *bmp, uint8_t reg) 
{
    return (int16_t)BMP280_Read16(bmp, reg);
}

static uint32_t BMP280_Read24(BMP280_t *bmp, uint8_t reg) 
{
    uint8_t data[3];
    HAL_I2C_Mem_Read(bmp->hi2c, BMP280_I2C_ADDR, reg, 1, data, 3, HAL_MAX_DELAY);
    return ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
}

uint8_t BMP280_Init(BMP280_t *bmp, I2C_HandleTypeDef *hi2c) 
{
    bmp->hi2c = hi2c;
    if (BMP280_Read8(bmp, BMP280_REG_ID) != 0x58) return 0;

    bmp->dig_T1 = BMP280_Read16(bmp, 0x88);
    bmp->dig_T2 = BMP280_ReadS16(bmp, 0x8A);
    bmp->dig_T3 = BMP280_ReadS16(bmp, 0x8C);
    bmp->dig_P1 = BMP280_Read16(bmp, 0x8E);
    bmp->dig_P2 = BMP280_ReadS16(bmp, 0x90);
    bmp->dig_P3 = BMP280_ReadS16(bmp, 0x92);
    bmp->dig_P4 = BMP280_ReadS16(bmp, 0x94);
    bmp->dig_P5 = BMP280_ReadS16(bmp, 0x96);
    bmp->dig_P6 = BMP280_ReadS16(bmp, 0x98);
    bmp->dig_P7 = BMP280_ReadS16(bmp, 0x9A);
    bmp->dig_P8 = BMP280_ReadS16(bmp, 0x9C);
    bmp->dig_P9 = BMP280_ReadS16(bmp, 0x9E);

    uint8_t config[2] = {BMP280_REG_CTRL_MEAS, 0x27}; 
    HAL_I2C_Master_Transmit(bmp->hi2c, BMP280_I2C_ADDR, config, 2, HAL_MAX_DELAY);

    config[0] = BMP280_REG_CONFIG;
    config[1] = 0xA0; 
    HAL_I2C_Master_Transmit(bmp->hi2c, BMP280_I2C_ADDR, config, 2, HAL_MAX_DELAY);

    return 1;
}

float BMP280_ReadTemperature(BMP280_t *bmp) 
{
    int32_t adc_T = BMP280_Read24(bmp, BMP280_REG_TEMP_MSB) >> 4;
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bmp->dig_T1 << 1))) * ((int32_t)bmp->dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bmp->dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp->dig_T1))) >> 12) *
                     ((int32_t)bmp->dig_T3)) >> 14;
    bmp->t_fine = var1 + var2;
    float T = (bmp->t_fine * 5 + 128) >> 8;
    return T / 100.0f;
}

float BMP280_ReadPressure(BMP280_t *bmp) 
{
    int32_t adc_P = BMP280_Read24(bmp, BMP280_REG_PRESS_MSB) >> 4;
    int64_t var1 = ((int64_t)bmp->t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)bmp->dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp->dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp->dig_P3) >> 8) +
           ((var1 * (int64_t)bmp->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp->dig_P1) >> 33;

    if (var1 == 0) return 0; 

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp->dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp->dig_P7) << 4);
    return (float)p / 256.0f; 
}