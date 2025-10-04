#ifndef DHT11_H
#define DHT11_H


#include <stdio.h>
#include <stm32f4xx_hal.h>

#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)

#define DHT11_PORT GPIOB
#define DHT11_PIN  GPIO_PIN_7

typedef struct
{
  uint8_t data[5];
uint8_t ok;
} DHT_t;

void DHT11_Set_Pin_Output();
void DHT11_Set_Pin_Input();
uint8_t DHT11_Read(DHT_t *dht);
void DWT_Init(void);
void delay_us(uint32_t us);

#endif