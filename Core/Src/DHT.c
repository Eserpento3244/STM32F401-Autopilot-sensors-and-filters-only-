#include "DHT.h"

void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (HAL_RCC_GetHCLKFreq() / 1000000) * us;
    while ((DWT->CYCCNT - start) < ticks);
}


void DHT11_Set_Pin_Output(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

void DHT11_Set_Pin_Input(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

uint8_t DHT11_Read(DHT_t *dht) {
    uint8_t i, j;
    uint16_t tm = 0;
    // ????????? ??????
    DHT11_Set_Pin_Output();
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    delay_us(18000); // ??????? 18 ??
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    delay_us(40);

    DHT11_Set_Pin_Input();

    if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) return 1;
    tm = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET)
    {
        if(tm > 1000) return 3;
        tm++;
    }
    tm = 0;
    while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET)
    {
        if(tm > 1000) return 3;
        tm++;
    }

    for (j = 0; j < 5; j++) {
        uint8_t byte = 0;
        for (i = 0; i < 8; i++) {
            tm = 0;
            while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET)
            {
                if(tm > 1000) return 3;
                tm++;
            }
            
            delay_us(35); 

            if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET)
                byte |= (1 << (7 - i));
            tm = 0;
            while (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET)
                          {
                if(tm > 1000) return 3;
                tm++;
            }
        }
        dht->data[j] = byte;
    }

    if (dht->data[4] == (uint8_t)(dht->data[0] + dht->data[1] + dht->data[2] + dht->data[3]))
    {
      dht->ok = 1;
        return 0;
    }
    else
    {
      dht->ok = 0;
        return 2; // ?????? CRC
    }
}

