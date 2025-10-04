/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
#include "MPU.h"
#include "BMP.h"
#include "DHT.h"
#include "imath.h"
#include "PID.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

osThreadId Task_PitoHandle;
uint32_t Task_PitoBuffer[ 128 ];
osStaticThreadDef_t Task_PitoControlBlock;
osThreadId Task_IMUHandle;
uint32_t TaskmpuBuffer[ 128 ];
osStaticThreadDef_t TaskmpuControlBlock;
osMutexId Mutex_speedHandle;
osStaticMutexDef_t myMutex01ControlBlock;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void StartPitoTask(void const * argument);
void StartImufTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
MPU9250_t MPU9250;
BMP280_t BMP2801;
DHT_t DHT11;
BMPdata_t bmdata;
PidData Pidroll;
PidData Pidpitch;
PidData Pidyaw;

float speed = 0;
uint8_t errDHT11 = 0;
int s = 0, s1 = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	DWT_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	while (!MPU9250_Init(&hi2c1, &MPU9250))
	{
		HAL_Delay(100);
	}
	while (!BMP280_Init(&BMP2801, &hi2c1))
	{
		HAL_Delay(100);
	}
	while (DHT11_Read(&DHT11) == 1)
	{
		HAL_Delay(100);
	}
    PID_init(&Pidroll,1,0.1,0.1,-45, 45,-10,10,0.96);
    PID_init(&Pidpitch,1,0.1,0.1,-45, 45,-10,10,0.96);
    PID_init(&Pidyaw,1,0.1,0.1,-45, 45,-10,10,0.96);

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of Mutex_speed */
  osMutexStaticDef(Mutex_speed, &myMutex01ControlBlock);
  Mutex_speedHandle = osMutexCreate(osMutex(Mutex_speed));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task_Pito */
  osThreadStaticDef(Task_Pito, StartPitoTask, osPriorityNormal, 0, 128, Task_PitoBuffer, &Task_PitoControlBlock);
  Task_PitoHandle = osThreadCreate(osThread(Task_Pito), NULL);

  /* definition and creation of Task_IMU */
  osThreadStaticDef(Task_IMU, StartImufTask, osPriorityNormal, 0, 128, TaskmpuBuffer, &TaskmpuControlBlock);
  Task_IMUHandle = osThreadCreate(osThread(Task_IMU), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{

    }

        
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartPitoTask */
/**
  * @brief  Function implementing the Task_Pito thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartPitoTask */
void StartPitoTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  char c = 0;
  for(;;)
  {
    if(c >= 10)
    {
      taskENTER_CRITICAL(); 
            if (DHT11_Read(&DHT11) == 0)
			{
              errDHT11 = 0;
			}
            else errDHT11++;
            taskEXIT_CRITICAL(); 
            c = 0;
            
    }
    bmdata.pr = BMP280_ReadPressure(&BMP2801);
    c++;
    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartImufTask */
/**
* @brief Function implementing the Task_IMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartImufTask */
void StartImufTask(void const * argument)
{
  /* USER CODE BEGIN StartImufTask */
  /* Infinite loop */
  uint32_t lasttime = 0;
  float dt = 0.001;
  for(;;)
  {
    	uint32_t now = HAL_GetTick();
		dt = (float)((float)(now - lasttime) / 1000.0f);
		lasttime = now;
        
    	MPU9250_Read_Accel(&hi2c1, &MPU9250);
		MPU9250_Read_Gyro(&hi2c1, &MPU9250);
		MPU9250_Read_Mag(&hi2c1, &MPU9250);
		MPU9250_Filter_Data(&MPU9250, dt);
        
		float imuspeed = update_forward_speed(&MPU9250, dt);
        
        if (xSemaphoreTake(Mutex_speedHandle, portMAX_DELAY) == pdTRUE) 
        {
            float ar = air_density(DHT11.data[2], DHT11.data[0], bmdata.pr);
		float sp = air_speed(bmdata.pr, 98800, ar);
        speedfilter(&speed,sp,imuspeed);
            xSemaphoreGive(Mutex_speedHandle);
        }
        int16_t duty1 = (1500 + ((int16_t) DEG2PWM(calculate(&Pidroll, MPU9250.roll, 0, dt)) )) & 2047;
        int16_t duty2 = (1500 + ((int16_t) DEG2PWM(calculate(&Pidpitch, MPU9250.pitch, 0, dt)) )) & 2047;
        int16_t duty3 = (1500 + ((int16_t) DEG2PWM(calculate(&Pidyaw, MPU9250.yaw, 0, dt)) )) & 2047;
        //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty1);
        //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty2);
        //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty2);
    osDelay(10);
  }
  /* USER CODE END StartImufTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	  /* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	  /* User can add his own implementation to report the file name and line number,
		 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
