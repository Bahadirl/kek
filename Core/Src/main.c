
/* USER CODE BEGIN Header */
/*
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <timers.h>
#include "semphr.h"

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
osThreadId defaultTaskHandle;
BaseType_t xTemperatureReturned;
BaseType_t xHumidityReturned;
BaseType_t xI2CReadDataReturned;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart6;
uint8_t SHT30_ADDR = 0x44 << 1;
uint8_t rawValues[6];
TaskHandle_t xTemperatureHandle = NULL;
TaskHandle_t xHumidityHandle = NULL;
TaskHandle_t xI2CHandle = NULL;
TimerHandle_t xTimers;
SemaphoreHandle_t xMutex = NULL;
uint16_t FETCH_SHT30 = 0xE000;
uint16_t START_SHT30 = 0x2130;
int8_t flag = 1;
float a, b;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

void vTaskUartTransmitTemperature(void * pvParameters);
void vTaskUartTransmitHumidity(void * pvParameters);
void vTaskSendI2C(void * pvParameters);
void vTimerCallback(TimerHandle_t xTimer);
void littleEndianToBigEndian(uint16_t value, uint8_t *buffer);
#ifdef _GNUC_
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart6, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();

    /* USER CODE BEGIN 2 */
    xMutex = xSemaphoreCreateMutex();
    if (xMutex == NULL)
    {
        // Handle mutex creation failure
    }

    xTimers = xTimerCreate("TIMER", pdMS_TO_TICKS(2000), pdTRUE, (void*)0, vTimerCallback);
    if (xTimers == NULL)
    {
        // Handle timer creation failure
    }

    if (xTimerStart(xTimers, 0) != pdPASS)
    {
        // Handle timer start failure
    }


xTemperatureReturned = xTaskCreate(vTaskUartTransmitTemperature, "TEMPERATURE", 500, (void*)1, 1, &xTemperatureHandle);
    xHumidityReturned = xTaskCreate(vTaskUartTransmitHumidity, "HUMIDITY", 500, (void*)1, 1, &xHumidityHandle);
    xI2CReadDataReturned = xTaskCreate(vTaskSendI2C, "I2C", 500, (void*)1, 1, &xI2CHandle);

    vTaskStartScheduler();
    /* USER CODE END 2 */

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
    /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

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
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */



    }
    /* USER CODE END 3 */
}

/*
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* *Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /* Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00303D5B;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

void littleEndianToBigEndian(uint16_t value, uint8_t *buffer)
{
    buffer[0] = (value & 0xff00u) >> 8u;
    buffer[1] = (value) & 0xffu;
}

void vTimerCallback(TimerHandle_t xTimer)
{
    flag = 1;
}

void vTaskSendI2C(void *pvParameters)
{
    for (;;)
    {
        const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
        if (flag == 1)
        {
            uint8_t cmd[2];
            littleEndianToBigEndian(START_SHT30, cmd);
            HAL_I2C_Master_Transmit_IT(&hi2c1, SHT30_ADDR, cmd, 2);
            HAL_Delay(2);
            littleEndianToBigEndian(FETCH_SHT30, cmd);
            HAL_I2C_Master_Transmit_IT(&hi2c1, SHT30_ADDR, cmd, 2);
            HAL_Delay(2);
            HAL_I2C_Master_Receive_IT(&hi2c1, SHT30_ADDR, rawValues, 6);
            vTaskDelay(xDelay);
            flag = 0;
        }
    }
}

void vTaskUartTransmitTemperature(void *pvParameters)
{
    for (;;)
    {
        const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
        if (xMutex != NULL)
        {
            if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
            {
                uint16_t rawTemp = (uint16_t)((((uint16_t)rawValues[0]) << 8) | (uint16_t)rawValues[1]);
                float temp = (float)((float)-45 + (float)175 * (float)rawTemp / (float)65535);
                printf("Temperature of room:%.2f\r\n", temp);
                xSemaphoreGive(xMutex);
                vTaskDelay(xDelay);
                a = temp;
                printf("Humidity of room:%.2f\r\n", a);
            }
        }
    }
}

void vTaskUartTransmitHumidity(void *pvParameters)
{
    for (;;)
    {
        const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
        if (xMutex != NULL)
        {
            if (xSemaphoreTake(xMutex, (TickType_t)10) == pdTRUE)
            {
                uint16_t rawHum = (uint16_t)((((uint16_t)rawValues[3]) << 8) | (uint16_t)rawValues[4]);
                float hum = (float)((float)100 * (float)rawHum / (float)65535);
                printf("Humidity of room:%.2f\r\n", hum);
                xSemaphoreGive(xMutex);
                vTaskDelay(xDelay);
                b = hum;
                printf("Temperature of room:%.2f\r\n", b);
            }
        }
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/*
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
    /* USER CODE END 5 */
}

/* MPU Configuration */
void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Disables the MPU */
    HAL_MPU_Disable();

    /* Initializes and configures the Region and the memory to be protected
    */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/*
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM9)
    {
        HAL_IncTick();
    }
}

/*
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
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
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
}
#endif /* USE_FULL_ASSERT */
