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
#include "bh1750.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "timers.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EG_BIT_MUTE_LED   (1U << 0)
#define EG_BIT_MUTE_BUZ   (1U << 1)
#define EG_BIT_SERVO      (1U << 2)
#define EG_BIT_DARK       (1U << 3)

#define LUX_DARK_THRESHOLD   15.0f
#define LUX_HYSTERESIS       2.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* Definitions for BuzzerTask */
osThreadId_t BuzzerTaskHandle;
const osThreadAttr_t BuzzerTask_attributes = {
  .name = "BuzzerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ServoTask */
osThreadId_t ServoTaskHandle;
const osThreadAttr_t ServoTask_attributes = {
  .name = "ServoTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for LedTask */
osThreadId_t LedTaskHandle;
const osThreadAttr_t LedTask_attributes = {
  .name = "LedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LuxTask */
osThreadId_t LuxTaskHandle;
const osThreadAttr_t LuxTask_attributes = {
  .name = "LuxTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */
static volatile uint32_t g_icStart = 0;
static volatile uint32_t g_echoUs = 0;
static volatile uint8_t g_capState =0;

volatile float g_distance_cm = 999.0f;

// USER CODE BEGIN PV
static volatile TickType_t g_servoDeadlineTicks = 0;
// USER CODE END PV

static EventGroupHandle_t gEg = NULL;
static TimerHandle_t MuteLedTimer = NULL;
static TimerHandle_t MuteBuzTimer = NULL;

volatile float g_lux = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C3_Init(void);
void StartTask01(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);

/* USER CODE BEGIN PFP */
static inline void DWT_Delay_Init(void);
static inline void delay_us(uint32_t us);
static inline void trig_pulse_10us(void);
static inline void LED_ALL_TOGGLE(void);
static inline void LED_ALL_OFF(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void DWT_Delay_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = (SystemCoreClock / 1000000) * us;
  while ((DWT->CYCCNT - start) < ticks) { __NOP(); }
}

static inline void trig_pulse_10us(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
  delay_us(2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  delay_us(12);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

static inline void LED_ALL_OFF(void) {
	HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);
}

static inline void LED_ALL_TOGGLE(void) {
	HAL_GPIO_TogglePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin);
}

static void vMuteLedTimerCb(TimerHandle_t xTimer) {
    xEventGroupClearBits(gEg, EG_BIT_MUTE_LED);
}

static void vMuteBuzTimerCb(TimerHandle_t xTimer) {
    xEventGroupClearBits(gEg, EG_BIT_MUTE_BUZ);
}
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  DWT_Delay_Init();
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
  gEg = xEventGroupCreate();
  configASSERT(gEg != NULL);

  MuteLedTimer = xTimerCreate("MuteLED5s", pdMS_TO_TICKS(5000), pdFALSE, NULL, vMuteLedTimerCb);
  configASSERT(MuteLedTimer != NULL);

  MuteBuzTimer = xTimerCreate("MuteBUZ5s", pdMS_TO_TICKS(5000), pdFALSE, NULL, vMuteBuzTimerCb);
  configASSERT(MuteBuzTimer != NULL);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of BuzzerTask */
  BuzzerTaskHandle = osThreadNew(StartTask01, NULL, &BuzzerTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartTask02, NULL, &SensorTask_attributes);

  /* creation of ServoTask */
  ServoTaskHandle = osThreadNew(StartTask03, NULL, &ServoTask_attributes);

  /* creation of LedTask */
  LedTaskHandle = osThreadNew(StartTask04, NULL, &LedTask_attributes);

  /* creation of LuxTask */
  LuxTaskHandle = osThreadNew(StartTask05, NULL, &LuxTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // PA2 -> TIM2_CH3 (ECHO)
  GPIO_InitTypeDef GPIO_InitStruct2 = {0};
  GPIO_InitStruct2.Pin       = GPIO_PIN_2;
  GPIO_InitStruct2.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct2.Pull      = GPIO_PULLDOWN;
  GPIO_InitStruct2.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct2.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct2);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    if (g_capState == 0) {
        g_icStart = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
        g_capState = 1;
    } else {
        uint32_t icEnd = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
        uint32_t dur = icEnd-g_icStart;
        g_echoUs = dur;
        __HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
        g_capState = 0;

        BaseType_t hp = pdFALSE;
        vTaskNotifyGiveFromISR(SensorTaskHandle, &hp);
        portYIELD_FROM_ISR(hp);
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN) {
    if (GPIO_PIN == GPIO_PIN_0) {
        static uint32_t last = 0;
        uint32_t nowMs = HAL_GetTick();
        if ((nowMs - last) < 150) return;
        last = nowMs;

        EventBits_t bitsNow = xEventGroupGetBitsFromISR(gEg);
        if (bitsNow & EG_BIT_DARK) {
            return;
        }

        BaseType_t hp = pdFALSE;

        TickType_t nowTicks = xTaskGetTickCountFromISR();
        TickType_t newDeadline = nowTicks + pdMS_TO_TICKS(5000);
        if ((int32_t)(newDeadline - g_servoDeadlineTicks) > 0) {
            g_servoDeadlineTicks = newDeadline;
        }

        xEventGroupSetBitsFromISR(gEg, EG_BIT_MUTE_LED, &hp);
        xTimerResetFromISR(MuteLedTimer, &hp);

        xEventGroupSetBitsFromISR(gEg, EG_BIT_MUTE_BUZ, &hp);
        xTimerResetFromISR(MuteBuzTimer, &hp);

        xEventGroupSetBitsFromISR(gEg, EG_BIT_SERVO, &hp);

        portYIELD_FROM_ISR(hp);
    }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the BuzzerTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask01 */
void StartTask01(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t buzzer = 0;
  /* Infinite loop */
  for(;;)
  {
	    EventBits_t bits = xEventGroupGetBits(gEg);
	    if (bits & EG_BIT_MUTE_BUZ) {
	        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	        vTaskDelay(pdMS_TO_TICKS(50));
	        continue;
	    }
	  float d = g_distance_cm;
	  if(!buzzer&&d<5.0f) {
		  buzzer=1;
	  } else if(buzzer&&d>6.0f) {
		  buzzer=0;
	  }
	  if(buzzer) {
	      HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
	      vTaskDelay(pdMS_TO_TICKS(300));
	  } else {
	      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET);
	      vTaskDelay(pdMS_TO_TICKS(100));
	  }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
	  const TickType_t period = pdMS_TO_TICKS(60);
	  TickType_t last = xTaskGetTickCount();
	  for(;;)
	  {
	    trig_pulse_10us();
	    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(40)) == 1) {
	      uint32_t us = g_echoUs;
	      float d = (float)us / 58.0f;

	      if (d >= 2.0f && d <= 400.0f) {
	        g_distance_cm = d;
	      }
	    }
	    vTaskDelayUntil(&last, period);
	  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the ServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
	  for(;;)
	  {
	    xEventGroupWaitBits(gEg, EG_BIT_SERVO, pdTRUE, pdFALSE, portMAX_DELAY);

	    if (xEventGroupGetBits(gEg) & EG_BIT_DARK) {
	        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	        continue;
	    }

	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);

	    while ((int32_t)(g_servoDeadlineTicks - xTaskGetTickCount()) > 0) {
	        if (xEventGroupGetBits(gEg) & EG_BIT_DARK) {
	            break;
	        }
	        vTaskDelay(pdMS_TO_TICKS(10));
	    }

	    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the LedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
	uint8_t nearState = 0;
	LED_ALL_OFF();
  for(;;)
  {
	    if (xEventGroupGetBits(gEg) & EG_BIT_MUTE_LED) {
	        LED_ALL_OFF();
	        vTaskDelay(pdMS_TO_TICKS(50));
	        continue;
	    }

	  float d = g_distance_cm;
	  if(!nearState&&d<5.0f) {
		  nearState=1;
	  } else if(nearState&&d>6.0f) {
		  nearState=0;
	  }
	  if(nearState) {
	      LED_ALL_TOGGLE();
	      vTaskDelay(pdMS_TO_TICKS(300));
	  } else {
	      LED_ALL_OFF();
	      vTaskDelay(pdMS_TO_TICKS(100));
	  }
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the LuxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
    BH1750_Handle_t bh = {
        .i2c  = &hi2c3,
        .addr = (0x23u << 1)
    };

    // Init BH1750 cho tới khi thành công
    while (BH1750_Init(&bh) != HAL_OK) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    uint8_t isDark = 0;
    const TickType_t period = pdMS_TO_TICKS(500);

    for (;;) {
        float lux = 0.0f;
        if (BH1750_ReadLux(&bh, &lux, 50) == HAL_OK) {
            g_lux = lux;

            if (!isDark && lux < LUX_DARK_THRESHOLD) {
                isDark = 1;
                xEventGroupSetBits(gEg, EG_BIT_DARK);

                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
                g_servoDeadlineTicks = xTaskGetTickCount();

                xEventGroupSetBits(gEg, EG_BIT_MUTE_LED | EG_BIT_MUTE_BUZ);
                xTimerStop(MuteLedTimer, 0);
                xTimerStop(MuteBuzTimer, 0);

            } else if (isDark && lux > (LUX_DARK_THRESHOLD + LUX_HYSTERESIS)) {
                isDark = 0;
                xEventGroupClearBits(gEg, EG_BIT_DARK);
                xEventGroupClearBits(gEg, EG_BIT_MUTE_LED | EG_BIT_MUTE_BUZ);
            }
        } else {
            (void)BH1750_Init(&bh);
        }

        vTaskDelay(period);
    }
  /* USER CODE END StartTask05 */
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
