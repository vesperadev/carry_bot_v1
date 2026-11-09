/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "gpio.h"
#include <string.h>
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
/* USER CODE BEGIN Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* hello-tx diagnostic: do not start Link/ZDT/Servo */
/* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Diagnostic:
   *   ´®¿Ú2 (4-pin) TX @115200 -> "hello\r\n"   (PA9)
   *   ´®¿Ú1 (3-pin) TX @9600   -> "hello1\r\n"  (PG14)
   *   onboard LED_B PH10: IO high = on (same as BoardLed)
   */
  static const char msg2[] = "hello\r\n";
  static const char msg1[] = "hello1\r\n";
  static const char fail[] = "fail\r\n";
  GPIO_InitTypeDef led_cfg = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
  led_cfg.Pin = GPIO_PIN_10;
  led_cfg.Mode = GPIO_MODE_OUTPUT_PP;
  led_cfg.Pull = GPIO_NOPULL;
  led_cfg.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &led_cfg);

  for (;;)
  {
    HAL_StatusTypeDef st2 = HAL_UART_Transmit(&huart1, (uint8_t *)msg2, (uint16_t)(sizeof(msg2) - 1U), 100);
    HAL_StatusTypeDef st1 = HAL_UART_Transmit(&huart6, (uint8_t *)msg1, (uint16_t)(sizeof(msg1) - 1U), 100);

    if ((st2 != HAL_OK) || (st1 != HAL_OK))
    {
      (void)HAL_UART_Transmit(&huart6, (uint8_t *)fail, (uint16_t)(sizeof(fail) - 1U), 100);
      /* fast blink on TX failure */
      HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);
      osDelay(100);
      HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
      osDelay(100);
    }
    else
    {
      HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
      osDelay(500);
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
