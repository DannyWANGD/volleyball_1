/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END Variables */
osThreadId InitTaskHandle;
osThreadId PlatformTaskHandle;
osThreadId CommunicateTaskHandle;
osThreadId RemoteTaskHandle;
osThreadId ChassisTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Init_Task(void const * argument);
void Platform_Task(void const * argument);
void Communicate_Task(void const * argument);
void Remote_Task(void const * argument);
void Chassis_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of InitTask */
  osThreadDef(InitTask, Init_Task, osPriorityRealtime, 0, 128);
  InitTaskHandle = osThreadCreate(osThread(InitTask), NULL);

  /* definition and creation of PlatformTask */
  osThreadDef(PlatformTask, Platform_Task, osPriorityRealtime, 0, 512);
  PlatformTaskHandle = osThreadCreate(osThread(PlatformTask), NULL);

  /* definition and creation of CommunicateTask */
  osThreadDef(CommunicateTask, Communicate_Task, osPriorityRealtime, 0, 512);
  CommunicateTaskHandle = osThreadCreate(osThread(CommunicateTask), NULL);

  /* definition and creation of RemoteTask */
  osThreadDef(RemoteTask, Remote_Task, osPriorityRealtime, 0, 256);
  RemoteTaskHandle = osThreadCreate(osThread(RemoteTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, Chassis_Task, osPriorityRealtime, 0, 256);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Init_Task */
/**
  * @brief  Function implementing the InitTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Init_Task */
__weak void Init_Task(void const * argument)
{
  /* USER CODE BEGIN Init_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Init_Task */
}

/* USER CODE BEGIN Header_Platform_Task */
/**
* @brief Function implementing the PlatformTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Platform_Task */
__weak void Platform_Task(void const * argument)
{
  /* USER CODE BEGIN Platform_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Platform_Task */
}

/* USER CODE BEGIN Header_Communicate_Task */
/**
* @brief Function implementing the CommunicateTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Communicate_Task */
__weak void Communicate_Task(void const * argument)
{
  /* USER CODE BEGIN Communicate_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Communicate_Task */
}

/* USER CODE BEGIN Header_Remote_Task */
/**
* @brief Function implementing the RemoteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remote_Task */
__weak void Remote_Task(void const * argument)
{
  /* USER CODE BEGIN Remote_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Remote_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
__weak void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
