/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
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
#include "includes.h"
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
osThreadId GimbalTaskHandle;
osThreadId INSTaskHandle;
osThreadId DetectTaskHandle;
osThreadId ShootTaskHandle;
osThreadId AimAssistHandle;
osThreadId ChassisTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartGimbalTask(void const *argument);
void StartINSTask(void const *argument);
void StartDetectTask(void const *argument);
void StartShootTask(void const *argument);
void StartAimAssist(void const *argument);
void StartChassisTask(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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
  /* definition and creation of GimbalTask */
  osThreadDef(GimbalTask, StartGimbalTask, osPriorityHigh, 0, 1024);
  GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);

  /* definition and creation of INSTask */
  osThreadDef(INSTask, StartINSTask, osPriorityAboveNormal, 0, 1024);
  INSTaskHandle = osThreadCreate(osThread(INSTask), NULL);

  /* definition and creation of DetectTask */
  osThreadDef(DetectTask, StartDetectTask, osPriorityBelowNormal, 0, 256);
  DetectTaskHandle = osThreadCreate(osThread(DetectTask), NULL);

  /* definition and creation of ShootTask */
  osThreadDef(ShootTask, StartShootTask, osPriorityNormal, 0, 512);
  ShootTaskHandle = osThreadCreate(osThread(ShootTask), NULL);

  /* definition and creation of AimAssist */
  osThreadDef(AimAssist, StartAimAssist, osPriorityAboveNormal, 0, 1528);
  AimAssistHandle = osThreadCreate(osThread(AimAssist), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, StartChassisTask, osPriorityNormal, 0, 256);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartGimbalTask */
/**
 * @brief  Function implementing the GimbalTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGimbalTask */
void StartGimbalTask(void const *argument)
{
  /* USER CODE BEGIN StartGimbalTask */
  static uint32_t count = 0;
  Gimbal_Init();
  osDelay(100);
  /* Infinite loop */
  for (;;)
  {
    if (RC_Update)
    {
      if (resetCount < 2000 && !((remote_control.key_code & Key_R) && (remote_control.key_code & Key_E)))
      {
        Send_RC_Data(&hcan1, RC_Data_Buffer);
        // Send_RC_Data(&hcan2, RC_Data_Buffer);
      }
      RC_Update = 0;
    }
    Gimbal_Control();

    // if (count % 10 == 0)
    //   Send_Gimbal_Info(&hcan1, AimAssist.miniPC_Online);
    count++;
    osDelay(GIMBAL_TASK_PERIOD);
  }
  /* USER CODE END StartGimbalTask */
}

/* USER CODE BEGIN Header_StartINSTask */
/**
 * @brief Function implementing the INSTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartINSTask */
void StartINSTask(void const *argument)
{
  /* USER CODE BEGIN StartINSTask */
  INS_Init();
  /* Infinite loop */
  for (;;)
  {
    if ((remote_control.key_code & Key_R) && (remote_control.key_code & Key_E))
    {
      resetCount++;
      if (resetCount * DETECT_TASK_PERIOD > 1500)
      {
        // Send_Reset_Command(&hcan1);
        __set_FAULTMASK(1);
        HAL_NVIC_SystemReset();
      }
    }
    else
      resetCount = 0;
    INS_Task();
    osDelay(INS_TASK_PERIOD);
  }
  /* USER CODE END StartINSTask */
}

/* USER CODE BEGIN Header_StartDetectTask */
/**
 * @brief Function implementing the DetectTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDetectTask */
void StartDetectTask(void const *argument)
{
  /* USER CODE BEGIN StartDetectTask */
  Detect_Task_Init();
  /* Infinite loop */
  for (;;)
  {
    // HAL_IWDG_Refresh(&hiwdg);

    Detect_Task();

    osDelay(DETECT_TASK_PERIOD);
  }
  /* USER CODE END StartDetectTask */
}

/* USER CODE BEGIN Header_StartShootTask */
/**
 * @brief Function implementing the ShootTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartShootTask */
void StartShootTask(void const *argument)
{
  /* USER CODE BEGIN StartShootTask */
  Shoot_Init();
  osDelay(100);
  /* Infinite loop */
  for (;;)
  {
    Shoot_Control();
    osDelay(SHOOT_TASK_PERIOD);
  }
  /* USER CODE END StartShootTask */
}

/* USER CODE BEGIN Header_StartAimAssist */
/**
 * @brief Function implementing the AimAssist thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartAimAssist */
void StartAimAssist(void const *argument)
{
  static uint32_t count = 0;
  /* USER CODE BEGIN StartAimAssist */
  AimAssist_Init(&huart6);
  /* Infinite loop */
  for (;;)
  {
    AimAssist_Task();
    osDelay(AUTOAIM_TASK_PERIOD);

    if (count % 5 == 0)
      Send_Gimbal_Info(&hcan1, Gimbal.aim_status);
    count++;
  }
  /* USER CODE END StartAimAssist */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
 * @brief Function implementing the ChassisTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void const *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartChassisTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
