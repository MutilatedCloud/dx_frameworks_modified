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

// #include "DBusSys.h"
// #include "dma.h"
// #include "usart.h"
// #include "PIDtool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IMU_TempCtrl */
osThreadId_t IMU_TempCtrlHandle;
uint32_t IMU_TempCtrlBuffer[ 128 ];
osStaticThreadDef_t IMU_TempCtrlControlBlock;
const osThreadAttr_t IMU_TempCtrl_attributes = {
  .name = "IMU_TempCtrl",
  .cb_mem = &IMU_TempCtrlControlBlock,
  .cb_size = sizeof(IMU_TempCtrlControlBlock),
  .stack_mem = &IMU_TempCtrlBuffer[0],
  .stack_size = sizeof(IMU_TempCtrlBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Remoter */
osThreadId_t RemoterHandle;
uint32_t RemoterBuffer[ 256 ];
osStaticThreadDef_t RemoterControlBlock;
const osThreadAttr_t Remoter_attributes = {
  .name = "Remoter",
  .cb_mem = &RemoterControlBlock,
  .cb_size = sizeof(RemoterControlBlock),
  .stack_mem = &RemoterBuffer[0],
  .stack_size = sizeof(RemoterBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartTest */
osThreadId_t uartTestHandle;
uint32_t uartTestBuffer[512];
osStaticThreadDef_t uartTestControlBlock;
const osThreadAttr_t uartTest_attributes = {
    .name       = "uartTest",
    .cb_mem     = &uartTestControlBlock,
    .cb_size    = sizeof(uartTestControlBlock),
    .stack_mem  = &uartTestBuffer[0],
    .stack_size = sizeof(uartTestBuffer),
    .priority   = (osPriority_t)osPriorityNormal,
};
/* Definitions for imuBinarySem01 */
osSemaphoreId_t imuBinarySem01Handle;
osStaticSemaphoreDef_t imuBinarySemControlBlock;
const osSemaphoreAttr_t imuBinarySem01_attributes = {
  .name = "imuBinarySem01",
  .cb_mem = &imuBinarySemControlBlock,
  .cb_size = sizeof(imuBinarySemControlBlock),
};
/* Definitions for controlBinaryIMU */
osSemaphoreId_t controlBinaryIMUHandle;
osStaticSemaphoreDef_t controlBinaryIMUControlBlock;
const osSemaphoreAttr_t controlBinaryIMU_attributes = {
  .name = "controlBinaryIMU",
  .cb_mem = &controlBinaryIMUControlBlock,
  .cb_size = sizeof(controlBinaryIMUControlBlock),
};

osThreadId_t Controller_TaskHandle;
uint32_t  Controller_TaskBuffer[1024];
osStaticThreadDef_t  Controller_TaskControlBlock;
const osThreadAttr_t  Controller_Task_attributes = {
    .name       = " Controller_Task",
    .cb_mem     = & Controller_TaskControlBlock,
    .cb_size    = sizeof( Controller_TaskControlBlock),
    .stack_mem  = & Controller_TaskBuffer[0],
    .stack_size = sizeof(Controller_TaskBuffer),
    .priority   = (osPriority_t)osPriorityNormal,
};

osThreadId_t Referee_TaskHandle;
uint32_t  Referee_TaskBuffer[256];
osStaticThreadDef_t  Referee_TaskControlBlock;
const osThreadAttr_t  Referee_Task_attributes = {
    .name       = " Referee_Task",
    .cb_mem     = & Referee_TaskControlBlock,
    .cb_size    = sizeof( Referee_TaskControlBlock),
    .stack_mem  = & Referee_TaskBuffer[0],
    .stack_size = sizeof(Referee_TaskBuffer),
    .priority   = (osPriority_t)osPriorityNormal,
};


uint32_t Buzzer_TaskBuffer[1024];  // 栈大小：1024 * 4字节 = 4096字节
osStaticThreadDef_t Buzzer_TaskControlBlock;  // 静态任务控制块
osThreadId_t Buzzer_TaskHandle;  // 任务句柄
const osThreadAttr_t Buzzer_Task_attributes = {
  .name = "Buzzer_Task",        // 任务名称（调试用）
  .cb_mem = &Buzzer_TaskControlBlock,  // 控制块地址（静态创建）
  .cb_size = sizeof(Buzzer_TaskControlBlock),  // 控制块大小
  .stack_mem = &Buzzer_TaskBuffer[0],  // 栈空间地址
  .stack_size = sizeof(Buzzer_TaskBuffer),  // 栈大小
  .priority = (osPriority_t)osPriorityNormal,  // 优先级（与默认任务相同）
};
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void uart_test(void *argument);
void Controller_Task(void *argument);
void Referee_Task(void *argument);
void Buzzer_Task(void *arguments);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void IMU_TempCtrlTask(void *argument);
void Remoter_Task(void *argument);


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

  /* Create the semaphores(s) */
  /* creation of imuBinarySem01 */
  imuBinarySem01Handle = osSemaphoreNew(1, 0, &imuBinarySem01_attributes);

  /* creation of controlBinaryIMU */
  controlBinaryIMUHandle = osSemaphoreNew(1, 0, &controlBinaryIMU_attributes);

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

  /* creation of IMU_TempCtrl */
  IMU_TempCtrlHandle = osThreadNew(IMU_TempCtrlTask, NULL, &IMU_TempCtrl_attributes);

  /* creation of Remoter */
  RemoterHandle = osThreadNew(Remoter_Task, NULL, &Remoter_attributes);

  
  uartTestHandle = osThreadNew(uart_test, NULL, &uartTest_attributes);
  Controller_TaskHandle = osThreadNew(Controller_Task, NULL, &Controller_Task_attributes);
  Referee_TaskHandle = osThreadNew(Referee_Task, NULL, &Referee_Task_attributes);
  Buzzer_TaskHandle = osThreadNew(Buzzer_Task, NULL, &Buzzer_Task_attributes);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  UNUSED(argument);
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_IMU_TempCtrlTask */
/**
* @brief Function implementing the IMU_TempCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_TempCtrlTask */
__weak void IMU_TempCtrlTask(void *argument)
{
  /* USER CODE BEGIN IMU_TempCtrlTask */
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_TempCtrlTask */
}

/* USER CODE BEGIN Header_Remoter_Task */
/**
* @brief Function implementing the Remoter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remoter_Task */
__weak void Remoter_Task(void *argument)
{
  /* USER CODE BEGIN Remoter_Task */
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Remoter_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

__weak void uart_test(void *argument)
{
  UNUSED(argument);
  for (;;) {
    osDelay(1);
  }
}

__weak void Controller_Task(void *argument)
{
  UNUSED(argument);
  for (;;) {
    
    osDelay(1);
  }
}

__weak void Referee_Task(void *argument)
{
  /* USER CODE BEGIN Referee_Task */
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Referee_Task */
}

__weak void Buzzer_Task(void *argument)
{
  UNUSED(argument);
  for(;;)
  {
    osDelay(1);
  }
}
/* USER CODE END Application */

