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
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "board_rgb.h"
#include "control_task.h"
#include "debug_uart.h"
#include "motor_task_c_api.h"
#include "wheel_motor_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
static volatile uint8_t s_app_ready = 0U;
static volatile uint8_t s_motor_ready = 0U;

/* USER CODE END Variables */
/* Definitions for Start_Control_Task */
osThreadId_t Start_Control_TaskHandle;
uint32_t Start_Control_TaskBuffer[1024];
osStaticThreadDef_t Start_Control_TaskControlBlock;
const osThreadAttr_t Start_Control_Task_attributes = {
  .name = "Start_Control_Task",
  .cb_mem = &Start_Control_TaskControlBlock,
  .cb_size = sizeof(Start_Control_TaskControlBlock),
  .stack_mem = &Start_Control_TaskBuffer[0],
  .stack_size = sizeof(Start_Control_TaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Start_CAN_Task */
osThreadId_t Start_CAN_TaskHandle;
uint32_t Start_CAN_TaskBuffer[1024];
osStaticThreadDef_t Start_CAN_TaskControlBlock;
const osThreadAttr_t Start_CAN_Task_attributes = {
  .name = "Start_CAN_Task",
  .cb_mem = &Start_CAN_TaskControlBlock,
  .cb_size = sizeof(Start_CAN_TaskControlBlock),
  .stack_mem = &Start_CAN_TaskBuffer[0],
  .stack_size = sizeof(Start_CAN_TaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Start_Wheel_Task */
osThreadId_t Start_Wheel_TaskHandle;
uint32_t Start_Wheel_TaskBuffer[512];
osStaticThreadDef_t Start_Wheel_TaskControlBlock;
const osThreadAttr_t Start_Wheel_Task_attributes = {
  .name = "Start_Wheel_Task",
  .cb_mem = &Start_Wheel_TaskControlBlock,
  .cb_size = sizeof(Start_Wheel_TaskControlBlock),
  .stack_mem = &Start_Wheel_TaskBuffer[0],
  .stack_size = sizeof(Start_Wheel_TaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Control_Task(void *argument);
void CAN_Task(void *argument);
void Wheel_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
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
  /* creation of Start_Control_Task */
  Start_Control_TaskHandle = osThreadNew(Control_Task, NULL, &Start_Control_Task_attributes);

  /* creation of Start_CAN_Task */
  Start_CAN_TaskHandle = osThreadNew(CAN_Task, NULL, &Start_CAN_Task_attributes);

  /* creation of Start_Wheel_Task */
  Start_Wheel_TaskHandle = osThreadNew(Wheel_Task, NULL, &Start_Wheel_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Control_Task */
/**
  * @brief  Function implementing the Start_Control_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Control_Task */
void Control_Task(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN Control_Task */
  (void)argument;
  DebugUart_Init();
  BoardRgb_SetColor(0U, 255U, 0U);
  DebugUart_Printf("quadruped SDK debug boot ok, CAN1/2 legs, CAN3 wheels 0x201..0x204\r\n");
  motor_task_init();
  s_motor_ready = 1U;
  control_task_init();
  s_app_ready = 1U;

  /* Infinite loop */
  for(;;)
  {
    control_task();
    osDelay(1);
  }
  /* USER CODE END Control_Task */
}

/* USER CODE BEGIN Header_CAN_Task */
/**
  * @brief  Function implementing the Start_CAN_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_CAN_Task */
void CAN_Task(void *argument)
{
  /* USER CODE BEGIN CAN_Task */
  (void)argument;
  while (s_motor_ready == 0U) {
    osDelay(1);
  }
  while (s_app_ready == 0U) {
    motor_can_rx_tick();
    osDelay(1);
  }

  for(;;)
  {
    motor_task_tick();
    osDelay(1);
  }
  /* USER CODE END CAN_Task */
}

void Wheel_Task(void *argument)
{
  (void)argument;
  while (s_motor_ready == 0U) {
    osDelay(1U);
  }

  uint32_t next_wake = osKernelGetTickCount();
  for (;;) {
    WheelDrive_Tick(HAL_GetTick());
    next_wake += 2U;
    (void)osDelayUntil(next_wake);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
