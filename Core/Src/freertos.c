#include "FreeRTOS.h"
#include "app_main.h"
#include "task.h"

static StackType_t startup_task_stack[2048];
static StaticTask_t startup_task_control_block;
static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];
static StaticTask_t idle_task_control_block;

void vApplicationGetIdleTaskMemory(StaticTask_t **task_control_block,
                                   StackType_t **task_stack,
                                   uint32_t *stack_size)
{
  *task_control_block = &idle_task_control_block;
  *task_stack = idle_task_stack;
  *stack_size = configMINIMAL_STACK_SIZE;
}

static void StartupTask(void *argument)
{
  (void)argument;
  app_main();
  vTaskDelete(NULL);
}

void MX_FREERTOS_Init(void)
{
  (void)xTaskCreateStatic(StartupTask, "xrobot_start",
                          sizeof(startup_task_stack) / sizeof(StackType_t), NULL,
                          tskIDLE_PRIORITY + 1U, startup_task_stack,
                          &startup_task_control_block);
}
