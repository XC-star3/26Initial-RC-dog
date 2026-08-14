#include "tim.h"

TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

void MX_TIM7_Init(void)
{
  TIM_MasterConfigTypeDef master = {0};
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 240 - 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 500 - 1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &master) != HAL_OK)
  {
    Error_Handler();
  }
}

void MX_TIM8_Init(void)
{
  TIM_ClockConfigTypeDef clock = {0};
  TIM_MasterConfigTypeDef master = {0};
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 240 - 1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000 - 1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  clock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &clock) != HAL_OK)
  {
    Error_Handler();
  }
  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &master) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *timer)
{
  if (timer->Instance == TIM7)
  {
    __HAL_RCC_TIM7_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
  }
  else if (timer->Instance == TIM8)
  {
    __HAL_RCC_TIM8_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *timer)
{
  if (timer->Instance == TIM7)
  {
    __HAL_RCC_TIM7_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
  }
  else if (timer->Instance == TIM8)
  {
    __HAL_RCC_TIM8_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
  }
}
