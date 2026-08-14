#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;

void MX_TIM7_Init(void);
void MX_TIM8_Init(void);

#ifdef __cplusplus
}
#endif

#endif
