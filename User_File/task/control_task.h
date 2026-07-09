#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void control_task_init(void);
void control_task(void);
void control_task_safety_poll(void);

#ifdef __cplusplus
}
#endif

#endif
