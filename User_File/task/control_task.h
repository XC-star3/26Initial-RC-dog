#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化 SBUS、USB 虚拟遥控器和控制模式状态。 */
void control_task_init(void);
/* 周期性高层控制任务：读取操作者输入并更新工作模式。 */
void control_task(void);
/* 常规控制任务未运行时使用的轻量级安全路径。 */
void control_task_safety_poll(void);

#ifdef __cplusplus
}
#endif

#endif
