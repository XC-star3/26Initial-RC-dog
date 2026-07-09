#ifndef MOTOR_TASK_C_API_H
#define MOTOR_TASK_C_API_H

#ifdef __cplusplus
extern "C" {
#endif

void motor_task_init(void);
void motor_can_rx_tick(void);
void motor_task_tick(void);

#ifdef __cplusplus
}
#endif

#endif
