#ifndef VOFA_PID_H
#define VOFA_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct Dog_Mit_Pid_Telemetry;

void VofaPid_Init(void);
void VofaPid_SetEnabled(uint8_t enable);
uint8_t VofaPid_IsEnabled(void);
void VofaPid_SetMotorIndex(uint8_t motor_index);
uint8_t VofaPid_GetMotorIndex(void);
void VofaPid_CycleMotorIndex(void);
void VofaPid_SendTelemetry(const struct Dog_Mit_Pid_Telemetry *telemetry);
uint8_t VofaPid_FeedRxByte(uint8_t byte);

#ifdef __cplusplus
}
#endif

#endif
