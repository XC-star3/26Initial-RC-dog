#ifndef ARM_MOTOR_TASK_H
#define ARM_MOTOR_TASK_H

#include "bsp_fdcan.h"
#include "struct_typedef.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ARM_JOINT_COUNT 2U
#define ARM_J0_DM4310 0U
#define ARM_J1_LZ 1U

struct ArmMotorFeedback {
    uint8_t online;
    uint8_t error;
    uint8_t mode;
    fp32 pos_rad;
    fp32 angle_deg;
    fp32 vel_rad_s;
    fp32 torque_nm;
    fp32 temperature_c;
};

struct ArmMotorPidConfig {
    fp32 kp;
    fp32 ki;
    fp32 kd;
    fp32 max_i;
    fp32 max_out;
};

void ArmMotor_Init(FDCAN_HandleTypeDef *j0_dm_can,
                   uint16_t j0_dm_can_id,
                   uint16_t j0_dm_feedback_id,
                   FDCAN_HandleTypeDef *j1_lz_can,
                   uint16_t j1_lz_can_id,
                   uint8_t j1_lz_model);
void ArmMotor_Enable(void);
void ArmMotor_Disable(void);
void ArmMotor_Zero(uint8_t joint);
void ArmMotor_SetTargetRad(uint8_t joint, fp32 pos_rad, fp32 vel_rad_s, fp32 torque_nm);
void ArmMotor_SetTargetDeg(uint8_t joint, fp32 angle_deg, fp32 vel_rad_s, fp32 torque_nm);
void ArmMotor_SetGains(uint8_t joint, fp32 kp, fp32 kd);
void ArmMotor_SetJ0LimitsDeg(fp32 min_deg, fp32 max_deg);
void ArmMotor_SetJ0OffsetDeg(fp32 offset_deg);
void ArmMotor_SetJ0Invert(uint8_t enable);
void ArmMotor_SetJ0Pid(const ArmMotorPidConfig *angle_pid, const ArmMotorPidConfig *speed_pid);
void ArmMotor_SetJ0GravityComp(uint8_t enable, fp32 max_torque_nm, fp32 horizontal_deg);
void ArmMotor_SetJ1LimitsDeg(fp32 min_deg, fp32 max_deg);
void ArmMotor_SetJ1OffsetDeg(fp32 offset_deg);
void ArmMotor_SetJ1Invert(uint8_t enable);
void ArmMotor_SetJ1MasterId(uint8_t master_id);
void ArmMotor_Send(void);
uint8_t ArmMotor_OnCanRx(FDCAN_HandleTypeDef *hfdcan, const FDCAN_RxHeaderTypeDef *header, uint8_t *data);
uint8_t ArmMotor_GetFeedback(uint8_t joint, ArmMotorFeedback *feedback);
uint8_t ArmMotor_IsInitialized(void);

#ifdef __cplusplus
}
#endif

#endif
