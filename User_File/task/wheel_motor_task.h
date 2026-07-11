#ifndef WHEEL_MOTOR_TASK_H
#define WHEEL_MOTOR_TASK_H

#include "fdcan.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WHEEL_MOTOR_COUNT 4U
#define WHEEL_MAX_OUTPUT_RPM 200.0f

typedef enum WheelDriveProfile {
    WHEEL_PROFILE_NORMAL = 0,
    WHEEL_PROFILE_MECHANICAL_CRAWL,
} WheelDriveProfile;

typedef enum WheelDriveOperatingMode {
    WHEEL_OPERATING_OFF = 0,
    WHEEL_OPERATING_HOLD,
    WHEEL_OPERATING_DRIVE,
} WheelDriveOperatingMode;

typedef struct WheelMotorFeedback {
    uint16_t encoder_raw;
    int32_t encoder_rounds;
    float output_position_rad;
    float output_speed_rad_s;
    int16_t torque_current_raw;
    uint8_t temperature_c;
    uint8_t received;
    uint32_t last_update_ms;
} WheelMotorFeedback;

typedef struct WheelDriveDiag {
    uint8_t can_ready;
    uint8_t mode_enabled;
    uint8_t locked;
    uint8_t all_online;
    uint8_t stopped;
    uint8_t feedback_seen_mask;
    uint8_t profile;
    uint8_t operating_mode;
    uint8_t brake_active;
    uint8_t peak_limited_mask;
    uint8_t thermal_derated_mask;
    uint8_t overtemp_mask;
    float requested_left_rpm;
    float requested_right_rpm;
    float requested_target_rpm[WHEEL_MOTOR_COUNT];
    float phase_scale[WHEEL_MOTOR_COUNT];
    float final_target_rpm[WHEEL_MOTOR_COUNT];
    uint8_t hybrid_mode;
    float ramped_target_rpm[WHEEL_MOTOR_COUNT];
    float vehicle_speed_rpm[WHEEL_MOTOR_COUNT];
    int16_t current_cmd[WHEEL_MOTOR_COUNT];
    int16_t current_limit_raw[WHEEL_MOTOR_COUNT];
    uint16_t peak_budget_ms[WHEEL_MOTOR_COUNT];
    uint32_t tx_fail_count;
    uint32_t bus_off_count;
    uint32_t feedback_timeout_count;
    uint32_t command_timeout_count;
    uint32_t rx_reject_count;
    WheelMotorFeedback motor[WHEEL_MOTOR_COUNT];
} WheelDriveDiag;

void WheelDrive_Init(FDCAN_HandleTypeDef *hfdcan);
uint8_t WheelDrive_OnCanRx(const FDCAN_RxHeaderTypeDef *header, const uint8_t data[8]);
/* forward/yaw are normalized to [-1, 1]; max_rpm is an output-shaft limit. */
void WheelDrive_SetMotion(float forward, float yaw, float max_rpm);
void WheelDrive_SetMotionScaled(float forward, float yaw, float max_rpm,
                                const float phase_scale[WHEEL_MOTOR_COUNT]);
void WheelDrive_SetWheelTargets(const float target_rpm[WHEEL_MOTOR_COUNT],
                                const float phase_scale[WHEEL_MOTOR_COUNT]);
void WheelDrive_SetProfile(WheelDriveProfile profile);
void WheelDrive_SetOperatingMode(WheelDriveOperatingMode mode);
void WheelDrive_HoldIfEnabled(void);
void WheelDrive_Enable(void);
void WheelDrive_Disable(void);
void WheelDrive_Tick(uint32_t now_ms);
void WheelDrive_StopAndLock(void);
uint8_t WheelDrive_TryClearLock(void);
uint8_t WheelDrive_AllOnline(void);
uint8_t WheelDrive_IsAvailable(void);
uint8_t WheelDrive_IsStopped(void);
void WheelDrive_GetDiag(WheelDriveDiag *diag);

#ifdef __cplusplus
}
#endif

#endif
