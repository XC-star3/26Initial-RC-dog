#include "wheel_motor_task.h"

#include "bsp_fdcan.h"
#include "motor_task.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include <math.h>
#include <string.h>

#define WHEEL_CAN_CONTROL_ID             0x200U
#define WHEEL_CAN_FEEDBACK_FIRST_ID      0x201U
#define WHEEL_CAN_FEEDBACK_LAST_ID       0x204U
#define WHEEL_CAN_NOMINAL_BAUD            1000000UL
#define WHEEL_CAN_NOMINAL_PRESCALER       4U
#define WHEEL_CAN_NOMINAL_SYNC_JUMP       8U
#define WHEEL_CAN_NOMINAL_TIME_SEG1       31U
#define WHEEL_CAN_NOMINAL_TIME_SEG2       8U
#define WHEEL_ENCODER_COUNTS             8192
#define WHEEL_ENCODER_HALF_COUNTS        (WHEEL_ENCODER_COUNTS / 2)
#define WHEEL_GEAR_RATIO                 (268.0f / 17.0f)
#define WHEEL_TWO_PI                     6.28318530717958647692f
#define WHEEL_RPM_TO_RAD_S               (WHEEL_TWO_PI / 60.0f)
#define WHEEL_RAD_S_TO_RPM               (60.0f / WHEEL_TWO_PI)
#define WHEEL_CONTROL_PERIOD_MS          2U
#define WHEEL_FEEDBACK_TIMEOUT_MS         250U
#define WHEEL_COMMAND_TIMEOUT_MS          100U
#define WHEEL_LOCK_CLEAR_STABLE_MS       200U
#define WHEEL_BUS_CHECK_PERIOD_MS        100U
#define WHEEL_TX_RETRY_BACKOFF_MS         10U
#define WHEEL_STOP_SPEED_RAD_S           0.5f
#define WHEEL_ACCEL_RAD_S2               30.0f
#define WHEEL_DECEL_RAD_S2               50.0f
#define WHEEL_BRAKE_RAD_S2               60.0f
#define WHEEL_PI_KP                      0.015f
#define WHEEL_PI_KI_PER_S                0.25f
#define WHEEL_CURRENT_RAW_SCALE          10000.0f
#define WHEEL_C620_FULL_SCALE_RAW        16384.0f
#define WHEEL_C620_FULL_SCALE_CURRENT_A  20.0f
#define WHEEL_CONTINUOUS_CURRENT_A       6.0f
#define WHEEL_PEAK_CURRENT_A             10.0f
#define WHEEL_CONTINUOUS_CURRENT_RAW     ((int32_t)(WHEEL_CONTINUOUS_CURRENT_A / WHEEL_C620_FULL_SCALE_CURRENT_A * WHEEL_C620_FULL_SCALE_RAW + 0.5f))
#define WHEEL_PEAK_CURRENT_RAW           ((int32_t)(WHEEL_PEAK_CURRENT_A / WHEEL_C620_FULL_SCALE_CURRENT_A * WHEEL_C620_FULL_SCALE_RAW + 0.5f))
#define WHEEL_PEAK_BUDGET_MS             500.0f
#define WHEEL_PEAK_RECOVERY_SCALE        0.5f
#define WHEEL_TEMP_DERATE_START_C        70U
#define WHEEL_TEMP_CUTOFF_C              85U
#define WHEEL_TEMP_RECOVER_C             65U
#define WHEEL_QUICK_TURN_FULL            0.05f
#define WHEEL_QUICK_TURN_END             0.25f
#define WHEEL_STRONG_TURN_START          0.45f
#define WHEEL_STRONG_TURN_FULL           0.85f
#define WHEEL_STRONG_TURN_FORWARD_SCALE  0.5f
#define WHEEL_MOTION_ZERO_EPSILON        0.0001f

/* LF, RF, RB, LB motor signs for positive vehicle-forward wheel speed. */
static const float s_forward_sign[WHEEL_MOTOR_COUNT] = {-1.0f, 1.0f, 1.0f, -1.0f};

static FDCAN_HandleTypeDef *s_can = nullptr;
static WheelMotorFeedback s_feedback[WHEEL_MOTOR_COUNT];
static StaticSemaphore_t s_feedback_mutex_storage;
static SemaphoreHandle_t s_feedback_mutex = nullptr;
static float s_integral[WHEEL_MOTOR_COUNT];
static int16_t s_current_cmd[WHEEL_MOTOR_COUNT];
static int16_t s_active_current_limit_raw[WHEEL_MOTOR_COUNT];
static float s_peak_budget_ms[WHEEL_MOTOR_COUNT];
static float s_ramped_target_rad_s[WHEEL_MOTOR_COUNT];
static volatile float s_requested_target_rpm[WHEEL_MOTOR_COUNT];
static volatile float s_phase_scale[WHEEL_MOTOR_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
static volatile float s_requested_left_rpm = 0.0f;
static volatile float s_requested_right_rpm = 0.0f;
static volatile uint8_t s_hybrid_mode = 0U;
static volatile uint8_t s_brake_requested = 0U;
static volatile uint8_t s_brake_active = 0U;
static volatile uint32_t s_motion_generation = 0U;
static volatile uint8_t s_mode_enabled = 0U;
static volatile WheelDriveOperatingMode s_operating_mode = WHEEL_OPERATING_OFF;
static volatile uint8_t s_locked = 0U;
static volatile uint8_t s_can_ready = 0U;
static volatile uint8_t s_can_config_valid = 0U;
static volatile uint8_t s_reset_pending = 0U;
static volatile uint8_t s_zero_pending = 0U;
static volatile uint32_t s_state_generation = 0U;
static volatile WheelDriveProfile s_profile = WHEEL_PROFILE_NORMAL;
static uint8_t s_feedback_seen_mask = 0U;
static uint8_t s_timeout_latched = 0U;
static uint8_t s_bus_off_active = 0U;
static uint32_t s_init_ms = 0U;
static uint32_t s_last_control_ms = 0U;
static uint32_t s_last_bus_check_ms = 0U;
static uint32_t s_next_tx_attempt_ms = 0U;
static volatile uint32_t s_stop_stable_since_ms = 0U;
static uint32_t s_tx_fail_count = 0U;
static uint32_t s_bus_off_count = 0U;
static uint32_t s_feedback_timeout_count = 0U;
static uint32_t s_command_timeout_count = 0U;
static uint32_t s_rx_reject_count = 0U;
static volatile uint32_t s_last_command_ms = 0U;
static volatile uint8_t s_peak_limited_mask = 0U;
static volatile uint8_t s_thermal_derated_mask = 0U;
static volatile uint8_t s_overtemp_mask = 0U;

static uint8_t feedback_lock(void)
{
    if (s_feedback_mutex == nullptr) {
        return 0U;
    }
    return (xSemaphoreTake(s_feedback_mutex, portMAX_DELAY) == pdTRUE) ? 1U : 0U;
}

static uint8_t wheel_can_bus_number(const FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == nullptr) {
        return 0U;
    }
    if (hfdcan->Instance == FDCAN1) {
        return 1U;
    }
    if (hfdcan->Instance == FDCAN2) {
        return 2U;
    }
    return (hfdcan->Instance == FDCAN3) ? 3U : 0U;
}

static uint8_t wheel_can_config_valid(const FDCAN_HandleTypeDef *hfdcan)
{
    return ((wheel_can_bus_number(hfdcan) == 3U) &&
            (hfdcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC) &&
            (hfdcan->Init.Mode == FDCAN_MODE_NORMAL) &&
            (hfdcan->Init.NominalPrescaler == WHEEL_CAN_NOMINAL_PRESCALER) &&
            (hfdcan->Init.NominalSyncJumpWidth == WHEEL_CAN_NOMINAL_SYNC_JUMP) &&
            (hfdcan->Init.NominalTimeSeg1 == WHEEL_CAN_NOMINAL_TIME_SEG1) &&
            (hfdcan->Init.NominalTimeSeg2 == WHEEL_CAN_NOMINAL_TIME_SEG2)) ? 1U : 0U;
}

static void feedback_unlock(void)
{
    if (s_feedback_mutex != nullptr) {
        (void)xSemaphoreGive(s_feedback_mutex);
    }
}

static uint8_t all_online_locked(uint32_t now)
{
    if ((s_can_ready == 0U) || (s_feedback_seen_mask != 0x0FU)) {
        return 0U;
    }
    for (uint8_t i = 0U; i < WHEEL_MOTOR_COUNT; ++i) {
        if ((uint32_t)(now - s_feedback[i].last_update_ms) > WHEEL_FEEDBACK_TIMEOUT_MS) {
            return 0U;
        }
    }
    return 1U;
}

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void reset_controller(void)
{
    memset(s_integral, 0, sizeof(s_integral));
    memset(s_current_cmd, 0, sizeof(s_current_cmd));
    memset(s_active_current_limit_raw, 0, sizeof(s_active_current_limit_raw));
    memset(s_ramped_target_rad_s, 0, sizeof(s_ramped_target_rad_s));
    s_peak_limited_mask = 0U;
    s_thermal_derated_mask = 0U;
}

static float smoothstep01(float value)
{
    const float x = clamp_float(value, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

static int32_t thermal_peak_limit_raw(uint8_t temperature_c)
{
    if (temperature_c <= WHEEL_TEMP_DERATE_START_C) {
        return WHEEL_PEAK_CURRENT_RAW;
    }
    if (temperature_c >= WHEEL_TEMP_CUTOFF_C) {
        return WHEEL_CONTINUOUS_CURRENT_RAW;
    }
    const float derate = (float)(temperature_c - WHEEL_TEMP_DERATE_START_C) /
                         (float)(WHEEL_TEMP_CUTOFF_C - WHEEL_TEMP_DERATE_START_C);
    return (int32_t)((float)WHEEL_PEAK_CURRENT_RAW -
                     derate * (float)(WHEEL_PEAK_CURRENT_RAW -
                                      WHEEL_CONTINUOUS_CURRENT_RAW));
}

static void commit_motion(const float target_rpm[WHEEL_MOTOR_COUNT],
                          const float phase_scale[WHEEL_MOTOR_COUNT],
                          uint8_t brake, uint8_t hybrid_mode)
{
    float targets[WHEEL_MOTOR_COUNT] = {};
    float scales[WHEEL_MOTOR_COUNT] = {};
    for (uint8_t i = 0U; i < WHEEL_MOTOR_COUNT; ++i) {
        targets[i] = clamp_float(target_rpm[i], -WHEEL_MAX_OUTPUT_RPM,
                                 WHEEL_MAX_OUTPUT_RPM);
        scales[i] = clamp_float(phase_scale[i], 0.0f, 1.0f);
    }

    taskENTER_CRITICAL();
    uint8_t changed = ((s_brake_requested != brake) ||
                       (s_hybrid_mode != hybrid_mode)) ? 1U : 0U;
    for (uint8_t i = 0U; i < WHEEL_MOTOR_COUNT; ++i) {
        if ((s_requested_target_rpm[i] != targets[i]) ||
            (s_phase_scale[i] != scales[i])) {
            changed = 1U;
        }
        s_requested_target_rpm[i] = targets[i];
        s_phase_scale[i] = scales[i];
    }
    if (changed != 0U) {
        s_requested_left_rpm = targets[0U];
        s_requested_right_rpm = targets[1U];
        s_brake_requested = brake;
        s_hybrid_mode = hybrid_mode;
        s_motion_generation++;
    }
    taskEXIT_CRITICAL();
}

static void clear_motion(void)
{
    const float zero[WHEEL_MOTOR_COUNT] = {};
    const float unity[WHEEL_MOTOR_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
    commit_motion(zero, unity, 0U, 0U);
}

static void snapshot_motion(float target_rpm[WHEEL_MOTOR_COUNT],
                            float phase_scale[WHEEL_MOTOR_COUNT],
                            uint8_t *brake, uint8_t *hybrid_mode,
                            uint32_t *generation)
{
    taskENTER_CRITICAL();
    for (uint8_t i = 0U; i < WHEEL_MOTOR_COUNT; ++i) {
        target_rpm[i] = s_requested_target_rpm[i];
        phase_scale[i] = s_phase_scale[i];
    }
    *brake = s_brake_requested;
    *hybrid_mode = s_hybrid_mode;
    *generation = s_motion_generation;
    taskEXIT_CRITICAL();
}

static uint8_t send_currents(const int16_t current[WHEEL_MOTOR_COUNT])
{
    if ((s_can == nullptr) || (s_can_ready == 0U)) {
        return 0U;
    }

    const uint32_t now_ms = HAL_GetTick();
    if ((int32_t)(now_ms - s_next_tx_attempt_ms) < 0) {
        return 0U;
    }

    uint8_t tx[8] = {};
    for (uint8_t id = 1U; id <= WHEEL_MOTOR_COUNT; ++id) {
        const uint8_t group = (uint8_t)((id - 1U) / 4U);
        const uint8_t slot = (uint8_t)((id - 1U) % 4U);
        if (group != 0U) {
            continue;
        }
        const uint16_t raw = (uint16_t)current[slot];
        tx[slot * 2U] = (uint8_t)(raw >> 8);
        tx[slot * 2U + 1U] = (uint8_t)raw;
    }

    if (fdcan_send_std8(s_can, WHEEL_CAN_CONTROL_ID, tx) != 0U) {
        s_tx_fail_count++;
        s_next_tx_attempt_ms = now_ms + WHEEL_TX_RETRY_BACKOFF_MS;
        return 0U;
    }
    s_next_tx_attempt_ms = now_ms;
    return 1U;
}

static uint8_t send_zero(void)
{
    const int16_t zero[WHEEL_MOTOR_COUNT] = {0, 0, 0, 0};
    return send_currents(zero);
}

static void lock_drive(uint8_t feedback_timeout)
{
    s_state_generation++;
    s_locked = 1U;
    s_mode_enabled = 0U;
    s_operating_mode = WHEEL_OPERATING_OFF;
    clear_motion();
    s_brake_active = 0U;
    s_profile = WHEEL_PROFILE_NORMAL;
    s_stop_stable_since_ms = 0U;
    s_reset_pending = 1U;
    s_zero_pending = 1U;
    if ((feedback_timeout != 0U) && (s_timeout_latched == 0U)) {
        s_feedback_timeout_count++;
        s_timeout_latched = 1U;
    }
}

void WheelDrive_Init(FDCAN_HandleTypeDef *hfdcan)
{
    s_can = hfdcan;
    if (s_feedback_mutex == nullptr) {
        s_feedback_mutex = xSemaphoreCreateMutexStatic(&s_feedback_mutex_storage);
    }
    memset(s_feedback, 0, sizeof(s_feedback));
    memset(s_peak_budget_ms, 0, sizeof(s_peak_budget_ms));
    reset_controller();
    s_mode_enabled = 0U;
    s_operating_mode = WHEEL_OPERATING_OFF;
    s_reset_pending = 0U;
    s_zero_pending = 0U;
    s_state_generation = 0U;
    clear_motion();
    s_brake_active = 0U;
    s_feedback_seen_mask = 0U;
    s_timeout_latched = 0U;
    s_bus_off_active = 0U;
    s_overtemp_mask = 0U;
    s_init_ms = HAL_GetTick();
    s_last_control_ms = s_init_ms;
    s_last_bus_check_ms = s_init_ms;
    s_next_tx_attempt_ms = s_init_ms;
    s_stop_stable_since_ms = 0U;
    s_tx_fail_count = 0U;
    s_bus_off_count = 0U;
    s_feedback_timeout_count = 0U;
    s_command_timeout_count = 0U;
    s_rx_reject_count = 0U;
    s_last_command_ms = s_init_ms;
    s_can_config_valid = wheel_can_config_valid(hfdcan);
    s_can_ready = ((s_can_config_valid != 0U) && system_can[2] &&
                   (s_feedback_mutex != nullptr)) ? 1U : 0U;
    s_locked = (s_can_ready != 0U) ? 0U : 1U;
    s_zero_pending = (s_can_ready != 0U) ? 1U : 0U;
}

uint8_t WheelDrive_OnCanRx(const FDCAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
    if ((header == nullptr) || (data == nullptr) ||
        (header->IdType != FDCAN_STANDARD_ID) ||
        (header->RxFrameType != FDCAN_DATA_FRAME) ||
        (fdcan_dlc_to_bytes(header->DataLength) != 8U) ||
        (header->Identifier < WHEEL_CAN_FEEDBACK_FIRST_ID) ||
        (header->Identifier > WHEEL_CAN_FEEDBACK_LAST_ID)) {
        s_rx_reject_count++;
        return 0U;
    }

    const uint8_t index = (uint8_t)(header->Identifier - WHEEL_CAN_FEEDBACK_FIRST_ID);
    if (feedback_lock() == 0U) {
        s_rx_reject_count++;
        return 0U;
    }
    WheelMotorFeedback *feedback = &s_feedback[index];
    const uint16_t encoder = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);

    if (feedback->received != 0U) {
        const int32_t delta = (int32_t)encoder - (int32_t)feedback->encoder_raw;
        if (delta > WHEEL_ENCODER_HALF_COUNTS) {
            feedback->encoder_rounds--;
        } else if (delta < -WHEEL_ENCODER_HALF_COUNTS) {
            feedback->encoder_rounds++;
        }
    } else {
        feedback->encoder_rounds = 0;
        feedback->received = 1U;
    }

    feedback->encoder_raw = encoder;
    const int32_t encoder_total = feedback->encoder_rounds * WHEEL_ENCODER_COUNTS + encoder;
    feedback->output_position_rad = ((float)encoder_total * WHEEL_TWO_PI) /
                                    ((float)WHEEL_ENCODER_COUNTS * WHEEL_GEAR_RATIO);
    const int16_t rotor_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    feedback->output_speed_rad_s = ((float)rotor_rpm * WHEEL_TWO_PI) /
                                   (60.0f * WHEEL_GEAR_RATIO);
    feedback->torque_current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    feedback->temperature_c = data[6];
    const uint8_t motor_bit = (uint8_t)(1U << index);
    if (feedback->temperature_c >= WHEEL_TEMP_CUTOFF_C) {
        s_overtemp_mask |= motor_bit;
    } else if (feedback->temperature_c <= WHEEL_TEMP_RECOVER_C) {
        s_overtemp_mask &= (uint8_t)~motor_bit;
    }
    feedback->last_update_ms = HAL_GetTick();
    s_feedback_seen_mask |= (uint8_t)(1U << index);
    feedback_unlock();
    return 1U;
}

static void set_motion_scaled(float forward, float yaw, float max_rpm,
                              const float phase_scale[WHEEL_MOTOR_COUNT],
                              uint8_t hybrid_mode)
{
    if (s_operating_mode != WHEEL_OPERATING_DRIVE) {
        return;
    }
    s_last_command_ms = HAL_GetTick();
    if ((!isfinite(forward)) || (!isfinite(yaw)) || (!isfinite(max_rpm))) {
        const float zero[WHEEL_MOTOR_COUNT] = {};
        const float unity[WHEEL_MOTOR_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
        commit_motion(zero, unity, 1U, hybrid_mode);
        return;
    }

    forward = clamp_float(forward, -1.0f, 1.0f);
    yaw = clamp_float(yaw, -1.0f, 1.0f);
    max_rpm = clamp_float(max_rpm, 0.0f, WHEEL_MAX_OUTPUT_RPM);
    const uint8_t brake = (((fabsf(forward) <= WHEEL_MOTION_ZERO_EPSILON) &&
                            (fabsf(yaw) <= WHEEL_MOTION_ZERO_EPSILON)) ||
                           (max_rpm <= WHEEL_MOTION_ZERO_EPSILON)) ? 1U : 0U;

    const float speed = fabsf(forward);
    float quick_turn_weight = 0.0f;
    if (speed <= WHEEL_QUICK_TURN_FULL) {
        quick_turn_weight = 1.0f;
    } else if (speed < WHEEL_QUICK_TURN_END) {
        quick_turn_weight = (WHEEL_QUICK_TURN_END - speed) /
                            (WHEEL_QUICK_TURN_END - WHEEL_QUICK_TURN_FULL);
    }
    const float yaw_scale = speed + quick_turn_weight * (1.0f - speed);
    const float strong_turn = smoothstep01((fabsf(yaw) - WHEEL_STRONG_TURN_START) /
                                           (WHEEL_STRONG_TURN_FULL -
                                            WHEEL_STRONG_TURN_START));
    const float turn_scale = yaw_scale + strong_turn * (1.0f - yaw_scale);
    const float forward_scale = 1.0f - strong_turn *
                               (1.0f - WHEEL_STRONG_TURN_FORWARD_SCALE);
    const float turn = yaw * turn_scale;
    const float mixed_forward = forward * forward_scale;
    float left = mixed_forward + turn;
    float right = mixed_forward - turn;
    const float max_mix = fmaxf(1.0f, fmaxf(fabsf(left), fabsf(right)));
    left /= max_mix;
    right /= max_mix;
    const float targets[WHEEL_MOTOR_COUNT] = {
        left * max_rpm,
        right * max_rpm,
        right * max_rpm,
        left * max_rpm,
    };
    const float unity[WHEEL_MOTOR_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
    commit_motion(targets, (phase_scale != nullptr) ? phase_scale : unity,
                  brake, hybrid_mode);
}

void WheelDrive_SetMotion(float forward, float yaw, float max_rpm)
{
    set_motion_scaled(forward, yaw, max_rpm, nullptr, 0U);
}

void WheelDrive_SetMotionScaled(float forward, float yaw, float max_rpm,
                                const float phase_scale[WHEEL_MOTOR_COUNT])
{
    set_motion_scaled(forward, yaw, max_rpm, phase_scale, 1U);
}

void WheelDrive_SetWheelTargets(const float target_rpm[WHEEL_MOTOR_COUNT],
                                const float phase_scale[WHEEL_MOTOR_COUNT])
{
    if (s_operating_mode != WHEEL_OPERATING_DRIVE) {
        return;
    }
    s_last_command_ms = HAL_GetTick();
    const float unity[WHEEL_MOTOR_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
    if ((target_rpm == nullptr) || (phase_scale == nullptr)) {
        const float zero[WHEEL_MOTOR_COUNT] = {};
        commit_motion(zero, unity, 1U, 1U);
        return;
    }

    uint8_t brake = 1U;
    for (uint8_t i = 0U; i < WHEEL_MOTOR_COUNT; ++i) {
        if ((!isfinite(target_rpm[i])) || (!isfinite(phase_scale[i]))) {
            const float zero[WHEEL_MOTOR_COUNT] = {};
            commit_motion(zero, unity, 1U, 1U);
            return;
        }
        if (fabsf(target_rpm[i] * phase_scale[i]) > WHEEL_MOTION_ZERO_EPSILON) {
            brake = 0U;
        }
    }
    commit_motion(target_rpm, phase_scale, brake, 1U);
}

void WheelDrive_SetProfile(WheelDriveProfile profile)
{
    if ((profile != WHEEL_PROFILE_NORMAL) &&
        (profile != WHEEL_PROFILE_MECHANICAL_CRAWL)) {
        profile = WHEEL_PROFILE_NORMAL;
    }
    if (s_profile != profile) {
        s_profile = profile;
        s_state_generation++;
        s_reset_pending = 1U;
        s_zero_pending = 1U;
    }
}

void WheelDrive_SetOperatingMode(WheelDriveOperatingMode mode)
{
    if ((mode != WHEEL_OPERATING_OFF) &&
        (mode != WHEEL_OPERATING_HOLD) &&
        (mode != WHEEL_OPERATING_DRIVE)) {
        mode = WHEEL_OPERATING_OFF;
    }
    s_last_command_ms = HAL_GetTick();
    if ((s_locked != 0U) && (mode != WHEEL_OPERATING_OFF)) {
        return;
    }
    if (s_operating_mode == mode) {
        return;
    }

    s_state_generation++;
    s_operating_mode = mode;
    s_reset_pending = 1U;
    s_zero_pending = 1U;
    if (mode == WHEEL_OPERATING_OFF) {
        s_mode_enabled = 0U;
        clear_motion();
        s_brake_active = 0U;
    } else if (mode == WHEEL_OPERATING_HOLD) {
        s_mode_enabled = 1U;
        const float zero[WHEEL_MOTOR_COUNT] = {};
        const float unity[WHEEL_MOTOR_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
        commit_motion(zero, unity, 1U, 0U);
    } else {
        s_mode_enabled = 1U;
    }
}

void WheelDrive_HoldIfEnabled(void)
{
    if (s_operating_mode != WHEEL_OPERATING_OFF) {
        WheelDrive_SetOperatingMode(WHEEL_OPERATING_HOLD);
    }
}

void WheelDrive_Enable(void)
{
    WheelDrive_SetOperatingMode(WHEEL_OPERATING_DRIVE);
}

void WheelDrive_Disable(void)
{
    WheelDrive_SetOperatingMode(WHEEL_OPERATING_OFF);
}

uint8_t WheelDrive_AllOnline(void)
{
    const uint32_t now = HAL_GetTick();
    if (feedback_lock() == 0U) {
        return 0U;
    }
    const uint8_t online = all_online_locked(now);
    feedback_unlock();
    return online;
}

uint8_t WheelDrive_IsAvailable(void)
{
    return ((s_can_ready != 0U) && (s_locked == 0U) &&
            (s_overtemp_mask == 0U) && (WheelDrive_AllOnline() != 0U)) ? 1U : 0U;
}

uint8_t WheelDrive_IsStopped(void)
{
    const uint32_t now = HAL_GetTick();
    if (feedback_lock() == 0U) {
        return 0U;
    }
    if (all_online_locked(now) == 0U) {
        feedback_unlock();
        return 0U;
    }
    for (uint8_t i = 0U; i < WHEEL_MOTOR_COUNT; ++i) {
        if (fabsf(s_feedback[i].output_speed_rad_s) >= WHEEL_STOP_SPEED_RAD_S) {
            feedback_unlock();
            return 0U;
        }
    }
    feedback_unlock();
    return 1U;
}

void WheelDrive_StopAndLock(void)
{
    lock_drive(0U);
}

uint8_t WheelDrive_TryClearLock(void)
{
    const uint32_t now = HAL_GetTick();
    if (s_locked == 0U) {
        return WheelDrive_IsStopped();
    }
    if ((s_can_ready == 0U) || (s_overtemp_mask != 0U) ||
        (WheelDrive_IsStopped() == 0U)) {
        s_stop_stable_since_ms = 0U;
        return 0U;
    }
    if (s_stop_stable_since_ms == 0U) {
        s_stop_stable_since_ms = now;
        return 0U;
    }
    if ((uint32_t)(now - s_stop_stable_since_ms) < WHEEL_LOCK_CLEAR_STABLE_MS) {
        return 0U;
    }
    s_locked = 0U;
    s_timeout_latched = 0U;
    s_stop_stable_since_ms = 0U;
    s_state_generation++;
    s_reset_pending = 1U;
    return 1U;
}

static void service_bus(uint32_t now_ms)
{
    if ((s_can == nullptr) ||
        ((uint32_t)(now_ms - s_last_bus_check_ms) < WHEEL_BUS_CHECK_PERIOD_MS)) {
        return;
    }
    s_last_bus_check_ms = now_ms;

    FDCAN_ProtocolStatusTypeDef status = {};
    if (HAL_FDCAN_GetProtocolStatus(s_can, &status) != HAL_OK) {
        return;
    }
    if (status.BusOff != 0U) {
        if (s_bus_off_active == 0U) {
            s_bus_off_active = 1U;
            s_bus_off_count++;
            s_can_ready = 0U;
            if (feedback_lock() != 0U) {
                s_feedback_seen_mask = 0U;
                feedback_unlock();
            }
            lock_drive(0U);
        }
        if (fdcan_recover_bus_off(s_can) == FDCAN_RECOVERY_RESTARTED) {
            s_next_tx_attempt_ms = now_ms;
        }
    } else {
        if (s_bus_off_active != 0U) {
            s_can_ready = s_can_config_valid;
            s_zero_pending = 1U;
            s_next_tx_attempt_ms = now_ms;
        }
        s_bus_off_active = 0U;
    }
}

void WheelDrive_Tick(uint32_t now_ms)
{
    service_bus(now_ms);
    const uint32_t elapsed_ms = (uint32_t)(now_ms - s_last_control_ms);
    if (elapsed_ms < WHEEL_CONTROL_PERIOD_MS) {
        return;
    }
    s_last_control_ms = now_ms;
    const float dt_s = clamp_float((float)elapsed_ms, 1.0f, 10.0f) * 0.001f;

    if (s_reset_pending != 0U) {
        s_reset_pending = 0U;
        reset_controller();
    }
    if (s_can_ready == 0U) {
        return;
    }
    if (s_zero_pending != 0U) {
        s_zero_pending = 0U;
        if (send_zero() == 0U) {
            s_zero_pending = 1U;
        }
        return;
    }

    const uint8_t online = WheelDrive_AllOnline();
    if (online == 0U) {
        if (((uint32_t)(now_ms - s_init_ms) > WHEEL_FEEDBACK_TIMEOUT_MS) &&
            (s_timeout_latched == 0U)) {
            lock_drive(1U);
        }
        reset_controller();
        s_reset_pending = 0U;
        s_zero_pending = (send_zero() != 0U) ? 0U : 1U;
        return;
    }
    if (s_overtemp_mask != 0U) {
        if (s_locked == 0U) {
            lock_drive(0U);
        }
        reset_controller();
        s_reset_pending = 0U;
        s_zero_pending = (send_zero() != 0U) ? 0U : 1U;
        return;
    }
    if ((s_operating_mode != WHEEL_OPERATING_OFF) &&
        ((uint32_t)(now_ms - s_last_command_ms) > WHEEL_COMMAND_TIMEOUT_MS)) {
        s_command_timeout_count++;
        lock_drive(0U);
        reset_controller();
        s_reset_pending = 0U;
        s_zero_pending = (send_zero() != 0U) ? 0U : 1U;
        return;
    }
    if ((s_locked != 0U) || (s_can_ready == 0U) || (s_mode_enabled == 0U)) {
        return;
    }
    if (DogSafety_IsLatched() != 0U) {
        lock_drive(0U);
        reset_controller();
        s_reset_pending = 0U;
        s_zero_pending = (send_zero() != 0U) ? 0U : 1U;
        return;
    }

    const uint32_t state_generation = s_state_generation;
    float requested_target_rpm[WHEEL_MOTOR_COUNT] = {};
    float phase_scale[WHEEL_MOTOR_COUNT] = {};
    uint8_t brake_requested = 0U;
    uint8_t hybrid_mode = 0U;
    uint32_t motion_generation = 0U;
    snapshot_motion(requested_target_rpm, phase_scale, &brake_requested,
                    &hybrid_mode, &motion_generation);
    (void)hybrid_mode;
    if (brake_requested != s_brake_active) {
        memset(s_integral, 0, sizeof(s_integral));
        s_brake_active = brake_requested;
    }
    float measured_speed[WHEEL_MOTOR_COUNT] = {};
    uint8_t measured_temperature_c[WHEEL_MOTOR_COUNT] = {};
    if (feedback_lock() == 0U) {
        s_zero_pending = 1U;
        return;
    }
    for (uint8_t i = 0U; i < WHEEL_MOTOR_COUNT; ++i) {
        measured_speed[i] = s_feedback[i].output_speed_rad_s;
        measured_temperature_c[i] = s_feedback[i].temperature_c;
    }
    feedback_unlock();

    float desired_target[WHEEL_MOTOR_COUNT] = {};
    for (uint8_t i = 0U; i < WHEEL_MOTOR_COUNT; ++i) {
        desired_target[i] = clamp_float(requested_target_rpm[i],
                                        -WHEEL_MAX_OUTPUT_RPM,
                                        WHEEL_MAX_OUTPUT_RPM) *
                            clamp_float(phase_scale[i], 0.0f, 1.0f) *
                            WHEEL_RPM_TO_RAD_S;
    }

    uint8_t peak_limited_mask = 0U;
    uint8_t thermal_derated_mask = 0U;
    for (uint8_t i = 0U; i < WHEEL_MOTOR_COUNT; ++i) {
        float ramp_target = desired_target[i];
        float ramp_rate;
        if (brake_requested != 0U) {
            ramp_rate = WHEEL_BRAKE_RAD_S2;
        } else if ((s_ramped_target_rad_s[i] * desired_target[i] < 0.0f) &&
                   (fabsf(s_ramped_target_rad_s[i]) > WHEEL_MOTION_ZERO_EPSILON)) {
            ramp_target = 0.0f;
            ramp_rate = WHEEL_DECEL_RAD_S2;
        } else if (fabsf(desired_target[i]) > fabsf(s_ramped_target_rad_s[i])) {
            ramp_rate = WHEEL_ACCEL_RAD_S2;
        } else {
            ramp_rate = WHEEL_DECEL_RAD_S2;
        }
        const float ramp_step = ramp_rate * dt_s;
        if (s_ramped_target_rad_s[i] < ramp_target) {
            s_ramped_target_rad_s[i] =
                fminf(s_ramped_target_rad_s[i] + ramp_step, ramp_target);
        } else if (s_ramped_target_rad_s[i] > ramp_target) {
            s_ramped_target_rad_s[i] =
                fmaxf(s_ramped_target_rad_s[i] - ramp_step, ramp_target);
        }
        const float motor_target = s_ramped_target_rad_s[i] * s_forward_sign[i];
        const float error = motor_target - measured_speed[i];
        const float proportional = WHEEL_PI_KP * error;
        float integral = s_integral[i];
        if (!((brake_requested != 0U) &&
              (fabsf(s_ramped_target_rad_s[i]) <= WHEEL_MOTION_ZERO_EPSILON) &&
              (fabsf(measured_speed[i]) < WHEEL_STOP_SPEED_RAD_S))) {
            integral += WHEEL_PI_KI_PER_S * error * dt_s;
        }
        const float demand = proportional + integral;
        const uint8_t peak_demand = (fabsf(demand * WHEEL_CURRENT_RAW_SCALE) >
                                     (float)WHEEL_CONTINUOUS_CURRENT_RAW) ? 1U : 0U;
        if (peak_demand != 0U) {
            s_peak_budget_ms[i] = fminf(s_peak_budget_ms[i] + dt_s * 1000.0f,
                                        WHEEL_PEAK_BUDGET_MS);
        } else {
            s_peak_budget_ms[i] = fmaxf(s_peak_budget_ms[i] -
                                        dt_s * 1000.0f * WHEEL_PEAK_RECOVERY_SCALE,
                                        0.0f);
        }
        int32_t current_limit = thermal_peak_limit_raw(measured_temperature_c[i]);
        if (measured_temperature_c[i] > WHEEL_TEMP_DERATE_START_C) {
            thermal_derated_mask |= (uint8_t)(1U << i);
        }
        if (s_peak_budget_ms[i] >= WHEEL_PEAK_BUDGET_MS) {
            current_limit = WHEEL_CONTINUOUS_CURRENT_RAW;
            if (peak_demand != 0U) {
                peak_limited_mask |= (uint8_t)(1U << i);
            }
        }
        if (current_limit < WHEEL_CONTINUOUS_CURRENT_RAW) {
            current_limit = WHEEL_CONTINUOUS_CURRENT_RAW;
        }
        if (current_limit > WHEEL_PEAK_CURRENT_RAW) {
            current_limit = WHEEL_PEAK_CURRENT_RAW;
        }
        s_active_current_limit_raw[i] = (int16_t)current_limit;
        const float output_limit = (float)current_limit / WHEEL_CURRENT_RAW_SCALE;
        float output;
        if ((brake_requested != 0U) &&
            (fabsf(s_ramped_target_rad_s[i]) <= WHEEL_MOTION_ZERO_EPSILON) &&
            (fabsf(measured_speed[i]) < WHEEL_STOP_SPEED_RAD_S)) {
            s_integral[i] = 0.0f;
            output = clamp_float(proportional, -output_limit, output_limit);
        } else {
            const float unsaturated = demand;
            output = clamp_float(unsaturated, -output_limit, output_limit);
            if (output != unsaturated) {
                integral = output - proportional;
            }
            s_integral[i] = clamp_float(integral, -output_limit, output_limit);
        }
        int32_t raw = (int32_t)(output * WHEEL_CURRENT_RAW_SCALE);
        if (raw > current_limit) raw = current_limit;
        if (raw < -current_limit) raw = -current_limit;
        s_current_cmd[i] = (int16_t)raw;
    }
    s_peak_limited_mask = peak_limited_mask;
    s_thermal_derated_mask = thermal_derated_mask;
    if ((state_generation != s_state_generation) ||
        (s_mode_enabled == 0U) || (s_locked != 0U)) {
        s_reset_pending = 1U;
        s_zero_pending = 1U;
        return;
    }
    if (motion_generation != s_motion_generation) {
        return;
    }
    (void)send_currents(s_current_cmd);
}

void WheelDrive_GetDiag(WheelDriveDiag *diag)
{
    if (diag == nullptr) {
        return;
    }
    memset(diag, 0, sizeof(*diag));
    diag->can_bus = wheel_can_bus_number(s_can);
    diag->can_ready = s_can_ready;
    diag->can_config_valid = s_can_config_valid;
    diag->mode_enabled = s_mode_enabled;
    diag->locked = s_locked;
    diag->all_online = WheelDrive_AllOnline();
    diag->stopped = WheelDrive_IsStopped();
    diag->profile = (uint8_t)s_profile;
    diag->operating_mode = (uint8_t)s_operating_mode;
    diag->brake_active = s_brake_active;
    diag->peak_limited_mask = s_peak_limited_mask;
    diag->thermal_derated_mask = s_thermal_derated_mask;
    diag->overtemp_mask = s_overtemp_mask;
    diag->requested_left_rpm = s_requested_left_rpm;
    diag->requested_right_rpm = s_requested_right_rpm;
    diag->hybrid_mode = s_hybrid_mode;
    diag->tx_fail_count = s_tx_fail_count;
    diag->bus_off_count = s_bus_off_count;
    diag->feedback_timeout_count = s_feedback_timeout_count;
    diag->command_timeout_count = s_command_timeout_count;
    diag->rx_reject_count = s_rx_reject_count;
    diag->nominal_baud = WHEEL_CAN_NOMINAL_BAUD;
    if (feedback_lock() != 0U) {
        diag->feedback_seen_mask = s_feedback_seen_mask;
        memcpy(diag->motor, s_feedback, sizeof(s_feedback));
        feedback_unlock();
    }
    for (uint8_t i = 0U; i < WHEEL_MOTOR_COUNT; ++i) {
        diag->requested_target_rpm[i] = s_requested_target_rpm[i];
        diag->phase_scale[i] = s_phase_scale[i];
        diag->final_target_rpm[i] = s_requested_target_rpm[i] * s_phase_scale[i];
        diag->ramped_target_rpm[i] = s_ramped_target_rad_s[i] * WHEEL_RAD_S_TO_RPM;
        diag->vehicle_speed_rpm[i] = diag->motor[i].output_speed_rad_s *
                                     s_forward_sign[i] * WHEEL_RAD_S_TO_RPM;
        diag->current_cmd[i] = s_current_cmd[i];
        diag->current_limit_raw[i] = s_active_current_limit_raw[i];
        diag->peak_budget_ms[i] = (uint16_t)clamp_float(s_peak_budget_ms[i],
                                                       0.0f,
                                                       WHEEL_PEAK_BUDGET_MS);
    }
}
