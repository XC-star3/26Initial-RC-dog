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
#define WHEEL_ENCODER_COUNTS             8192
#define WHEEL_ENCODER_HALF_COUNTS        (WHEEL_ENCODER_COUNTS / 2)
#define WHEEL_GEAR_RATIO                 19.0f
#define WHEEL_TWO_PI                     6.28318530717958647692f
#define WHEEL_CONTROL_PERIOD_MS          2U
#define WHEEL_FEEDBACK_TIMEOUT_MS         100U
#define WHEEL_LOCK_CLEAR_STABLE_MS       200U
#define WHEEL_BUS_CHECK_PERIOD_MS        100U
#define WHEEL_STOP_SPEED_RAD_S           0.5f
#define WHEEL_NORMAL_SPEED_RAD_S         3.0f
#define WHEEL_NORMAL_RAMP_RAD_S2         6.0f
#define WHEEL_CRAWL_SPEED_RAD_S          2.0f
#define WHEEL_CRAWL_RAMP_RAD_S2          4.0f
#define WHEEL_PI_KP                      0.015f
#define WHEEL_PI_KI_PER_S                0.25f
#define WHEEL_PI_NORMALIZED_LIMIT        0.3f
#define WHEEL_CURRENT_RAW_SCALE          10000.0f
#define WHEEL_NORMAL_CURRENT_RAW_LIMIT   3000
#define WHEEL_CRAWL_CURRENT_RAW_LIMIT    2000

static const float s_forward_sign[WHEEL_MOTOR_COUNT] = {1.0f, -1.0f, -1.0f, 1.0f};

static FDCAN_HandleTypeDef *s_can = nullptr;
static WheelMotorFeedback s_feedback[WHEEL_MOTOR_COUNT];
static StaticSemaphore_t s_feedback_mutex_storage;
static SemaphoreHandle_t s_feedback_mutex = nullptr;
static float s_integral[WHEEL_MOTOR_COUNT];
static int16_t s_current_cmd[WHEEL_MOTOR_COUNT];
static volatile uint8_t s_mode_enabled = 0U;
static volatile uint8_t s_locked = 0U;
static volatile uint8_t s_can_ready = 0U;
static volatile uint8_t s_reset_pending = 0U;
static volatile uint8_t s_zero_pending = 0U;
static volatile uint32_t s_state_generation = 0U;
static volatile WheelDriveCommand s_command = WHEEL_DRIVE_STOP;
static volatile WheelDriveProfile s_profile = WHEEL_PROFILE_NORMAL;
static uint8_t s_feedback_seen_mask = 0U;
static uint8_t s_timeout_latched = 0U;
static uint8_t s_bus_off_active = 0U;
static float s_ramped_target_rad_s = 0.0f;
static uint32_t s_init_ms = 0U;
static uint32_t s_last_control_ms = 0U;
static uint32_t s_last_bus_check_ms = 0U;
static volatile uint32_t s_stop_stable_since_ms = 0U;
static uint32_t s_tx_fail_count = 0U;
static uint32_t s_bus_off_count = 0U;
static uint32_t s_feedback_timeout_count = 0U;
static uint32_t s_rx_reject_count = 0U;

static uint8_t feedback_lock(void)
{
    if (s_feedback_mutex == nullptr) {
        return 0U;
    }
    return (xSemaphoreTake(s_feedback_mutex, portMAX_DELAY) == pdTRUE) ? 1U : 0U;
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
    s_ramped_target_rad_s = 0.0f;
}

static uint8_t send_currents(const int16_t current[WHEEL_MOTOR_COUNT])
{
    if ((s_can == nullptr) || (s_can_ready == 0U)) {
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
        return 0U;
    }
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
    s_command = WHEEL_DRIVE_STOP;
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
    reset_controller();
    s_mode_enabled = 0U;
    s_reset_pending = 0U;
    s_zero_pending = 0U;
    s_state_generation = 0U;
    s_command = WHEEL_DRIVE_STOP;
    s_feedback_seen_mask = 0U;
    s_timeout_latched = 0U;
    s_bus_off_active = 0U;
    s_init_ms = HAL_GetTick();
    s_last_control_ms = s_init_ms;
    s_last_bus_check_ms = s_init_ms;
    s_stop_stable_since_ms = 0U;
    s_tx_fail_count = 0U;
    s_bus_off_count = 0U;
    s_feedback_timeout_count = 0U;
    s_rx_reject_count = 0U;
    s_can_ready = ((hfdcan != nullptr) && (hfdcan->Instance == FDCAN3) &&
                   system_can[2] && (s_feedback_mutex != nullptr)) ? 1U : 0U;
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
    feedback->last_update_ms = HAL_GetTick();
    s_feedback_seen_mask |= (uint8_t)(1U << index);
    feedback_unlock();
    return 1U;
}

void WheelDrive_SetCommand(WheelDriveCommand command)
{
    if ((command < WHEEL_DRIVE_STOP) || (command > WHEEL_DRIVE_REVERSE)) {
        command = WHEEL_DRIVE_STOP;
    }
    s_command = command;
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

void WheelDrive_Enable(void)
{
    if ((s_can_ready != 0U) && (s_locked == 0U)) {
        s_mode_enabled = 1U;
    }
}

void WheelDrive_Disable(void)
{
    const uint8_t was_enabled = s_mode_enabled;
    s_mode_enabled = 0U;
    s_command = WHEEL_DRIVE_STOP;
    if (was_enabled != 0U) {
        s_state_generation++;
        s_reset_pending = 1U;
        s_zero_pending = 1U;
    }
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
    if ((s_can_ready == 0U) || (WheelDrive_IsStopped() == 0U)) {
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
            lock_drive(0U);
        }
        if (fdcan_recover_bus_off(s_can) == FDCAN_RECOVERY_RESTARTED) {
            s_can_ready = 1U;
            s_zero_pending = 1U;
        }
    } else {
        s_bus_off_active = 0U;
    }
}

void WheelDrive_Tick(uint32_t now_ms)
{
    service_bus(now_ms);
    if ((uint32_t)(now_ms - s_last_control_ms) < WHEEL_CONTROL_PERIOD_MS) {
        return;
    }
    s_last_control_ms = now_ms;

    if (s_reset_pending != 0U) {
        s_reset_pending = 0U;
        reset_controller();
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
    if ((s_locked != 0U) || (s_can_ready == 0U) || (s_mode_enabled == 0U)) {
        return;
    }
    const WheelDriveProfile profile = s_profile;
    const uint8_t mechanical_ready = DogStand_IsMechanicalLimitIdleReady();
    if ((DogSafety_IsLatched() != 0U) ||
        ((profile == WHEEL_PROFILE_MECHANICAL_CRAWL) && (mechanical_ready == 0U)) ||
        ((profile != WHEEL_PROFILE_MECHANICAL_CRAWL) && (DogStand_IsDisabled() != 0U))) {
        lock_drive(0U);
        reset_controller();
        s_reset_pending = 0U;
        s_zero_pending = (send_zero() != 0U) ? 0U : 1U;
        return;
    }

    const uint32_t state_generation = s_state_generation;
    float measured_speed[WHEEL_MOTOR_COUNT] = {};
    if (feedback_lock() == 0U) {
        s_zero_pending = 1U;
        return;
    }
    for (uint8_t i = 0U; i < WHEEL_MOTOR_COUNT; ++i) {
        measured_speed[i] = s_feedback[i].output_speed_rad_s;
    }
    feedback_unlock();

    float desired_target = 0.0f;
    if (s_command == WHEEL_DRIVE_FORWARD) {
        desired_target = (profile == WHEEL_PROFILE_MECHANICAL_CRAWL) ?
                         WHEEL_CRAWL_SPEED_RAD_S : WHEEL_NORMAL_SPEED_RAD_S;
    } else if (s_command == WHEEL_DRIVE_REVERSE) {
        desired_target = (profile == WHEEL_PROFILE_MECHANICAL_CRAWL) ?
                         -WHEEL_CRAWL_SPEED_RAD_S : -WHEEL_NORMAL_SPEED_RAD_S;
    }

    const float ramp_rate = (profile == WHEEL_PROFILE_MECHANICAL_CRAWL) ?
                            WHEEL_CRAWL_RAMP_RAD_S2 : WHEEL_NORMAL_RAMP_RAD_S2;
    const float ramp_step = ramp_rate *
                            ((float)WHEEL_CONTROL_PERIOD_MS * 0.001f);
    if (s_ramped_target_rad_s < desired_target) {
        s_ramped_target_rad_s = fminf(s_ramped_target_rad_s + ramp_step, desired_target);
    } else if (s_ramped_target_rad_s > desired_target) {
        s_ramped_target_rad_s = fmaxf(s_ramped_target_rad_s - ramp_step, desired_target);
    }

    for (uint8_t i = 0U; i < WHEEL_MOTOR_COUNT; ++i) {
        const float motor_target = s_ramped_target_rad_s * s_forward_sign[i];
        const float error = motor_target - measured_speed[i];
        const float proportional = WHEEL_PI_KP * error;
        float integral = s_integral[i] + WHEEL_PI_KI_PER_S * error *
                         ((float)WHEEL_CONTROL_PERIOD_MS * 0.001f);
        const float unsaturated = proportional + integral;
        const float output = clamp_float(unsaturated,
                                         -WHEEL_PI_NORMALIZED_LIMIT,
                                         WHEEL_PI_NORMALIZED_LIMIT);
        if (output != unsaturated) {
            integral = output - proportional;
        }
        s_integral[i] = clamp_float(integral,
                                    -WHEEL_PI_NORMALIZED_LIMIT,
                                    WHEEL_PI_NORMALIZED_LIMIT);
        int32_t raw = (int32_t)(output * WHEEL_CURRENT_RAW_SCALE);
        const int32_t current_limit = (profile == WHEEL_PROFILE_MECHANICAL_CRAWL) ?
                                      WHEEL_CRAWL_CURRENT_RAW_LIMIT :
                                      WHEEL_NORMAL_CURRENT_RAW_LIMIT;
        if (raw > current_limit) raw = current_limit;
        if (raw < -current_limit) raw = -current_limit;
        s_current_cmd[i] = (int16_t)raw;
    }
    if ((state_generation != s_state_generation) ||
        (s_mode_enabled == 0U) || (s_locked != 0U)) {
        s_reset_pending = 1U;
        s_zero_pending = 1U;
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
    diag->can_ready = s_can_ready;
    diag->mode_enabled = s_mode_enabled;
    diag->locked = s_locked;
    diag->all_online = WheelDrive_AllOnline();
    diag->stopped = WheelDrive_IsStopped();
    diag->command = (uint8_t)s_command;
    diag->profile = (uint8_t)s_profile;
    diag->ramped_target_rad_s = s_ramped_target_rad_s;
    diag->tx_fail_count = s_tx_fail_count;
    diag->bus_off_count = s_bus_off_count;
    diag->feedback_timeout_count = s_feedback_timeout_count;
    diag->rx_reject_count = s_rx_reject_count;
    if (feedback_lock() != 0U) {
        diag->feedback_seen_mask = s_feedback_seen_mask;
        memcpy(diag->motor, s_feedback, sizeof(s_feedback));
        feedback_unlock();
    }
}
