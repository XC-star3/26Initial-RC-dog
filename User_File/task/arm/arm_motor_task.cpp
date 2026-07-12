#include "arm_motor_task.h"

#include "motor_DM.h"
#include "motor_LZ.h"

#include <math.h>

static constexpr fp32 kPi = 3.14159265358979323846f;
static constexpr fp32 kDegToRad = kPi / 180.0f;
static constexpr fp32 kRadToDeg = 180.0f / kPi;
static constexpr fp32 kRadSToRpm = 60.0f / (2.0f * kPi);
static constexpr fp32 kControlNominalDtSec = 0.005f;
static constexpr fp32 kControlMaxDtSec = 0.050f;
static constexpr uint32_t kFeedbackTimeoutMs = 100U;
static constexpr uint32_t kPresentTimeoutMs = 1000U;
static constexpr uint32_t kDiagnosticPeriodMs = 300U;
static constexpr uint8_t kEl05DeviceIdResponseTarget = 0xFEU;

struct J0PidState {
    ArmMotorPidConfig config;
    fp32 integral;
    fp32 last_error;
};

struct J0Controller {
    fp32 target_angle_deg;
    fp32 speed_ff_rpm;
    fp32 torque_ff_nm;
    fp32 limit_min_deg;
    fp32 limit_max_deg;
    fp32 offset_deg;
    uint8_t invert;
    uint8_t enabled;
    uint8_t first_set_angle;
    uint8_t gravity_enable;
    fp32 gravity_max_torque_nm;
    fp32 gravity_horizontal_deg;
    fp32 last_torque_cmd_nm;
    J0PidState angle_pid;
    J0PidState speed_pid;
};

struct J1Controller {
    fp32 target_angle_deg;
    fp32 target_vel_rad_s;
    fp32 target_torque_nm;
    fp32 limit_min_deg;
    fp32 limit_max_deg;
    fp32 offset_deg;
    uint8_t invert;
    uint8_t enabled;
    uint8_t first_set_angle;
    uint8_t master_id;
};

static Class_Motor_DM s_j0_dm4310;
static Class_Motor_LZ s_j1_lz;
static FDCAN_HandleTypeDef *s_j0_can = nullptr;
static FDCAN_HandleTypeDef *s_j1_can = nullptr;
static uint16_t s_j0_dm_id = 0U;
static uint16_t s_j0_dm_feedback_id = 0U;
static uint16_t s_j1_lz_id = 0U;
static uint8_t s_initialized = 0U;
static uint32_t s_last_send_ms = 0U;
static uint32_t s_last_status_feedback_ms[ARM_JOINT_COUNT] = {};
static uint32_t s_last_present_ms[ARM_JOINT_COUNT] = {};
static uint32_t s_last_diagnostic_ms[ARM_JOINT_COUNT] = {};
static uint32_t s_enable_request_ms[ARM_JOINT_COUNT] = {};
static uint8_t s_status_feedback_seen[ARM_JOINT_COUNT] = {};
static uint8_t s_present_seen[ARM_JOINT_COUNT] = {};
static uint8_t s_diagnostic_sent[ARM_JOINT_COUNT] = {};
static ArmMotorFeedback s_feedback[ARM_JOINT_COUNT] = {};
static J0Controller s_j0_ctrl = {
    0.0f,
    0.0f,
    0.0f,
    -60.0f,
    90.0f,
    0.0f,
    0U,
    0U,
    1U,
    0U,
    1.05f,
    -40.36f,
    0.0f,
    {{1.0f, 0.0f, 0.0f, 50.0f, 100.0f}, 0.0f, 0.0f},
    {{0.1f, 0.0f, 0.0f, 2.0f, 2.0f}, 0.0f, 0.0f},
};
static J1Controller s_j1_ctrl = {
    0.0f,
    0.0f,
    0.0f,
    -720.0f,
    720.0f,
    0.0f,
    0U,
    0U,
    1U,
    MOTOR_LZ_DEFAULT_MASTER_ID,
};

static fp32 clamp_fp32(fp32 value, fp32 min_value, fp32 max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void refresh_feedback_age(uint32_t now)
{
    for (uint8_t joint = 0U; joint < ARM_JOINT_COUNT; ++joint) {
        if (s_status_feedback_seen[joint] != 0U) {
            s_feedback[joint].feedback_age_ms = now - s_last_status_feedback_ms[joint];
        } else {
            s_feedback[joint].feedback_age_ms = ARM_MOTOR_FEEDBACK_AGE_INVALID;
        }

        if ((s_present_seen[joint] == 0U) ||
            ((uint32_t)(now - s_last_present_ms[joint]) > kPresentTimeoutMs)) {
            s_feedback[joint].present = 0U;
        }

        if ((s_feedback[joint].online != 0U) &&
            ((s_feedback[joint].enabled == 0U) ||
             (s_status_feedback_seen[joint] == 0U) ||
             ((uint32_t)(now - s_last_status_feedback_ms[joint]) > kFeedbackTimeoutMs))) {
            s_feedback[joint].online = 0U;
        }
    }
}

static void mark_present(uint8_t joint, uint32_t now)
{
    s_present_seen[joint] = 1U;
    s_last_present_ms[joint] = now;
    s_feedback[joint].present = 1U;
}

static void mark_status_feedback(uint8_t joint, uint32_t now)
{
    mark_present(joint, now);
    s_status_feedback_seen[joint] = 1U;
    s_last_status_feedback_ms[joint] = now;
    s_feedback[joint].feedback_age_ms = 0U;
}

static void pid_clear(J0PidState *pid)
{
    if (pid == nullptr) {
        return;
    }
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

static fp32 pid_calculate(J0PidState *pid, fp32 error, fp32 dt_s)
{
    if (pid == nullptr) {
        return 0.0f;
    }

    const fp32 p_out = pid->config.kp * error;
    const fp32 d_out = pid->config.kd * (error - pid->last_error) / dt_s;
    fp32 out = p_out + pid->integral + d_out;

    fp32 delta_i = pid->config.ki * error * dt_s;
    if ((out >= pid->config.max_out && delta_i > 0.0f) ||
        (out <= -pid->config.max_out && delta_i < 0.0f)) {
        delta_i = 0.0f;
    }
    pid->integral = clamp_fp32(pid->integral + delta_i, -pid->config.max_i, pid->config.max_i);

    out = p_out + pid->integral + d_out;
    pid->last_error = error;
    return clamp_fp32(out, -pid->config.max_out, pid->config.max_out);
}

static void j0_clear_pid(void)
{
    pid_clear(&s_j0_ctrl.angle_pid);
    pid_clear(&s_j0_ctrl.speed_pid);
}

static fp32 j0_correct_angle_deg(fp32 raw_angle_deg)
{
    const fp32 corrected = raw_angle_deg - s_j0_ctrl.offset_deg;
    return (s_j0_ctrl.invert != 0U) ? -corrected : corrected;
}

static fp32 j0_correct_signed(fp32 raw_value)
{
    return (s_j0_ctrl.invert != 0U) ? -raw_value : raw_value;
}

static fp32 j1_motor_to_joint_angle_deg(fp32 raw_angle_deg)
{
    const fp32 corrected = raw_angle_deg - s_j1_ctrl.offset_deg;
    return (s_j1_ctrl.invert != 0U) ? -corrected : corrected;
}

static fp32 j1_joint_to_motor_pos_rad(fp32 joint_pos_rad)
{
    const fp32 offset_rad = s_j1_ctrl.offset_deg * kDegToRad;
    return (s_j1_ctrl.invert != 0U) ? (offset_rad - joint_pos_rad) : (joint_pos_rad + offset_rad);
}

static fp32 j1_joint_to_motor_signed(fp32 joint_value)
{
    return (s_j1_ctrl.invert != 0U) ? -joint_value : joint_value;
}

static void j1_apply_target_to_motor(void)
{
    const fp32 target_deg = clamp_fp32(s_j1_ctrl.target_angle_deg,
                                       s_j1_ctrl.limit_min_deg,
                                       s_j1_ctrl.limit_max_deg);
    const fp32 joint_pos_rad = target_deg * kDegToRad;
    s_j1_ctrl.target_angle_deg = target_deg;
    s_j1_lz.Set_Control_Mode(Motor_LZ_Pos_control);
    s_j1_lz.Set_Pos(j1_joint_to_motor_pos_rad(joint_pos_rad));
    s_j1_lz.Set_W(j1_joint_to_motor_signed(s_j1_ctrl.target_vel_rad_s));
    s_j1_lz.Set_T(j1_joint_to_motor_signed(s_j1_ctrl.target_torque_nm));
}

static void j1_lock_target_to_measure(void)
{
    if (s_feedback[ARM_J1_LZ].online != 0U) {
        s_j1_ctrl.target_angle_deg = s_feedback[ARM_J1_LZ].angle_deg;
    }
    s_j1_ctrl.target_vel_rad_s = 0.0f;
    s_j1_ctrl.target_torque_nm = 0.0f;
    j1_apply_target_to_motor();
}

static void j0_lock_target_to_measure(void)
{
    if (s_feedback[ARM_J0_DM4310].online != 0U) {
        s_j0_ctrl.target_angle_deg = s_feedback[ARM_J0_DM4310].angle_deg;
    }
    s_j0_ctrl.speed_ff_rpm = 0.0f;
    s_j0_ctrl.torque_ff_nm = 0.0f;
    s_j0_ctrl.last_torque_cmd_nm = 0.0f;
    j0_clear_pid();
}

static void j0_disable(void)
{
    const uint8_t was_enabled = s_j0_ctrl.enabled;
    s_j0_ctrl.enabled = 0U;
    s_feedback[ARM_J0_DM4310].enabled = 0U;
    s_feedback[ARM_J0_DM4310].online = 0U;
    s_enable_request_ms[ARM_J0_DM4310] = 0U;
    j0_lock_target_to_measure();
    if (was_enabled != 0U) {
        s_j0_dm4310.lose();
    }
}

static void j1_disable(void)
{
    const uint8_t was_enabled = s_j1_ctrl.enabled;
    s_j1_ctrl.enabled = 0U;
    s_feedback[ARM_J1_LZ].enabled = 0U;
    s_feedback[ARM_J1_LZ].online = 0U;
    s_enable_request_ms[ARM_J1_LZ] = 0U;
    j1_lock_target_to_measure();
    if (was_enabled != 0U) {
        s_j1_lz.lose();
    }
}

static void copy_dm_feedback(void)
{
    const fp32 angle_deg = j0_correct_angle_deg(s_j0_dm4310.Get_Now_Angle());
    const fp32 speed_rad_s = j0_correct_signed(s_j0_dm4310.Get_Now_W());
    const fp32 torque_nm = j0_correct_signed(s_j0_dm4310.Get_Now_T());

    s_feedback[ARM_J0_DM4310].online = s_feedback[ARM_J0_DM4310].enabled;
    s_feedback[ARM_J0_DM4310].error = s_j0_dm4310.Get_MError();
    s_feedback[ARM_J0_DM4310].fault = s_j0_dm4310.Get_MError();
    s_feedback[ARM_J0_DM4310].fault_valid = 1U;
    s_feedback[ARM_J0_DM4310].mode = s_j0_dm4310.Get_mode();
    s_feedback[ARM_J0_DM4310].pos_rad = angle_deg * kDegToRad;
    s_feedback[ARM_J0_DM4310].angle_deg = angle_deg;
    s_feedback[ARM_J0_DM4310].vel_rad_s = speed_rad_s;
    s_feedback[ARM_J0_DM4310].torque_nm = torque_nm;
    s_feedback[ARM_J0_DM4310].temperature_c = s_j0_dm4310.Get_motor_Temperature();
}

static void copy_lz_feedback(void)
{
    const fp32 angle_deg = j1_motor_to_joint_angle_deg(s_j1_lz.Get_Now_Angle());
    const fp32 speed_rad_s = j1_joint_to_motor_signed(s_j1_lz.Get_Now_W());
    const fp32 torque_nm = j1_joint_to_motor_signed(s_j1_lz.Get_Now_T());

    s_feedback[ARM_J1_LZ].online = s_feedback[ARM_J1_LZ].enabled;
    s_feedback[ARM_J1_LZ].error = s_j1_lz.Get_MError();
    s_feedback[ARM_J1_LZ].fault = s_j1_lz.Get_MError();
    s_feedback[ARM_J1_LZ].fault_valid = 1U;
    s_feedback[ARM_J1_LZ].mode = s_j1_lz.Get_mode();
    s_feedback[ARM_J1_LZ].pos_rad = angle_deg * kDegToRad;
    s_feedback[ARM_J1_LZ].angle_deg = angle_deg;
    s_feedback[ARM_J1_LZ].vel_rad_s = speed_rad_s;
    s_feedback[ARM_J1_LZ].torque_nm = torque_nm;
    s_feedback[ARM_J1_LZ].temperature_c = s_j1_lz.Get_Now_Temperature();
}

void ArmMotor_Init(FDCAN_HandleTypeDef *j0_dm_can,
                   uint16_t j0_dm_can_id,
                   uint16_t j0_dm_feedback_id,
                   FDCAN_HandleTypeDef *j1_lz_can,
                   uint16_t j1_lz_can_id,
                   uint8_t j1_lz_model)
{
    (void)j1_lz_model;

    s_j0_can = j0_dm_can;
    s_j1_can = j1_lz_can;
    s_j0_dm_id = j0_dm_can_id;
    s_j0_dm_feedback_id = j0_dm_feedback_id;
    s_j1_lz_id = j1_lz_can_id;

    s_j0_dm4310.Init(j0_dm_can, j0_dm_can_id, MOTOR_DM_J4310, Motor_DM_Pos_control);
    s_j1_lz.Init(j1_lz_can, j1_lz_can_id, MOTOR_LZ_EL05, Motor_LZ_Pos_control);
    s_j1_lz.set_master_id(s_j1_ctrl.master_id);
    s_j0_ctrl.enabled = 0U;
    s_j0_ctrl.first_set_angle = 1U;
    s_j1_ctrl.enabled = 0U;
    s_j1_ctrl.first_set_angle = 1U;
    j0_lock_target_to_measure();
    j1_lock_target_to_measure();
    s_last_send_ms = 0U;
    for (uint8_t joint = 0U; joint < ARM_JOINT_COUNT; ++joint) {
        s_feedback[joint] = {};
        s_feedback[joint].feedback_age_ms = ARM_MOTOR_FEEDBACK_AGE_INVALID;
        s_last_status_feedback_ms[joint] = 0U;
        s_last_present_ms[joint] = 0U;
        s_last_diagnostic_ms[joint] = 0U;
        s_enable_request_ms[joint] = 0U;
        s_status_feedback_seen[joint] = 0U;
        s_present_seen[joint] = 0U;
        s_diagnostic_sent[joint] = 0U;
    }
    s_initialized = 1U;
}

void ArmMotor_Enable(void)
{
    if (s_initialized == 0U) {
        return;
    }
    const uint32_t now = HAL_GetTick();
    if (s_j0_ctrl.enabled == 0U) {
        s_j0_ctrl.enabled = 1U;
        s_feedback[ARM_J0_DM4310].enabled = 1U;
        s_feedback[ARM_J0_DM4310].online = 0U;
        s_j0_ctrl.first_set_angle = 1U;
        s_enable_request_ms[ARM_J0_DM4310] = now;
        j0_lock_target_to_measure();
        s_j0_dm4310.enable();
    }
    if (s_j1_ctrl.enabled == 0U) {
        s_j1_ctrl.enabled = 1U;
        s_feedback[ARM_J1_LZ].enabled = 1U;
        s_feedback[ARM_J1_LZ].online = 0U;
        s_j1_ctrl.first_set_angle = 1U;
        s_enable_request_ms[ARM_J1_LZ] = now;
        j1_lock_target_to_measure();
        s_j1_lz.set_run_mode(MOTOR_LZ_EL05_RUN_MODE_MIT);
        s_j1_lz.enable();
        s_j1_lz.active_recv(1U);
    }
}

void ArmMotor_Disable(void)
{
    if (s_initialized == 0U) {
        return;
    }
    j0_disable();
    j1_disable();
}

void ArmMotor_Zero(uint8_t joint)
{
    if (s_initialized == 0U) {
        return;
    }
    if (joint == ARM_J0_DM4310) {
        s_j0_dm4310.zero();
    } else if (joint == ARM_J1_LZ) {
        s_j1_lz.zero();
    }
}

void ArmMotor_SetTargetRad(uint8_t joint, fp32 pos_rad, fp32 vel_rad_s, fp32 torque_nm)
{
    if (joint == ARM_J0_DM4310) {
        if ((s_j0_ctrl.enabled == 0U) || (s_feedback[ARM_J0_DM4310].online == 0U)) {
            j0_lock_target_to_measure();
            return;
        }

        if (s_j0_ctrl.first_set_angle != 0U) {
            j0_lock_target_to_measure();
            s_j0_ctrl.first_set_angle = 0U;
            return;
        }

        const fp32 min_deg = s_j0_ctrl.limit_min_deg;
        const fp32 max_deg = s_j0_ctrl.limit_max_deg;
        const fp32 target_deg = pos_rad * kRadToDeg;
        const fp32 clamped_deg = clamp_fp32(target_deg, min_deg, max_deg);
        s_j0_ctrl.target_angle_deg = clamped_deg;
        s_j0_ctrl.speed_ff_rpm = (target_deg == clamped_deg) ? (vel_rad_s * kRadSToRpm) : 0.0f;
        s_j0_ctrl.torque_ff_nm = torque_nm;
    } else if (joint == ARM_J1_LZ) {
        if ((s_j1_ctrl.enabled == 0U) || (s_feedback[ARM_J1_LZ].online == 0U)) {
            j1_lock_target_to_measure();
            return;
        }

        if (s_j1_ctrl.first_set_angle != 0U) {
            j1_lock_target_to_measure();
            s_j1_ctrl.first_set_angle = 0U;
            return;
        }

        s_j1_ctrl.target_angle_deg = pos_rad * kRadToDeg;
        s_j1_ctrl.target_vel_rad_s = vel_rad_s;
        s_j1_ctrl.target_torque_nm = torque_nm;
        j1_apply_target_to_motor();
    }
}

void ArmMotor_SetTargetDeg(uint8_t joint, fp32 angle_deg, fp32 vel_rad_s, fp32 torque_nm)
{
    ArmMotor_SetTargetRad(joint, angle_deg * kDegToRad, vel_rad_s, torque_nm);
}

void ArmMotor_SetGains(uint8_t joint, fp32 kp, fp32 kd)
{
    if (joint == ARM_J0_DM4310) {
        s_j0_ctrl.angle_pid.config.kp = kp;
        s_j0_ctrl.angle_pid.config.kd = kd;
        j0_clear_pid();
    } else if (joint == ARM_J1_LZ) {
        s_j1_lz.Set_Kp(kp);
        s_j1_lz.Set_Kd(kd);
    }
}

void ArmMotor_SetJ0LimitsDeg(fp32 min_deg, fp32 max_deg)
{
    if (min_deg <= max_deg) {
        s_j0_ctrl.limit_min_deg = min_deg;
        s_j0_ctrl.limit_max_deg = max_deg;
    }
}

void ArmMotor_SetJ0OffsetDeg(fp32 offset_deg)
{
    s_j0_ctrl.offset_deg = offset_deg;
    if (s_feedback[ARM_J0_DM4310].online != 0U) {
        copy_dm_feedback();
    }
}

void ArmMotor_SetJ0Invert(uint8_t enable)
{
    s_j0_ctrl.invert = (enable != 0U) ? 1U : 0U;
    if (s_feedback[ARM_J0_DM4310].online != 0U) {
        copy_dm_feedback();
    }
}

void ArmMotor_SetJ0Pid(const ArmMotorPidConfig *angle_pid, const ArmMotorPidConfig *speed_pid)
{
    if (angle_pid != nullptr) {
        s_j0_ctrl.angle_pid.config = *angle_pid;
    }
    if (speed_pid != nullptr) {
        s_j0_ctrl.speed_pid.config = *speed_pid;
    }
    j0_clear_pid();
}

void ArmMotor_SetJ0GravityComp(uint8_t enable, fp32 max_torque_nm, fp32 horizontal_deg)
{
    s_j0_ctrl.gravity_enable = (enable != 0U) ? 1U : 0U;
    s_j0_ctrl.gravity_max_torque_nm = max_torque_nm;
    s_j0_ctrl.gravity_horizontal_deg = horizontal_deg;
}

void ArmMotor_SetJ1LimitsDeg(fp32 min_deg, fp32 max_deg)
{
    if (min_deg <= max_deg) {
        s_j1_ctrl.limit_min_deg = min_deg;
        s_j1_ctrl.limit_max_deg = max_deg;
    }
}

void ArmMotor_SetJ1OffsetDeg(fp32 offset_deg)
{
    s_j1_ctrl.offset_deg = offset_deg;
    if (s_feedback[ARM_J1_LZ].online != 0U) {
        copy_lz_feedback();
    }
}

void ArmMotor_SetJ1Invert(uint8_t enable)
{
    s_j1_ctrl.invert = (enable != 0U) ? 1U : 0U;
    if (s_feedback[ARM_J1_LZ].online != 0U) {
        copy_lz_feedback();
    }
}

void ArmMotor_SetJ1MasterId(uint8_t master_id)
{
    s_j1_ctrl.master_id = master_id;
    s_j1_lz.set_master_id(master_id);
}

static fp32 j0_calculate_torque(fp32 dt_s)
{
    if ((s_j0_ctrl.enabled == 0U) || (s_feedback[ARM_J0_DM4310].online == 0U)) {
        j0_lock_target_to_measure();
        return 0.0f;
    }

    if (s_j0_ctrl.first_set_angle != 0U) {
        j0_lock_target_to_measure();
        s_j0_ctrl.first_set_angle = 0U;
        return 0.0f;
    }

    s_j0_ctrl.target_angle_deg = clamp_fp32(s_j0_ctrl.target_angle_deg,
                                            s_j0_ctrl.limit_min_deg,
                                            s_j0_ctrl.limit_max_deg);

    const fp32 current_angle_deg = s_feedback[ARM_J0_DM4310].angle_deg;
    const fp32 current_speed_rpm = s_feedback[ARM_J0_DM4310].vel_rad_s * kRadSToRpm;
    const fp32 angle_error_deg = s_j0_ctrl.target_angle_deg - current_angle_deg;
    const fp32 speed_outer_rpm = pid_calculate(&s_j0_ctrl.angle_pid, angle_error_deg, dt_s);
    const fp32 speed_error_rpm = speed_outer_rpm + s_j0_ctrl.speed_ff_rpm - current_speed_rpm;
    fp32 torque_nm = pid_calculate(&s_j0_ctrl.speed_pid, speed_error_rpm, dt_s) + s_j0_ctrl.torque_ff_nm;

    if (s_j0_ctrl.gravity_enable != 0U) {
        const fp32 angle_diff_rad = (current_angle_deg - s_j0_ctrl.gravity_horizontal_deg) * kDegToRad;
        torque_nm += s_j0_ctrl.gravity_max_torque_nm * cosf(angle_diff_rad);
    }

    torque_nm = clamp_fp32(torque_nm, -s_j0_dm4310.Get_T_Max(), s_j0_dm4310.Get_T_Max());
    s_j0_ctrl.last_torque_cmd_nm = torque_nm;
    return torque_nm;
}

void ArmMotor_Send(void)
{
    if (s_initialized == 0U) {
        return;
    }

    const uint32_t now = HAL_GetTick();
    refresh_feedback_age(now);
    if ((s_j0_ctrl.enabled != 0U) && (s_feedback[ARM_J0_DM4310].online == 0U) &&
        (s_enable_request_ms[ARM_J0_DM4310] != 0U) &&
        ((uint32_t)(now - s_enable_request_ms[ARM_J0_DM4310]) > kFeedbackTimeoutMs)) {
        j0_disable();
    }
    if ((s_j1_ctrl.enabled != 0U) && (s_feedback[ARM_J1_LZ].online == 0U) &&
        (s_enable_request_ms[ARM_J1_LZ] != 0U) &&
        ((uint32_t)(now - s_enable_request_ms[ARM_J1_LZ]) > kFeedbackTimeoutMs)) {
        j1_disable();
    }

    fp32 dt_s = kControlNominalDtSec;
    if (s_last_send_ms != 0U) {
        dt_s = (fp32)(now - s_last_send_ms) * 0.001f;
        if ((dt_s <= 0.0f) || (dt_s > kControlMaxDtSec)) {
            dt_s = kControlNominalDtSec;
        }
    }
    s_last_send_ms = now;

    if ((s_j0_ctrl.enabled != 0U) && (s_feedback[ARM_J0_DM4310].online != 0U)) {
        const fp32 j0_torque_nm = j0_calculate_torque(dt_s);
        s_j0_dm4310.can_send_torque_only(j0_torque_nm, s_j0_ctrl.invert);
    }

    if ((s_j1_ctrl.enabled != 0U) && (s_feedback[ARM_J1_LZ].online != 0U)) {
        if (s_j1_ctrl.first_set_angle != 0U) {
            j1_lock_target_to_measure();
            s_j1_ctrl.first_set_angle = 0U;
        }
        j1_apply_target_to_motor();
        s_j1_lz.can_send();
    }
}

void ArmMotor_DiagnosticPoll(uint32_t now_ms, uint8_t tx_allowed_mask)
{
    if (s_initialized == 0U) {
        return;
    }

    refresh_feedback_age(now_ms);
    for (uint8_t joint = 0U; joint < ARM_JOINT_COUNT; ++joint) {
        const uint8_t joint_mask = (uint8_t)(1U << joint);
        if ((s_feedback[joint].enabled != 0U) ||
            ((tx_allowed_mask & joint_mask) == 0U) ||
            ((s_diagnostic_sent[joint] != 0U) &&
             ((uint32_t)(now_ms - s_last_diagnostic_ms[joint]) < kDiagnosticPeriodMs))) {
            continue;
        }

        uint8_t sent = 0U;
        if (joint == ARM_J0_DM4310) {
            sent = s_j0_dm4310.probe_disable();
        } else if (joint == ARM_J1_LZ) {
            sent = s_j1_lz.probe_device_id();
        }
        if (sent != 0U) {
            s_diagnostic_sent[joint] = 1U;
            s_last_diagnostic_ms[joint] = now_ms;
        }
    }
}

uint8_t ArmMotor_OnCanRx(FDCAN_HandleTypeDef *hfdcan, const FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    if ((s_initialized == 0U) || (hfdcan == nullptr) || (header == nullptr) || (data == nullptr)) {
        return 0U;
    }

    if ((hfdcan == s_j0_can) && (header->IdType == FDCAN_STANDARD_ID)) {
        const uint8_t feedback_node = (uint8_t)(data[0] & 0x0FU);
        if ((header->Identifier == s_j0_dm_feedback_id) &&
            (fdcan_dlc_to_bytes(header->DataLength) == 8U) &&
            (feedback_node == (uint8_t)(s_j0_dm_id & 0x0FU))) {
            s_j0_dm4310.can_recv(data);
            copy_dm_feedback();
            mark_status_feedback(ARM_J0_DM4310, HAL_GetTick());
            return 1U;
        }
    }

    if ((hfdcan == s_j1_can) && (header->IdType == FDCAN_EXTENDED_ID)) {
        const uint8_t host_id = (uint8_t)(header->Identifier & 0xFFU);
        const uint8_t lz_id = (uint8_t)((header->Identifier >> 8) & 0xFFU);
        const uint8_t lz_mode = (uint8_t)((header->Identifier >> 24) & 0x1FU);
        if ((lz_mode == CANCOM_ANNOUNCE_DEVID) &&
            (host_id == kEl05DeviceIdResponseTarget) &&
            (lz_id == s_j1_lz_id) &&
            (fdcan_dlc_to_bytes(header->DataLength) == 8U)) {
            mark_present(ARM_J1_LZ, HAL_GetTick());
            return 1U;
        }
        if ((host_id == s_j1_ctrl.master_id) &&
            (lz_id == s_j1_lz_id) &&
            (fdcan_dlc_to_bytes(header->DataLength) == 8U) &&
            ((lz_mode == CANCOM_MODE_ACTIVE_RECV) || (lz_mode == CANCOM_MOTOR_FEEDBACK))) {
            s_j1_lz.can_recv(header->Identifier, data);
            copy_lz_feedback();
            mark_status_feedback(ARM_J1_LZ, HAL_GetTick());
            return 1U;
        }
    }

    return 0U;
}

uint8_t ArmMotor_GetFeedback(uint8_t joint, ArmMotorFeedback *feedback)
{
    if ((joint >= ARM_JOINT_COUNT) || (feedback == nullptr)) {
        return 0U;
    }
    refresh_feedback_age(HAL_GetTick());
    *feedback = s_feedback[joint];
    return s_feedback[joint].online;
}

uint8_t ArmMotor_IsInitialized(void)
{
    return s_initialized;
}
