#include "motor_task.h"

#include "arm_motor_task.h"
#include "bsp_fdcan.h"
#include "debug_uart.h"
#include "vofa_pid.h"

#include <string.h>
#include <math.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

#define ARM_J0_DM_CAN_ID               0x01U
#define ARM_J0_DM_FEEDBACK_ID          0x10U
#define ARM_J1_EL05_CAN_ID             0x7FU
#define ARM_J1_EL05_INIT_MODEL_IGNORED 0U
#define ARM_CMD_PERIOD_MS              1U

#define DOG_CTRL_HZ                    500U
#define DOG_CMD_PERIOD_MS              (1000U / DOG_CTRL_HZ)
#define DOG_RX_TIMEOUT_MS              250U
#define DOG_STAND_WAIT_MS              1500U
#define DOG_STAND_CONFIG_MS            200U
#define DOG_STAND_LOOP_MS              300U
#define DOG_STAND_MOVE_MS              2500U
#define DOG_JUMP_SETTLE_MS             500U
#define DOG_ENCODER_FEEDBACK_PERIOD_MS   DOG_CMD_PERIOD_MS
#define DOG_SLOW_FEEDBACK_PERIOD_MS    100U
#define DOG_DIAG_PERIOD_MS             1000U
#define DOG_CAN_TX_ECHO_GUARD_MS       2U
#define DOG_ENCODER_WAIT_MS            300U
#define DOG_ENCODER_TORQUE_WAIT_MS     500U
#define DOG_CLOSED_LOOP_RETRY_MS       200U

#define DOG_DEG_TO_TURN           0.002777777777777778f
#define DOG_TURN_TO_DEG           360.0f
#define DOG_PI                    3.14159265358979323846f
#define DOG_RAD_TO_DEG            57.29577951308232f

#define DOG_THIGH_MM              210.0f
#define DOG_SHANK_MM              250.0f
#define DOG_KNEE_MIN_DEG          33.0f
#define DOG_KNEE_MAX_DEG          143.0f
#define DOG_STAND_HEIGHT_MM       260.0f
#define DOG_STAND_X_MM            0.0f

#define DOG_TRAJ_VEL_LIMIT        3.0f
#define DOG_TRAJ_ACCEL_LIMIT      8.0f
#define DOG_TRAJ_DECEL_LIMIT      8.0f
#define DOG_TRAJ_INERTIA          0.0f

#define DOG_STAND_KP_A_PER_DEG          1.7f
#define DOG_STAND_KI_A_PER_DEG_S        0.00f
#define DOG_STAND_KD_A_PER_DPS          0.017f
#define DOG_STAND_OUTPUT_LIMIT_A        11.0f
#define DOG_MIT_ANG_PID_INTEGRAL_LIMIT_A 3.0f

#define DOG_SWING_KP_A_PER_DEG          1.7f
#define DOG_SWING_KI_A_PER_DEG_S        0.00f
#define DOG_SWING_KD_A_PER_DPS          0.017f
#define DOG_SWING_OUTPUT_LIMIT_A        11.0f

#define DOG_MIT_CURRENT_LIMIT_A         12.0f
#define DOG_MIT_VEL_LIMIT_TURN_S        30.0f
#define DOG_MIT_TORQUE_NM_PER_A         1.2f
#define DOG_MIT_TORQUE_LIMIT_NM         50.0f
#define DOG_MIT_POS_RAD_LIMIT           12.5f
#define DOG_TEACH_HOLD_BREAKAWAY_DPS    12.0f
#define DOG_TEACH_HOLD_RELEASE_DPS   2.5f
#define DOG_TEACH_HOLD_CAPTURE_MS    180U

#define DOG_LEG_CRANK_MM          112.50f
#define DOG_LEG_PARA_MM           210.00f
#define DOG_LEG_KNEE_OFFSET_H_MM  115.50f
#define DOG_LEG_KNEE_OFFSET_D_MM  65.00f

uint8_t g_mw_node_ids[DOG_MOTOR_COUNT] = {2U, 1U, 3U, 4U, 1U, 2U, 3U, 4U};
MW_MOTOR_DATA g_mw_motor_data[DOG_MOTOR_COUNT];

Dog_Motor_Config g_dog_motor_config[DOG_MOTOR_COUNT] = {
    /* LF: CAN1 id2=大腿(HIP), id1=小腿(KNEE 传力电机); encoder_gear_ratio=8 => 8 encoder turns / 1 joint turn */
    {DOG_LEG_LF, DOG_JOINT_HIP,  DOG_CAN_FRONT_BUS, 2U, 1.0f,  -1.0f, 0.0f, 8.0f, -120.0f, 120.0f},
    {DOG_LEG_LF, DOG_JOINT_KNEE, DOG_CAN_FRONT_BUS, 1U, -1.0f,  1.0f, 0.0f, 8.0f,  -120.0f, 143.0f},//33.0
    {DOG_LEG_RF, DOG_JOINT_HIP,  DOG_CAN_FRONT_BUS, 4U,  -1.0f, 1.0f, 0.0f, 8.0f, -120.0f, 120.0f},
    {DOG_LEG_RF, DOG_JOINT_KNEE, DOG_CAN_FRONT_BUS, 3U,  1.0f,  1.0f, 0.0f, 8.0f,   -120.0f, 143.0f},//33.0
    {DOG_LEG_LB, DOG_JOINT_HIP,  DOG_CAN_REAR_BUS,  2U, 1.0f,  -1.0f, 0.0f, 8.0f, -120.0f, 120.0f},
    {DOG_LEG_LB, DOG_JOINT_KNEE, DOG_CAN_REAR_BUS,  1U,  -1.0f, -1.0f, 0.0f, 8.0f,   -120.0f, 143.0f},//33.0
    {DOG_LEG_RB, DOG_JOINT_HIP,  DOG_CAN_REAR_BUS,  4U,  -1.0f, 1.0f, 0.0f, 8.0f, -120.0f, 120.0f},
    {DOG_LEG_RB, DOG_JOINT_KNEE, DOG_CAN_REAR_BUS,  3U, 1.0f,  1.0f, 0.0f, 8.0f,   -120.0f, 143.0f},//33.0
};

Dog_Leg_Kin_Params g_dog_leg_kin_params[DOG_LEG_COUNT] = {
    {DOG_LEG_CRANK_MM, DOG_THIGH_MM, DOG_SHANK_MM, DOG_LEG_PARA_MM,
     DOG_LEG_KNEE_OFFSET_H_MM, DOG_LEG_KNEE_OFFSET_D_MM,
     1.0f, 0.0f, 1.0f, 0.0f,
     DOG_LEG_KIN_LINEAR},
    {DOG_LEG_CRANK_MM, DOG_THIGH_MM, DOG_SHANK_MM, DOG_LEG_PARA_MM,
     DOG_LEG_KNEE_OFFSET_H_MM, DOG_LEG_KNEE_OFFSET_D_MM,
     1.0f, 0.0f, 1.0f, 0.0f,
     DOG_LEG_KIN_LINEAR},
    {DOG_LEG_CRANK_MM, DOG_THIGH_MM, DOG_SHANK_MM, DOG_LEG_PARA_MM,
     DOG_LEG_KNEE_OFFSET_H_MM, DOG_LEG_KNEE_OFFSET_D_MM,
     1.0f, 0.0f, 1.0f, 0.0f,
     DOG_LEG_KIN_LINEAR},
    {DOG_LEG_CRANK_MM, DOG_THIGH_MM, DOG_SHANK_MM, DOG_LEG_PARA_MM,
     DOG_LEG_KNEE_OFFSET_H_MM, DOG_LEG_KNEE_OFFSET_D_MM,
     1.0f, 0.0f, 1.0f, 0.0f,
     DOG_LEG_KIN_LINEAR},
};

Dog_Mit_Ang_Pid g_dog_mit_stand_pid = {
    DOG_STAND_KP_A_PER_DEG,
    DOG_STAND_KI_A_PER_DEG_S,
    DOG_STAND_KD_A_PER_DPS,
    DOG_STAND_OUTPUT_LIMIT_A,
};

Dog_Mit_Ang_Pid g_dog_mit_swing_pid = {
    DOG_SWING_KP_A_PER_DEG,
    DOG_SWING_KI_A_PER_DEG_S,
    DOG_SWING_KD_A_PER_DPS,
    DOG_SWING_OUTPUT_LIMIT_A,
};

Dog_Mit_Motor_Limits g_dog_mit_motor_limits = {
    DOG_MIT_CURRENT_LIMIT_A,
    DOG_MIT_VEL_LIMIT_TURN_S,
    DOG_MIT_TORQUE_NM_PER_A,
};

static uint8_t s_debug_target = DOG_DEBUG_TARGET_ALL;
static uint8_t s_target_leg = DOG_LEG_LF;
static uint8_t s_position_tx_enabled = 0U;
static uint8_t s_auto_stand_enabled = 0U;
static Dog_Control_Loop_Mode s_control_loop_mode = DOG_CTRL_LOOP_POSITION;
static Dog_Stand_State s_stand_state = DOG_STAND_IDLE;
static uint32_t s_state_start_ms = 0U;
static uint32_t s_last_command_tick_ms = 0U;
static uint32_t s_last_rx_tick_ms[DOG_MOTOR_COUNT];
static uint8_t s_motor_online[DOG_MOTOR_COUNT];
static uint8_t s_motor_configured[DOG_MOTOR_COUNT];
static uint8_t s_motor_loop_requested[DOG_MOTOR_COUNT];
static float s_zero_offset_turn[DOG_MOTOR_COUNT];
static float s_target_turn[DOG_MOTOR_COUNT];
static float s_start_turn[DOG_MOTOR_COUNT];
static float s_stand_turn[DOG_MOTOR_COUNT];
static float s_target_deg[DOG_MOTOR_COUNT];
static float s_mit_ang_integral_deg_s[DOG_MOTOR_COUNT];
static float s_mit_cmd_current_a[DOG_MOTOR_COUNT];
static float s_mit_pid_p_a[DOG_MOTOR_COUNT];
static float s_mit_pid_i_a[DOG_MOTOR_COUNT];
static float s_mit_pid_d_a[DOG_MOTOR_COUNT];
static float s_mit_pid_ff_a[DOG_MOTOR_COUNT];
static float s_mit_ang_err_prev_deg[DOG_MOTOR_COUNT];
static uint8_t s_mit_pid_profile = DOG_MIT_PID_SWING;
static uint8_t s_mit_mixed_pid_active = 0U;
static uint8_t s_mit_swing_motor_mask = 0U;

enum Dog_March_Phase {
    DOG_MARCH_PHASE_SWING_UP = 0U,
    DOG_MARCH_PHASE_HOLD = 1U,
    DOG_MARCH_PHASE_SWING_DOWN = 2U,
    DOG_MARCH_PHASE_PAUSE = 3U,
};

static struct {
    uint8_t active;
    uint8_t mode;
    uint8_t leg;
    uint8_t cycles_remaining;
    uint8_t phase;
    uint32_t phase_t0_ms;
    uint32_t swing_t0_ms;
    float lift_hip_deg;
    float lift_knee_deg;
    uint8_t trot_stride_applied;
} s_march = {};

static const uint8_t s_trot_swing_pairs[2U][2U] = {
    {DOG_LEG_LF, DOG_LEG_RB},
    {DOG_LEG_RF, DOG_LEG_LB},
};

struct Dog_Gait_Speed_Profile {
    const char *name;
    float trot_hz;
    float forward_stride_x_mm;
    float turn_stride_x_mm;
};

static const Dog_Gait_Speed_Profile s_gait_speed_profiles[] = {
    {"LOW",  DOG_TROT_HZ, DOG_FORWARD_STRIDE_X_MM, DOG_TURN_STRIDE_X_MM},
    {"MID",  DOG_TROT_HZ, DOG_FORWARD_STRIDE_X_MM, DOG_TURN_STRIDE_X_MM},
    {"HIGH", 3.0f, DOG_FORWARD_STRIDE_X_MM, DOG_TURN_STRIDE_X_MM},
};

static float s_leg_foot_x_offset[DOG_LEG_COUNT] = {};
static float s_leg_hip_offset_deg[DOG_LEG_COUNT] = {};
static float s_trot_direction_sign = 1.0f;
static uint8_t s_gait_speed_profile = DOG_GAIT_SPEED_DEFAULT;
static uint8_t s_diag_support_active = 0U;
static uint32_t s_mit_last_pid_ms = 0U;
static uint32_t s_parsed_frame_count[2];
static uint16_t s_last_parsed_id[2];
static uint8_t s_last_parsed_node[2];
static uint8_t s_last_parsed_cmd[2];
static uint16_t s_last_rx_id[2];
static uint8_t s_last_rx_len[2];
static uint8_t s_last_rx_ext[2];
static uint32_t s_rx_reject_format[2];
static uint32_t s_rx_reject_node[2];
static uint32_t s_rx_reject_nodata[2];
static uint32_t s_can_tx_tick_ms[2][256];
static uint8_t s_encoder_est_fresh[DOG_MOTOR_COUNT];
static uint32_t s_encoder_rx_count[DOG_MOTOR_COUNT];
static uint8_t s_motor_mit_probe_active[DOG_MOTOR_COUNT];
static uint8_t s_mit_debug_active = 0U;
static uint8_t s_mit_fault_hold_active = 0U;
static uint8_t s_teach_hold_active = 0U;
static uint8_t s_teach_hold_following[DOG_MOTOR_COUNT];
static uint32_t s_teach_hold_last_motion_ms[DOG_MOTOR_COUNT];
static float s_encoder_turn_filt[DOG_MOTOR_COUNT];
static uint8_t s_encoder_turn_valid[DOG_MOTOR_COUNT];
static uint8_t s_mit_boot_ok[DOG_MOTOR_COUNT];
static uint8_t s_mit_torque_test_active = 0U;
static uint8_t s_mit_torque_test_index = DOG_MOTOR_COUNT;
static float s_mit_torque_test_nm = 0.0f;
static uint8_t s_jump_active = 0U;
static Dog_Imu_Sample s_imu_sample;
static uint8_t s_imu_balance_enabled = 1U;
static uint8_t s_imu_balance_stand_hold_active = 0U;
static Dog_Remote_Sample s_remote_sample;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

uint8_t dog_imu_balance_is_enabled(void)
{
    return s_imu_balance_enabled;
}

void dog_imu_balance_set_enabled(uint8_t enable)
{
    s_imu_balance_enabled = (enable != 0U) ? 1U : 0U;
}

uint8_t dog_imu_balance_is_active(void)
{
    if (s_imu_balance_enabled == 0U) {
        return 0U;
    }
    if ((s_imu_sample.valid == 0U) || (s_imu_sample.calibrated == 0U)) {
        return 0U;
    }
    if (s_imu_sample.tick_ms == 0U) {
        return 0U;
    }
    if ((uint32_t)(HAL_GetTick() - s_imu_sample.tick_ms) > DOG_IMU_SAMPLE_TIMEOUT_MS) {
        return 0U;
    }
    return 1U;
}

void dog_imu_balance_get_leg_z_offsets(float offsets_mm[DOG_LEG_COUNT])
{
    if (offsets_mm == nullptr) {
        return;
    }
    for (uint8_t i = 0U; i < DOG_LEG_COUNT; ++i) {
        offsets_mm[i] = 0.0f;
    }
    if (dog_imu_balance_is_active() == 0U) {
        return;
    }

    const float roll_term = clampf(((s_imu_sample.roll_deg * DOG_IMU_BALANCE_ROLL_KP_MM_PER_DEG) +
                                    (s_imu_sample.gyro_dps[0] * DOG_IMU_BALANCE_D_MM_PER_DPS)) *
                                   DOG_IMU_BALANCE_ROLL_SIGN,
                                   -DOG_IMU_BALANCE_LIMIT_MM,
                                   DOG_IMU_BALANCE_LIMIT_MM);
    const float pitch_term = clampf(((s_imu_sample.pitch_deg * DOG_IMU_BALANCE_PITCH_KP_MM_PER_DEG) +
                                     (s_imu_sample.gyro_dps[1] * DOG_IMU_BALANCE_D_MM_PER_DPS)) *
                                    DOG_IMU_BALANCE_PITCH_SIGN,
                                    -DOG_IMU_BALANCE_LIMIT_MM,
                                    DOG_IMU_BALANCE_LIMIT_MM);

    offsets_mm[DOG_LEG_LF] = clampf(roll_term + pitch_term, -DOG_IMU_BALANCE_LIMIT_MM, DOG_IMU_BALANCE_LIMIT_MM);
    offsets_mm[DOG_LEG_RF] = clampf(-roll_term + pitch_term, -DOG_IMU_BALANCE_LIMIT_MM, DOG_IMU_BALANCE_LIMIT_MM);
    offsets_mm[DOG_LEG_LB] = clampf(roll_term - pitch_term, -DOG_IMU_BALANCE_LIMIT_MM, DOG_IMU_BALANCE_LIMIT_MM);
    offsets_mm[DOG_LEG_RB] = clampf(-roll_term - pitch_term, -DOG_IMU_BALANCE_LIMIT_MM, DOG_IMU_BALANCE_LIMIT_MM);
}

static float dog_imu_balance_z_for_leg(uint8_t leg)
{
    float offsets[DOG_LEG_COUNT] = {};
    if (leg >= DOG_LEG_COUNT) {
        return 0.0f;
    }
    if ((s_jump_active != 0U) || (s_diag_support_active != 0U)) {
        return 0.0f;
    }
    dog_imu_balance_get_leg_z_offsets(offsets);
    return offsets[leg];
}

static const Dog_Gait_Speed_Profile *gait_speed_profile(void)
{
    if (s_gait_speed_profile > DOG_GAIT_SPEED_HIGH) {
        s_gait_speed_profile = DOG_GAIT_SPEED_DEFAULT;
    }
    return &s_gait_speed_profiles[s_gait_speed_profile];
}

float dog_mit_gait_trot_hz(void)
{
    return gait_speed_profile()->trot_hz;
}

float dog_mit_gait_forward_stride_x_mm(void)
{
    return gait_speed_profile()->forward_stride_x_mm;
}

float dog_mit_gait_turn_stride_x_mm(void)
{
    return gait_speed_profile()->turn_stride_x_mm;
}

uint32_t dog_mit_gait_trot_swing_ms(void)
{
    const float hz = dog_mit_gait_trot_hz();
    if (hz <= 0.01f) {
        return DOG_TROT_SWING_MS;
    }
    return (uint32_t)((500.0f / hz) + 0.5f);
}

const char *dog_mit_gait_speed_profile_name(void)
{
    return gait_speed_profile()->name;
}

uint8_t dog_mit_get_gait_speed_profile(void)
{
    (void)gait_speed_profile();
    return s_gait_speed_profile;
}

void dog_mit_set_gait_speed_profile(uint8_t profile)
{
    if (profile > DOG_GAIT_SPEED_HIGH) {
        profile = DOG_GAIT_SPEED_DEFAULT;
    }
    s_gait_speed_profile = profile;
}

static float sqrt_newton(float v)
{
    if (v <= 0.0f) return 0.0f;
    float x = (v > 1.0f) ? v : 1.0f;
    for (uint8_t i = 0U; i < 8U; ++i) {
        x = 0.5f * (x + (v / x));
    }
    return x;
}

[[maybe_unused]] static float acos_approx(float x)
{
    x = clampf(x, -1.0f, 1.0f);
    float negate = (x < 0.0f) ? 1.0f : 0.0f;
    float ax = (x < 0.0f) ? -x : x;
    float ret = -0.0187293f;
    ret = ret * ax + 0.0742610f;
    ret = ret * ax - 0.2121144f;
    ret = ret * ax + 1.5707288f;
    ret = ret * sqrt_newton(1.0f - ax);
    ret = ret - (2.0f * negate * ret);
    return (negate * DOG_PI) + ret;
}

static uint8_t bus_to_diag_index(uint8_t bus)
{
    return (bus == DOG_CAN_REAR_BUS) ? 1U : 0U;
}

static FDCAN_HandleTypeDef *bus_handle(uint8_t bus)
{
    if (bus == DOG_CAN_FRONT_BUS) return &hfdcan1;
    if (bus == DOG_CAN_REAR_BUS) return &hfdcan2;
    return nullptr;
}

static uint8_t motor_index(uint8_t bus, uint8_t node_id)
{
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((g_dog_motor_config[i].bus == bus) && (g_dog_motor_config[i].node_id == node_id)) {
            return i;
        }
    }
    return DOG_MOTOR_COUNT;
}

static uint8_t leg_joint_index(uint8_t leg, uint8_t joint)
{
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((g_dog_motor_config[i].leg == leg) && (g_dog_motor_config[i].joint == joint)) {
            return i;
        }
    }
    return DOG_MOTOR_COUNT;
}

static uint8_t selected_count(void)
{
    if (s_debug_target == DOG_DEBUG_TARGET_SINGLE) return 1U;
    if (s_debug_target == DOG_DEBUG_TARGET_SINGLE_KNEE) return 1U;
    if (s_debug_target == DOG_DEBUG_TARGET_LEG) return DOG_MOTORS_PER_LEG;
    if (s_debug_target == DOG_DEBUG_TARGET_FRONT_PAIR) return DOG_MOTORS_PER_LEG * 2U;
    if (s_debug_target == DOG_DEBUG_TARGET_REAR_PAIR) return DOG_MOTORS_PER_LEG * 2U;
    return DOG_MOTOR_COUNT;
}

static uint8_t selected_index(uint8_t n)
{
    if (s_debug_target == DOG_DEBUG_TARGET_SINGLE) {
        return leg_joint_index(s_target_leg, DOG_JOINT_HIP);
    }
    if (s_debug_target == DOG_DEBUG_TARGET_SINGLE_KNEE) {
        return leg_joint_index(s_target_leg, DOG_JOINT_KNEE);
    }
    if (s_debug_target == DOG_DEBUG_TARGET_LEG) {
        return leg_joint_index(s_target_leg, n);
    }
    if (s_debug_target == DOG_DEBUG_TARGET_FRONT_PAIR) {
        if (n < DOG_MOTORS_PER_LEG) {
            return leg_joint_index(DOG_LEG_LF, n);
        }
        return leg_joint_index(DOG_LEG_RF, (uint8_t)(n - DOG_MOTORS_PER_LEG));
    }
    if (s_debug_target == DOG_DEBUG_TARGET_REAR_PAIR) {
        if (n < DOG_MOTORS_PER_LEG) {
            return leg_joint_index(DOG_LEG_LB, n);
        }
        return leg_joint_index(DOG_LEG_RB, (uint8_t)(n - DOG_MOTORS_PER_LEG));
    }
    return (n < DOG_MOTOR_COUNT) ? n : DOG_MOTOR_COUNT;
}

static uint8_t motor_is_selected(uint8_t index)
{
    for (uint8_t i = 0U; i < selected_count(); ++i) {
        if (selected_index(i) == index) {
            return 1U;
        }
    }
    return 0U;
}

static uint8_t leg_selected(uint8_t leg)
{
    if (s_debug_target == DOG_DEBUG_TARGET_ALL) {
        return 1U;
    }
    if (s_debug_target == DOG_DEBUG_TARGET_FRONT_PAIR) {
        return ((leg == DOG_LEG_LF) || (leg == DOG_LEG_RF)) ? 1U : 0U;
    }
    if (s_debug_target == DOG_DEBUG_TARGET_REAR_PAIR) {
        return ((leg == DOG_LEG_LB) || (leg == DOG_LEG_RB)) ? 1U : 0U;
    }
    return (leg == s_target_leg) ? 1U : 0U;
}

static float encoder_gear_ratio(uint8_t index)
{
    if (index >= DOG_MOTOR_COUNT) {
        return 1.0f;
    }
    float ratio = g_dog_motor_config[index].encoder_gear_ratio;
    return (ratio > 1.0e-3f) ? ratio : 1.0f;
}

static float mech_deg_from_encoder_turn(uint8_t index, float encoder_turn)
{
    if (index >= DOG_MOTOR_COUNT) {
        return 0.0f;
    }
    Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    return (encoder_turn * DOG_TURN_TO_DEG * cfg->direction) / encoder_gear_ratio(index);
}

static float raw_deg(uint8_t index)
{
    if (index >= DOG_MOTOR_COUNT) {
        return 0.0f;
    }
    return mech_deg_from_encoder_turn(index, g_mw_motor_data[index].encoderEstimates.encoderPosEstimate);
}

static float user_deg(uint8_t index)
{
    if (index >= DOG_MOTOR_COUNT) return 0.0f;
    float user_turn = g_mw_motor_data[index].encoderEstimates.encoderPosEstimate - s_zero_offset_turn[index];
    return mech_deg_from_encoder_turn(index, user_turn);
}

static float user_vel_dps(uint8_t index)
{
    if (index >= DOG_MOTOR_COUNT) return 0.0f;
    Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    return (g_mw_motor_data[index].encoderEstimates.encoderVelEstimate * DOG_TURN_TO_DEG * cfg->direction) /
           encoder_gear_ratio(index);
}

static void mit_reset_motor_integrator(uint8_t index);
static float user_rad(uint8_t index);
static void send_mit_zero_effort(uint8_t index, float pos_rad);

static const char *joint_name(uint8_t joint)
{
    return (joint == DOG_JOINT_HIP) ? "HIP" : "KNEE";
}

static void teach_hold_stop(void)
{
    s_teach_hold_active = 0U;
    memset(s_teach_hold_following, 0, sizeof(s_teach_hold_following));
}

static void mit_clear_mixed_pid(void)
{
    s_mit_mixed_pid_active = 0U;
    s_mit_swing_motor_mask = 0U;
}

static void mit_add_swing_leg_to_mask(uint8_t leg)
{
    uint8_t hip = leg_joint_index(leg, DOG_JOINT_HIP);
    uint8_t knee = leg_joint_index(leg, DOG_JOINT_KNEE);
    if (hip < DOG_MOTOR_COUNT) {
        s_mit_swing_motor_mask |= (uint8_t)(1U << hip);
        mit_reset_motor_integrator(hip);
    }
    if (knee < DOG_MOTOR_COUNT) {
        s_mit_swing_motor_mask |= (uint8_t)(1U << knee);
        mit_reset_motor_integrator(knee);
    }
}

static void mit_set_mixed_swing_legs(uint8_t leg_a, uint8_t leg_b)
{
    mit_clear_mixed_pid();
    s_mit_mixed_pid_active = 1U;
    mit_add_swing_leg_to_mask(leg_a);
    if (leg_b < DOG_LEG_COUNT) {
        mit_add_swing_leg_to_mask(leg_b);
    }
}

[[maybe_unused]] static void mit_set_mixed_swing_leg(uint8_t leg)
{
    mit_set_mixed_swing_legs(leg, DOG_LEG_COUNT);
}

static void mit_set_all_stand_pid_mode(void)
{
    mit_clear_mixed_pid();
    s_mit_pid_profile = DOG_MIT_PID_STAND;
}

static void mit_debug_stop_tx(void)
{
    memset(&s_march, 0, sizeof(s_march));
    memset(s_leg_foot_x_offset, 0, sizeof(s_leg_foot_x_offset));
    memset(s_leg_hip_offset_deg, 0, sizeof(s_leg_hip_offset_deg));
    s_diag_support_active = 0U;
    mit_clear_mixed_pid();
    s_position_tx_enabled = 0U;
    s_mit_debug_active = 0U;
    s_mit_fault_hold_active = 0U;
    s_last_command_tick_ms = 0U;
    s_mit_pid_profile = DOG_MIT_PID_SWING;
    teach_hold_stop();
    memset(s_mit_boot_ok, 0, sizeof(s_mit_boot_ok));
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        s_target_deg[i] = user_deg(i);
        s_target_turn[i] = g_mw_motor_data[i].encoderEstimates.encoderPosEstimate;
        mit_reset_motor_integrator(i);
    }
}

static void mit_debug_fault_hold(void)
{
    s_mit_debug_active = 0U;
    s_mit_fault_hold_active = 1U;
    s_position_tx_enabled = 1U;
    s_last_command_tick_ms = 0U;
    teach_hold_stop();

    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((s_mit_boot_ok[i] == 0U) || (s_encoder_est_fresh[i] == 0U)) {
            continue;
        }
        s_target_deg[i] = user_deg(i);
        s_target_turn[i] = g_mw_motor_data[i].encoderEstimates.encoderPosEstimate;
        mit_reset_motor_integrator(i);
    }
}

static void mit_debug_abort_fault_hold(const char *reason)
{
    DebugUart_Printf("SAFETY fault hold %s -> stop TX\r\n", reason);
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if (s_mit_boot_ok[i] != 0U) {
            send_mit_zero_effort(i, user_rad(i));
        }
    }
    mit_debug_stop_tx();
}

static uint8_t motor_closed_loop(uint8_t index)
{
    return ((index < DOG_MOTOR_COUNT) &&
            (g_mw_motor_data[index].heartBeat.currentState == MW_AXIS_STATE_CLOSED_LOOP_CONTROL)) ? 1U : 0U;
}

static uint8_t motor_has_fault(uint8_t index)
{
    if (index >= DOG_MOTOR_COUNT) return 1U;
    return (g_mw_motor_data[index].heartBeat.ErrorStatus.axisError != 0U) ||
           (g_mw_motor_data[index].heartBeat.ErrorStatus.motorErrorFlag != 0U) ||
           (g_mw_motor_data[index].heartBeat.ErrorStatus.encoderErrorFlag != 0U) ||
           (g_mw_motor_data[index].heartBeat.ErrorStatus.controllerErrorFlag != 0U) ||
           (g_mw_motor_data[index].heartBeat.ErrorStatus.ErrorFlag != 0U) ||
           (g_mw_motor_data[index].error != 0U);
}

static uint8_t motor_ready(uint8_t index)
{
    return ((index < DOG_MOTOR_COUNT) &&
            (s_motor_online[index] != 0U) &&
            (motor_closed_loop(index) != 0U) &&
            (motor_has_fault(index) == 0U)) ? 1U : 0U;
}

static float command_turn_from_user_deg(uint8_t index, float deg)
{
    if (index >= DOG_MOTOR_COUNT) return 0.0f;
    Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    float limited = deg;
    if (s_mit_debug_active == 0U) {
        limited = clampf(deg, cfg->min_deg, cfg->max_deg);
    }
    s_target_deg[index] = limited;
    return s_zero_offset_turn[index] +
           ((limited * cfg->direction + cfg->zero_offset_deg) * DOG_DEG_TO_TURN * encoder_gear_ratio(index));
}

static void mw_sender(uint8_t busId, uint8_t canId, uint8_t *data, uint8_t dataSize)
{
    FDCAN_HandleTypeDef *h = bus_handle(busId);
    if (h == nullptr) return;

    uint8_t di = bus_to_diag_index(busId);
    if (di < 2U) {
        s_can_tx_tick_ms[di][canId] = HAL_GetTick();
    }

    (void)fdcan_send_data_Exten(h, canId, data, dataSize);
}

static void mw_query_encoder(uint8_t index)
{
    if (index >= DOG_MOTOR_COUNT) {
        return;
    }

    Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    FDCAN_HandleTypeDef *h = bus_handle(cfg->bus);
    if (h == nullptr) {
        return;
    }

    uint8_t tx[8] = {0U};
    uint32_t id_est = (((uint32_t)cfg->node_id << 5) | MW_GET_ENCODER_ESTIMATES_CMD);
    uint32_t id_cnt = (((uint32_t)cfg->node_id << 5) | MW_GET_ENCODER_COUNT_CMD);
    uint8_t di = bus_to_diag_index(cfg->bus);
    uint32_t now = HAL_GetTick();

    if (di < 2U) {
        s_can_tx_tick_ms[di][(uint8_t)id_est] = now;
        s_can_tx_tick_ms[di][(uint8_t)id_cnt] = now;
    }

    (void)fdcan_send_data_Exten(h, id_est, tx, 8U);
    (void)fdcan_send_data_stand(h, id_est, tx, 8U);
    (void)fdcan_send_data_Exten(h, id_cnt, tx, 8U);
    (void)fdcan_send_data_stand(h, id_cnt, tx, 8U);
}

static uint8_t encoder_filter_estimate(uint8_t index)
{
    if (index >= DOG_MOTOR_COUNT) {
        return 0U;
    }

    float new_turn = g_mw_motor_data[index].encoderEstimates.encoderPosEstimate;

    if (s_encoder_turn_valid[index] == 0U) {
        s_encoder_turn_filt[index] = new_turn;
        s_encoder_turn_valid[index] = 1U;
        return 1U;
    }

    if ((fabsf(new_turn) < 1.0e-4f) && (fabsf(s_encoder_turn_filt[index]) > 0.05f)) {
        g_mw_motor_data[index].encoderEstimates.encoderPosEstimate = s_encoder_turn_filt[index];
        return 0U;
    }

    float delta_turn = new_turn - s_encoder_turn_filt[index];
    while (delta_turn > 0.5f) {
        new_turn -= 1.0f;
        delta_turn -= 1.0f;
    }
    while (delta_turn < -0.5f) {
        new_turn += 1.0f;
        delta_turn += 1.0f;
    }

    s_encoder_turn_filt[index] = new_turn;
    g_mw_motor_data[index].encoderEstimates.encoderPosEstimate = new_turn;
    return 1U;
}

static void mw_notifier(uint8_t busId, uint8_t nodeId, MW_CMD_ID cmdId)
{
    uint8_t index = motor_index(busId, nodeId);
    if (index >= DOG_MOTOR_COUNT) return;

    if ((cmdId == MW_GET_ENCODER_ESTIMATES_CMD) || (cmdId == MW_GET_ENCODER_COUNT_CMD)) {
        s_encoder_rx_count[index]++;
        if ((cmdId == MW_GET_ENCODER_ESTIMATES_CMD) && (motor_closed_loop(index) != 0U)) {
            s_encoder_est_fresh[index] = encoder_filter_estimate(index);
        }
    }

    uint8_t diag = bus_to_diag_index(busId);
    s_motor_online[index] = 1U;
    s_last_rx_tick_ms[index] = HAL_GetTick();
    s_parsed_frame_count[diag]++;
    s_last_parsed_node[diag] = nodeId;
    s_last_parsed_cmd[diag] = (uint8_t)cmdId;
}

static void configure_motor_position(uint8_t index)
{
    if ((index >= DOG_MOTOR_COUNT) || (s_motor_online[index] == 0U) || (motor_has_fault(index) != 0U)) {
        return;
    }
    Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    MWSetControllerMode(cfg->bus, cfg->node_id, MW_POSITION_CONTROL, MW_TRAPEZOIDAL_CURVE_INPUT);
    MWSetTrajVelLimit(cfg->bus, cfg->node_id, DOG_TRAJ_VEL_LIMIT);
    MWSetTrajAccelLimits(cfg->bus, cfg->node_id, DOG_TRAJ_ACCEL_LIMIT, DOG_TRAJ_DECEL_LIMIT);
    MWSetTrajInertia(cfg->bus, cfg->node_id, DOG_TRAJ_INERTIA);
    MWSetLimits(cfg->bus, cfg->node_id, DOG_TRAJ_VEL_LIMIT, g_dog_mit_motor_limits.current_limit_a);
    s_motor_configured[index] = 1U;
}

static void configure_motor_mit(uint8_t index)
{
    if ((index >= DOG_MOTOR_COUNT) || (s_motor_online[index] == 0U) || (motor_has_fault(index) != 0U)) {
        return;
    }
    Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    MWSetControllerMode(cfg->bus, cfg->node_id, MW_TORQUE_CONTROL, MW_MIT_INPUT);
    MWSetLimits(cfg->bus, cfg->node_id, g_dog_mit_motor_limits.vel_limit_turn_s, g_dog_mit_motor_limits.current_limit_a);
    s_motor_configured[index] = 1U;
}

static void configure_motor(uint8_t index)
{
    configure_motor_position(index);
}

static float clamp_current_a(float current_a)
{
    float lim = g_dog_mit_motor_limits.current_limit_a;
    return clampf(current_a, -lim, lim);
}

static float clamp_ang_output_a(float current_a, const Dog_Mit_Ang_Pid *pid)
{
    if (pid == nullptr) {
        return 0.0f;
    }

    float lim = pid->output_limit_a;
    if (lim <= 0.0f) {
        return 0.0f;
    }
    return clampf(current_a, -lim, lim);
}

static const Dog_Mit_Ang_Pid *mit_active_ang_pid(void)
{
    return (s_mit_pid_profile == DOG_MIT_PID_STAND) ? &g_dog_mit_stand_pid : &g_dog_mit_swing_pid;
}

static const Dog_Mit_Ang_Pid *mit_ang_pid_for_motor(uint8_t index)
{
    const uint8_t use_swing = (s_mit_mixed_pid_active != 0U) && (index < DOG_MOTOR_COUNT) &&
                              ((s_mit_swing_motor_mask & (uint8_t)(1U << index)) != 0U);

    if (use_swing != 0U) {
        return &g_dog_mit_swing_pid;
    }
    if (s_mit_pid_profile == DOG_MIT_PID_STAND) {
        return &g_dog_mit_stand_pid;
    }
    return &g_dog_mit_swing_pid;
}

static float mit_jump_dynamic_ff_a(uint8_t index, float err_deg)
{
    if ((s_jump_active == 0U) || (index >= DOG_MOTOR_COUNT)) {
        return 0.0f;
    }
    if (mit_ang_pid_for_motor(index) != &g_dog_mit_stand_pid) {
        return 0.0f;
    }

    const Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    if ((cfg->leg != DOG_LEG_LB) && (cfg->leg != DOG_LEG_RB)) {
        return 0.0f;
    }
    if (fabsf(err_deg) < DOG_JUMP_FF_ERR_DEADBAND_DEG) {
        return 0.0f;
    }
    if (g_dog_mit_motor_limits.torque_nm_per_a <= 0.0f) {
        return 0.0f;
    }

    const float ff_max_a = DOG_JUMP_FF_MAX_NM / g_dog_mit_motor_limits.torque_nm_per_a;
    return (err_deg > 0.0f) ? ff_max_a : -ff_max_a;
}

static float user_rad(uint8_t index)
{
    return user_deg(index) * (DOG_PI / 180.0f);
}

static void mit_reset_motor_integrator(uint8_t index)
{
    if (index >= DOG_MOTOR_COUNT) return;
    s_mit_ang_integral_deg_s[index] = 0.0f;
    s_mit_cmd_current_a[index] = 0.0f;
    s_mit_pid_p_a[index] = 0.0f;
    s_mit_pid_i_a[index] = 0.0f;
    s_mit_pid_d_a[index] = 0.0f;
    s_mit_pid_ff_a[index] = 0.0f;
    s_mit_ang_err_prev_deg[index] = s_target_deg[index] - user_deg(index);
}

static float mit_ang_compute_err_rate_dps(uint8_t index, float err_deg, float dt_s)
{
    float err_rate_dps = 0.0f;
    if ((index < DOG_MOTOR_COUNT) && (dt_s > 0.0f) && (dt_s <= 0.1f)) {
        err_rate_dps = (err_deg - s_mit_ang_err_prev_deg[index]) / dt_s;
    }
    if (index < DOG_MOTOR_COUNT) {
        s_mit_ang_err_prev_deg[index] = err_deg;
    }
    return err_rate_dps;
}

static float mit_ang_pid_compute_current_a(uint8_t index, float dt_s)
{
    if (index >= DOG_MOTOR_COUNT) return 0.0f;

    const Dog_Mit_Ang_Pid *pid = mit_ang_pid_for_motor(index);
    float err_deg = s_target_deg[index] - user_deg(index);
    if (s_mit_debug_active != 0U) {
        err_deg = clampf(err_deg, -DOG_MIT_DEBUG_MAX_ERR_DEG, DOG_MIT_DEBUG_MAX_ERR_DEG);
    }

    s_mit_ang_integral_deg_s[index] += err_deg * dt_s;

    if ((pid->ki_a_per_deg_s > 1.0e-6f) && (DOG_MIT_ANG_PID_INTEGRAL_LIMIT_A > 0.0f)) {
        const float int_deg_s_lim = DOG_MIT_ANG_PID_INTEGRAL_LIMIT_A / pid->ki_a_per_deg_s;
        s_mit_ang_integral_deg_s[index] =
            clampf(s_mit_ang_integral_deg_s[index], -int_deg_s_lim, int_deg_s_lim);
    }

    float err_rate_dps = mit_ang_compute_err_rate_dps(index, err_deg, dt_s);
    float ang_p = pid->kp_a_per_deg * err_deg;
    float ang_i = pid->ki_a_per_deg_s * s_mit_ang_integral_deg_s[index];
    float ang_d = pid->kd_a_per_dps * err_rate_dps;
    float ang_ff = mit_jump_dynamic_ff_a(index, err_deg);
    float current_a = ang_p + ang_i + ang_d + ang_ff;

    s_mit_pid_p_a[index] = ang_p;
    s_mit_pid_i_a[index] = ang_i;
    s_mit_pid_d_a[index] = ang_d;
    s_mit_pid_ff_a[index] = ang_ff;

    float limited_a = clamp_ang_output_a(current_a, pid);
    limited_a = clamp_current_a(limited_a);
    if (limited_a != current_a) {
        s_mit_ang_integral_deg_s[index] -= err_deg * dt_s;
        s_mit_pid_i_a[index] = pid->ki_a_per_deg_s * s_mit_ang_integral_deg_s[index];
    }

    s_mit_cmd_current_a[index] = limited_a;
    return limited_a;
}

static void send_mit_fixed_torque(uint8_t index, float torque_nm)
{
    if (index >= DOG_MOTOR_COUNT) {
        return;
    }

    Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    MW_MIT_CTRL mit = {};
    mit.pos = 0.0;
    mit.vel = 0.0;
    mit.kp = 0.0;
    mit.kd = 0.0;
    mit.torque = torque_nm;
    (void)MWMitControl(cfg->bus, cfg->node_id, &mit);
}

static void send_mit_zero_effort(uint8_t index, float pos_rad)
{
    if (index >= DOG_MOTOR_COUNT) {
        return;
    }

    Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    MW_MIT_CTRL mit = {};
    mit.pos = clampf(pos_rad, -DOG_MIT_POS_RAD_LIMIT, DOG_MIT_POS_RAD_LIMIT);
    mit.vel = 0.0;
    mit.kp = 0.0;
    mit.kd = 0.0;
    mit.torque = 0.0;
    (void)MWMitControl(cfg->bus, cfg->node_id, &mit);
}

static void teach_hold_update_target(uint8_t index, uint32_t now)
{
    if ((s_teach_hold_active == 0U) || (index >= DOG_MOTOR_COUNT)) {
        return;
    }
    if ((motor_is_selected(index) == 0U) || (s_encoder_est_fresh[index] == 0U)) {
        return;
    }

    float speed_dps = fabsf(user_vel_dps(index));
    if (speed_dps > DOG_TEACH_HOLD_BREAKAWAY_DPS) {
        s_teach_hold_following[index] = 1U;
        s_teach_hold_last_motion_ms[index] = now;
    }

    if (s_teach_hold_following[index] == 0U) {
        return;
    }

    if (speed_dps > DOG_TEACH_HOLD_RELEASE_DPS) {
        s_teach_hold_last_motion_ms[index] = now;
    }

    if ((uint32_t)(now - s_teach_hold_last_motion_ms[index]) >= DOG_TEACH_HOLD_CAPTURE_MS) {
        s_teach_hold_following[index] = 0U;
        return;
    }

    {
        s_target_deg[index] = user_deg(index);
        s_target_turn[index] = g_mw_motor_data[index].encoderEstimates.encoderPosEstimate;
        mit_reset_motor_integrator(index);
    }
}

static void send_mit_torque_commands(uint32_t now)
{
    float dt_s = (float)DOG_CMD_PERIOD_MS * 0.001f;
    if (s_mit_last_pid_ms != 0U) {
        dt_s = (float)(now - s_mit_last_pid_ms) * 0.001f;
        if ((dt_s <= 0.0f) || (dt_s > 0.1f)) {
            dt_s = (float)DOG_CMD_PERIOD_MS * 0.001f;
        }
    }
    s_mit_last_pid_ms = now;

    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((s_mit_fault_hold_active != 0U) && (s_mit_boot_ok[i] != 0U) &&
            ((s_motor_online[i] == 0U) || (s_encoder_est_fresh[i] == 0U) || (motor_closed_loop(i) == 0U))) {
            mit_debug_abort_fault_hold("lost feedback");
            return;
        }
        if (motor_ready(i) == 0U) {
            if ((s_mit_fault_hold_active != 0U) && (s_mit_boot_ok[i] != 0U)) {
                mit_reset_motor_integrator(i);
            }
            continue;
        }
        if (s_mit_debug_active != 0U) {
            if (motor_is_selected(i) == 0U) {
                continue;
            }
        }
        if ((s_mit_fault_hold_active != 0U) && (s_mit_boot_ok[i] == 0U)) {
            continue;
        }
        if (s_encoder_est_fresh[i] == 0U) {
            send_mit_zero_effort(i, user_rad(i));
            mit_reset_motor_integrator(i);
            continue;
        }

        Dog_Motor_Config *cfg = &g_dog_motor_config[i];
        teach_hold_update_target(i, now);
        float current_a = mit_ang_pid_compute_current_a(i, dt_s);

        if (s_mit_debug_active != 0U) {
            float user_now_deg = user_deg(i);
            float err_deg = s_target_deg[i] - user_now_deg;
            if (fabsf(err_deg) > DOG_MIT_DEBUG_SAFETY_ERR_DEG) {
                    if (VofaPid_IsEnabled() == 0U) {
                        DebugUart_Printf("SAFETY M%u %s %s bus%u id%u user=%ldmdeg tgt=%ldmdeg err=%ldmdeg -> fault hold\r\n",
                                         (unsigned)i,
                                         dog_leg_name(cfg->leg),
                                         joint_name(cfg->joint),
                                         (unsigned)cfg->bus,
                                         (unsigned)cfg->node_id,
                                         (long)(user_now_deg * 1000.0f),
                                         (long)(s_target_deg[i] * 1000.0f),
                                         (long)(err_deg * 1000.0f));
                    }
                    for (uint8_t k = 0U; k < DOG_MOTOR_COUNT; ++k) {
                        if (s_mit_boot_ok[k] != 0U) {
                            send_mit_zero_effort(k, user_rad(k));
                        }
                    }
                    mit_debug_fault_hold();
                    return;
                }
        }

        float torque_nm = clampf(current_a * g_dog_mit_motor_limits.torque_nm_per_a * cfg->torque_direction,
                                 -DOG_MIT_TORQUE_LIMIT_NM, DOG_MIT_TORQUE_LIMIT_NM);
        float pos_rad = clampf(user_rad(i), -DOG_MIT_POS_RAD_LIMIT, DOG_MIT_POS_RAD_LIMIT);

        MW_MIT_CTRL mit = {};
        mit.pos = pos_rad;
        mit.vel = 0.0;
        mit.kp = 0.0;
        mit.kd = 0.0;
        mit.torque = torque_nm;
        (void)MWMitControl(cfg->bus, cfg->node_id, &mit);
    }

    if (VofaPid_IsEnabled() != 0U) {
        Dog_Mit_Pid_Telemetry telemetry = {};
        if (dog_mit_get_pid_telemetry(VofaPid_GetMotorIndex(), &telemetry) != 0U) {
            VofaPid_SendTelemetry(&telemetry);
        }
    }
}

static void send_mit_torque_test_keepalive(uint32_t now)
{
    if (s_mit_torque_test_active == 0U) {
        return;
    }

    static uint32_t s_last_ms = 0U;
    if ((uint32_t)(now - s_last_ms) < DOG_CMD_PERIOD_MS) {
        return;
    }
    s_last_ms = now;

    if (s_mit_torque_test_index < DOG_MOTOR_COUNT) {
        send_mit_fixed_torque(s_mit_torque_test_index, s_mit_torque_test_nm);
    }
}

static void send_mit_probe_keepalive(uint32_t now)
{
    static uint32_t s_last_ms = 0U;
    if ((uint32_t)(now - s_last_ms) < DOG_CMD_PERIOD_MS) {
        return;
    }
    s_last_ms = now;

    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((s_motor_mit_probe_active[i] != 0U) && (motor_closed_loop(i) != 0U)) {
            send_mit_zero_effort(i, 0.0f);
        }
    }
}

static uint8_t wait_motor_encoder(uint8_t index, uint32_t timeout_ms)
{
    if (index >= DOG_MOTOR_COUNT) {
        return 0U;
    }

    uint32_t t0 = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - t0) < timeout_ms) {
        send_mit_zero_effort(index, 0.0f);
        mw_query_encoder(index);
        fdcan_poll_rx(&hfdcan1);
        fdcan_poll_rx(&hfdcan2);
        if ((motor_closed_loop(index) != 0U) && (s_encoder_est_fresh[index] != 0U)) {
            return 1U;
        }
        HAL_Delay(5U);
    }
    return ((motor_closed_loop(index) != 0U) && (s_encoder_est_fresh[index] != 0U)) ? 1U : 0U;
}

static uint8_t enter_mit_probe_closed_loop(uint8_t index)
{
    if (index >= DOG_MOTOR_COUNT) {
        return 0U;
    }

    Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    MWSetLimits(cfg->bus, cfg->node_id,
                g_dog_mit_motor_limits.vel_limit_turn_s,
                DOG_MIT_PROBE_CURRENT_LIMIT_A);
    MWSetControllerMode(cfg->bus, cfg->node_id, MW_TORQUE_CONTROL, MW_MIT_INPUT);
    send_mit_zero_effort(index, 0.0f);
    MWSetAxisState(cfg->bus, cfg->node_id, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    s_motor_mit_probe_active[index] = 1U;
    fdcan_poll_rx(&hfdcan1);
    fdcan_poll_rx(&hfdcan2);
    return 1U;
}

static uint8_t request_mit_probe_for_angle(uint8_t index, uint32_t encoder_wait_ms)
{
    if ((index >= DOG_MOTOR_COUNT) || (s_motor_online[index] == 0U) || (motor_has_fault(index) != 0U)) {
        return 0U;
    }

    s_encoder_est_fresh[index] = 0U;
    s_encoder_turn_valid[index] = 0U;
    (void)enter_mit_probe_closed_loop(index);

    uint8_t have_encoder = ((s_encoder_est_fresh[index] != 0U) && (motor_closed_loop(index) != 0U)) ? 1U : 0U;
    if ((have_encoder == 0U) && (encoder_wait_ms > 0U)) {
        have_encoder = wait_motor_encoder(index, encoder_wait_ms);
    }

    fdcan_poll_rx(&hfdcan1);
    fdcan_poll_rx(&hfdcan2);
    s_motor_loop_requested[index] = 1U;

    if ((have_encoder == 0U) || (motor_closed_loop(index) == 0U)) {
        return 2U;
    }

    return 1U;
}

static uint8_t request_closed_loop_with_position_hold(uint8_t index)
{
    if ((index >= DOG_MOTOR_COUNT) || (s_motor_online[index] == 0U) || (motor_has_fault(index) != 0U)) {
        return 0U;
    }
    Dog_Motor_Config *cfg = &g_dog_motor_config[index];

    (void)enter_mit_probe_closed_loop(index);

    if (s_encoder_est_fresh[index] == 0U) {
        s_motor_loop_requested[index] = 1U;
        return 2U;
    }

    float hold_turn = g_mw_motor_data[index].encoderEstimates.encoderPosEstimate;
    s_target_turn[index] = hold_turn;
    s_target_deg[index] = user_deg(index);

    if (s_control_loop_mode == DOG_CTRL_LOOP_MIT_PID) {
        configure_motor_mit(index);
        send_mit_zero_effort(index, user_rad(index));
    } else {
        configure_motor_position(index);
        MWPosControl(cfg->bus, cfg->node_id, hold_turn, 0, 0);
        MWSetAxisState(cfg->bus, cfg->node_id, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    }

    fdcan_poll_rx(&hfdcan1);
    fdcan_poll_rx(&hfdcan2);
    s_motor_mit_probe_active[index] = 0U;
    s_motor_loop_requested[index] = 1U;
    return 1U;
}

static uint8_t request_closed_loop(uint8_t index)
{
    return request_closed_loop_with_position_hold(index);
}

static uint8_t foot_xz_workspace_ok(float x_mm, float z_mm, float *d_out)
{
    float l1 = DOG_THIGH_MM;
    float l2 = DOG_SHANK_MM;

    if (z_mm <= 0.0f) {
        return 0U;
    }

    float d2 = (x_mm * x_mm) + (z_mm * z_mm);
    float d = sqrt_newton(d2);
    float min_d = sqrt_newton(l1 * l1 + l2 * l2 - 2.0f * l1 * l2 * (float)0.83867057f);
    float max_d = sqrt_newton(l1 * l1 + l2 * l2 - 2.0f * l1 * l2 * (float)-0.79863551f);

    if ((d < min_d) || (d > max_d)) {
        return 0U;
    }

    if (d_out != nullptr) {
        *d_out = d;
    }
    return 1U;
}

static float s_ik_hip_zero_rad = 0.0f;
static float s_ik_knee_zero_rad = 0.0f;
static uint8_t s_ik_calib_ready = 0U;

static uint8_t dog_ik_foot_geom(float x_mm, float z_mm, float *range_leg_rad, float *range_separate_rad)
{
    const float l1 = DOG_THIGH_MM;
    const float l2 = DOG_SHANK_MM;
    const float l = sqrtf((x_mm * x_mm) + (z_mm * z_mm));

    if (l < 1.0e-3f) {
        return 0U;
    }

    const float sin_range_leg = clampf(x_mm / l, -1.0f, 1.0f);
    const float cos_psi = clampf((l * l + l1 * l1 - l2 * l2) / (2.0f * l1 * l), -1.0f, 1.0f);

    if (range_leg_rad != nullptr) {
        *range_leg_rad = asinf(sin_range_leg);
    }
    if (range_separate_rad != nullptr) {
        *range_separate_rad = acosf(cos_psi);
    }
    return 1U;
}

static void dog_ik_add_calib_point(float motor_hip_deg, float motor_knee_deg,
                                   float foot_x_mm, float foot_z_mm,
                                   float *hip_off_sum_deg, float *knee_off_sum_deg,
                                   uint8_t *count)
{
    float range_leg = 0.0f;
    float range_separate = 0.0f;

    if (dog_ik_foot_geom(foot_x_mm, foot_z_mm, &range_leg, &range_separate) == 0U) {
        return;
    }

    const float psi_deg = range_separate * DOG_RAD_TO_DEG;
    const float range_leg_deg = range_leg * DOG_RAD_TO_DEG;
    *hip_off_sum_deg += motor_hip_deg + psi_deg - range_leg_deg;
    *knee_off_sum_deg += motor_knee_deg + psi_deg + range_leg_deg;
    *count = (uint8_t)(*count + 1U);
}

static void dog_ik_ensure_calib(void)
{
    if (s_ik_calib_ready != 0U) {
        return;
    }

    float hip_off_sum_deg = 0.0f;
    float knee_off_sum_deg = 0.0f;
    uint8_t count = 0U;

    dog_ik_add_calib_point(DOG_IK_CAL_A_MOTOR_HIP_DEG, DOG_IK_CAL_A_MOTOR_KNEE_DEG,
                           DOG_IK_CAL_A_FOOT_X_MM, DOG_IK_CAL_A_FOOT_Z_MM,
                           &hip_off_sum_deg, &knee_off_sum_deg, &count);
    dog_ik_add_calib_point(DOG_IK_CAL_B_MOTOR_HIP_DEG, DOG_IK_CAL_B_MOTOR_KNEE_DEG,
                           DOG_IK_CAL_B_FOOT_X_MM, DOG_IK_CAL_B_FOOT_Z_MM,
                           &hip_off_sum_deg, &knee_off_sum_deg, &count);

    if (count == 0U) {
        s_ik_hip_zero_rad = DOG_PI * 0.5f;
        s_ik_knee_zero_rad = DOG_PI * 0.5f;
    } else {
        const float inv = 1.0f / (float)count;
        s_ik_hip_zero_rad = (hip_off_sum_deg * inv) * (DOG_PI / 180.0f);
        s_ik_knee_zero_rad = (knee_off_sum_deg * inv) * (DOG_PI / 180.0f);
    }
    s_ik_calib_ready = 1U;
}

static float leg_ik_x_sign(uint8_t leg)
{
    uint8_t hip = leg_joint_index(leg, DOG_JOINT_HIP);
    if (hip >= DOG_MOTOR_COUNT) {
        return 1.0f;
    }
    return (g_dog_motor_config[hip].direction > 0.0f) ? 1.0f : -1.0f;
}

static uint8_t dog_leg_is_rear(uint8_t leg)
{
    return ((leg == DOG_LEG_LB) || (leg == DOG_LEG_RB)) ? 1U : 0U;
}

static float dog_leg_stand_foot_z_mm(uint8_t leg)
{
    float z_mm = DOG_STAND_FOOT_Z_MM;
    if (dog_leg_is_rear(leg) != 0U) {
        z_mm += DOG_REAR_FOOT_EXTRA_Z_MM;
    }
    z_mm += dog_imu_balance_z_for_leg(leg);
    return z_mm;
}

static float dog_leg_stand_foot_z_start_mm(uint8_t leg)
{
    float z_mm = DOG_STAND_FOOT_Z_START_MM;
    if (dog_leg_is_rear(leg) != 0U) {
        z_mm += DOG_REAR_FOOT_EXTRA_Z_MM;
    }
    return z_mm;
}

static uint8_t dog_leg_is_right(uint8_t leg)
{
    return ((leg == DOG_LEG_RF) || (leg == DOG_LEG_RB)) ? 1U : 0U;
}

static float march_trot_apply_yaw_trim(float delta, uint8_t leg)
{
    if (DOG_TROT_YAW_TRIM_X_MM <= 0.0f) {
        return delta;
    }

    const float trim = DOG_TROT_YAW_TRIM_X_MM;
    const float toward_zero = (delta > 0.0f) ? trim : -trim;
    if (dog_leg_is_right(leg) != 0U) {
        delta += toward_zero;
    } else {
        delta -= toward_zero;
    }
    return delta;
}

static float march_trot_forward_x_delta(uint8_t leg)
{
    const float delta = dog_mit_gait_forward_stride_x_mm() * s_trot_direction_sign *
                        leg_ik_x_sign(leg) * DOG_TROT_FORWARD_X_SIGN;
    return march_trot_apply_yaw_trim(delta, leg);
}

static uint8_t march_mode_uses_cycloid(uint8_t mode)
{
    return ((mode == DOG_MARCH_MODE_TROT) || (mode == DOG_MARCH_MODE_TURN_LEFT) ||
            (mode == DOG_MARCH_MODE_TURN_RIGHT)) ? 1U : 0U;
}

static uint8_t march_mode_is_turn(uint8_t mode)
{
    return ((mode == DOG_MARCH_MODE_TURN_LEFT) || (mode == DOG_MARCH_MODE_TURN_RIGHT)) ? 1U : 0U;
}

static float march_stride_x_delta(uint8_t leg)
{
    /*
     * Forward trot: LF-/RB+ user-x (leg_ik_x_sign) -> same physical per diagonal.
     * Turn: uniform user-x swing delta -> IK diagonal opposite; support holds x1 so
     *       each diagonal swing half-cycle adds the same yaw (support scrape was CW).
     */
    if (s_march.mode == DOG_MARCH_MODE_TURN_LEFT) {
        (void)leg;
        return -dog_mit_gait_turn_stride_x_mm() * DOG_TROT_FORWARD_X_SIGN;
    }
    if (s_march.mode == DOG_MARCH_MODE_TURN_RIGHT) {
        (void)leg;
        return dog_mit_gait_turn_stride_x_mm() * DOG_TROT_FORWARD_X_SIGN;
    }
    return march_trot_forward_x_delta(leg);
}

static float march_trot_swing_peak_z_mm(uint8_t leg)
{
    return dog_leg_stand_foot_z_mm(leg) + DOG_TROT_FOOT_Z1_MM + DOG_TROT_FOOT_Z2_MM;
}

static uint8_t dog_leg_inverse_xz_mm(float x_mm, float z_mm, float *hip_motor_deg, float *knee_motor_deg)
{
    dog_ik_ensure_calib();

    float range_leg = 0.0f;
    float range_separate = 0.0f;
    if (dog_ik_foot_geom(x_mm, z_mm, &range_leg, &range_separate) == 0U) {
        return 0U;
    }

    const float hip_rad = s_ik_hip_zero_rad - (range_separate - range_leg);
    const float knee_rad = s_ik_knee_zero_rad - (range_separate + range_leg);

    if (hip_motor_deg != nullptr) {
        *hip_motor_deg = hip_rad * DOG_RAD_TO_DEG;
    }
    if (knee_motor_deg != nullptr) {
        *knee_motor_deg = knee_rad * DOG_RAD_TO_DEG;
    }
    return 1U;
}

static uint8_t dog_leg_fk_from_motor_rad(float hip_rad, float knee_rad, float *x_mm, float *z_mm)
{
    dog_ik_ensure_calib();

    const float l1 = DOG_THIGH_MM;
    const float l2 = DOG_SHANK_MM;
    const float psi = (s_ik_hip_zero_rad + s_ik_knee_zero_rad - hip_rad - knee_rad) * 0.5f;
    const float range_leg = (s_ik_knee_zero_rad - s_ik_hip_zero_rad - knee_rad + hip_rad) * 0.5f;
    const float cos_psi = cosf(psi);
    const float b = -2.0f * l1 * cos_psi;
    const float c = (l1 * l1) - (l2 * l2);
    const float disc = (b * b) - (4.0f * c);

    if (disc < 0.0f) {
        return 0U;
    }

    const float sqrt_disc = sqrtf(disc);
    float l = (-b + sqrt_disc) * 0.5f;
    const float l_alt = (-b - sqrt_disc) * 0.5f;

    if ((l <= 1.0e-3f) || ((l_alt > l) && (l_alt > 1.0e-3f))) {
        l = l_alt;
    }
    if (l <= 1.0e-3f) {
        return 0U;
    }

    const float x = l * sinf(range_leg);
    const float z2 = (l * l) - (x * x);
    if (z2 < 0.0f) {
        return 0U;
    }

    if (x_mm != nullptr) {
        *x_mm = x;
    }
    if (z_mm != nullptr) {
        *z_mm = sqrtf(z2);
    }
    return 1U;
}

uint8_t dog_leg_foot_xz_is_reachable(float x_mm, float z_mm)
{
    return foot_xz_workspace_ok(x_mm, z_mm, nullptr);
}

static uint8_t foot_motor_in_limits(uint8_t leg, float hip_motor_deg, float knee_motor_deg)
{
    uint8_t hip_idx = leg_joint_index(leg, DOG_JOINT_HIP);
    uint8_t knee_idx = leg_joint_index(leg, DOG_JOINT_KNEE);

    if (hip_idx < DOG_MOTOR_COUNT) {
        const Dog_Motor_Config *cfg = &g_dog_motor_config[hip_idx];
        if ((hip_motor_deg < cfg->min_deg) || (hip_motor_deg > cfg->max_deg)) {
            return 0U;
        }
    }
    if (knee_idx < DOG_MOTOR_COUNT) {
        const Dog_Motor_Config *cfg = &g_dog_motor_config[knee_idx];
        if ((knee_motor_deg < cfg->min_deg) || (knee_motor_deg > cfg->max_deg)) {
            return 0U;
        }
    }
    return 1U;
}

uint8_t dog_leg_forward_kinematics(float hip_motor_deg, float knee_motor_deg,
                                   float *x_mm, float *z_mm)
{
    const float hip_rad = hip_motor_deg * (DOG_PI / 180.0f);
    const float knee_rad = knee_motor_deg * (DOG_PI / 180.0f);
    return dog_leg_fk_from_motor_rad(hip_rad, knee_rad, x_mm, z_mm);
}

uint8_t dog_leg_foot_xz_from_motor_deg(uint8_t leg, float hip_motor_deg, float knee_motor_deg,
                                     float *x_mm, float *z_mm)
{
    if (leg >= DOG_LEG_COUNT) {
        return 0U;
    }

    float x = 0.0f;
    float z = 0.0f;
    if (dog_leg_forward_kinematics(hip_motor_deg, knee_motor_deg, &x, &z) == 0U) {
        return 0U;
    }

    const float x_sign = leg_ik_x_sign(leg);
    if (x_mm != nullptr) {
        *x_mm = x * x_sign;
    }
    if (z_mm != nullptr) {
        *z_mm = z;
    }
    return 1U;
}

uint8_t dog_leg_foot_x_stand_ref_mm(uint8_t leg, float *stand_x_mm)
{
    (void)leg;
    if (stand_x_mm != nullptr) {
        *stand_x_mm = DOG_STAND_FOOT_X_MM;
    }
    return 1U;
}

uint8_t dog_leg_foot_xz_to_motor_deg(uint8_t leg, float x_mm, float z_mm,
                                     float *hip_motor_deg, float *knee_motor_deg)
{
    if (leg >= DOG_LEG_COUNT) {
        return 0U;
    }

    if (foot_xz_workspace_ok(x_mm, z_mm, nullptr) == 0U) {
        return 0U;
    }

    const float x_ik = x_mm * leg_ik_x_sign(leg);
    float hip_motor = 0.0f;
    float knee_motor = 0.0f;
    if (dog_leg_inverse_xz_mm(x_ik, z_mm, &hip_motor, &knee_motor) == 0U) {
        return 0U;
    }

    if (foot_motor_in_limits(leg, hip_motor, knee_motor) == 0U) {
        return 0U;
    }

    if (hip_motor_deg != nullptr) {
        *hip_motor_deg = hip_motor;
    }
    if (knee_motor_deg != nullptr) {
        *knee_motor_deg = knee_motor;
    }
    return 1U;
}

static const Dog_Leg_Kin_Params *leg_kin_params(uint8_t leg)
{
    if (leg >= DOG_LEG_COUNT) return nullptr;
    return &g_dog_leg_kin_params[leg];
}

static float deg_to_rad(float deg)
{
    return deg * (DOG_PI / 180.0f);
}

static float rad_to_deg(float rad)
{
    return rad * DOG_RAD_TO_DEG;
}

static uint8_t leg_motor_to_geom_linear(const Dog_Leg_Kin_Params *p,
                                        float q_thigh_motor_deg, float q_shank_motor_deg,
                                        float *thigh_geom_deg, float *knee_geom_deg)
{
    if (p == nullptr) return 0U;

    float thigh = (q_thigh_motor_deg * p->thigh_scale) + p->thigh_offset_deg;
    float knee = (q_shank_motor_deg * p->shank_scale) + p->shank_offset_deg;

    if (thigh_geom_deg != nullptr) {
        *thigh_geom_deg = clampf(thigh, -120.0f, 120.0f);
    }
    if (knee_geom_deg != nullptr) {
        *knee_geom_deg = clampf(knee, DOG_KNEE_MIN_DEG, DOG_KNEE_MAX_DEG);
    }
    return 1U;
}

static uint8_t leg_geom_to_motor_linear(const Dog_Leg_Kin_Params *p,
                                      float thigh_geom_deg, float knee_geom_deg,
                                      float *q_thigh_motor_deg, float *q_shank_motor_deg)
{
    if (p == nullptr) return 0U;
    if ((fabsf(p->thigh_scale) < 1.0e-6f) || (fabsf(p->shank_scale) < 1.0e-6f)) {
        return 0U;
    }

    float q_thigh = (thigh_geom_deg - p->thigh_offset_deg) / p->thigh_scale;
    float q_shank = (knee_geom_deg - p->shank_offset_deg) / p->shank_scale;

    if (q_thigh_motor_deg != nullptr) {
        *q_thigh_motor_deg = q_thigh;
    }
    if (q_shank_motor_deg != nullptr) {
        *q_shank_motor_deg = q_shank;
    }
    return 1U;
}

/*
 * 闭链正解占位：用 CAD 六尺寸做共轴五连杆 + 平行四边形近似。
 * 当前假设：大腿电机主控 theta_thigh，小腿传力电机主控 theta_knee，
 * 112.5 曲柄与 115.5/65 膝端偏置带来小量耦合修正。
 * 若后续标定发现线性层足够，可保持 DOG_LEG_KIN_LINEAR。
 */
static uint8_t leg_motor_to_geom_closed_chain(const Dog_Leg_Kin_Params *p,
                                              float q_thigh_motor_deg, float q_shank_motor_deg,
                                              float *thigh_geom_deg, float *knee_geom_deg)
{
    if (p == nullptr) return 0U;

    float q1 = deg_to_rad(q_thigh_motor_deg);
    float q2 = deg_to_rad(q_shank_motor_deg);
    float r = p->crank_mm;
    float L1 = p->thigh_mm;
    float kh = p->knee_offset_h_mm;
    float kd = p->knee_offset_d_mm;
    float knee_attach = sqrt_newton((kh * kh) + (kd * kd));

    float thigh = (q_thigh_motor_deg * p->thigh_scale) + p->thigh_offset_deg;
    float knee = (q_shank_motor_deg * p->shank_scale) + p->shank_offset_deg;

    if (L1 > 1.0e-3f) {
        float crank_couple = rad_to_deg((r / L1) * sinf(q1));
        thigh += crank_couple;
    }
    if (knee_attach > 1.0e-3f) {
        float para_couple = rad_to_deg((r / knee_attach) * sinf(q2 - q1));
        knee += para_couple;
    }

    if (thigh_geom_deg != nullptr) {
        *thigh_geom_deg = clampf(thigh, -120.0f, 120.0f);
    }
    if (knee_geom_deg != nullptr) {
        *knee_geom_deg = clampf(knee, DOG_KNEE_MIN_DEG, DOG_KNEE_MAX_DEG);
    }
    return 1U;
}

/*
 * 闭链逆解占位：先用线性逆解，再用一次耦合修正（与正解对应的一阶近似）。
 * 完整 Newton 闭链逆解可在标定阶段替换此函数体。
 */
static uint8_t leg_geom_to_motor_closed_chain(const Dog_Leg_Kin_Params *p,
                                              float thigh_geom_deg, float knee_geom_deg,
                                              float *q_thigh_motor_deg, float *q_shank_motor_deg)
{
    float q_thigh = 0.0f;
    float q_shank = 0.0f;
    if (leg_geom_to_motor_linear(p, thigh_geom_deg, knee_geom_deg, &q_thigh, &q_shank) == 0U) {
        return 0U;
    }

    if (p == nullptr) return 0U;

    float q1 = deg_to_rad(q_thigh);
    float r = p->crank_mm;
    float L1 = p->thigh_mm;
    float kh = p->knee_offset_h_mm;
    float kd = p->knee_offset_d_mm;
    float knee_attach = sqrt_newton((kh * kh) + (kd * kd));

    if (L1 > 1.0e-3f) {
        float crank_couple = rad_to_deg((r / L1) * sinf(q1));
        q_thigh -= crank_couple / p->thigh_scale;
    }
    if (knee_attach > 1.0e-3f) {
        float q2 = deg_to_rad(q_shank);
        float para_couple = rad_to_deg((r / knee_attach) * sinf(q2 - q1));
        q_shank -= para_couple / p->shank_scale;
    }

    if (q_thigh_motor_deg != nullptr) {
        *q_thigh_motor_deg = q_thigh;
    }
    if (q_shank_motor_deg != nullptr) {
        *q_shank_motor_deg = q_shank;
    }
    return 1U;
}

uint8_t dog_leg_motor_to_geom(uint8_t leg, float q_thigh_motor_deg, float q_shank_motor_deg,
                              float *thigh_geom_deg, float *knee_geom_deg)
{
    const Dog_Leg_Kin_Params *p = leg_kin_params(leg);
    if (p == nullptr) return 0U;

    if (p->mode == DOG_LEG_KIN_CLOSED_CHAIN) {
        return leg_motor_to_geom_closed_chain(p, q_thigh_motor_deg, q_shank_motor_deg,
                                              thigh_geom_deg, knee_geom_deg);
    }
    return leg_motor_to_geom_linear(p, q_thigh_motor_deg, q_shank_motor_deg,
                                    thigh_geom_deg, knee_geom_deg);
}

uint8_t dog_leg_geom_to_motor(uint8_t leg, float thigh_geom_deg, float knee_geom_deg,
                              float *q_thigh_motor_deg, float *q_shank_motor_deg)
{
    const Dog_Leg_Kin_Params *p = leg_kin_params(leg);
    if (p == nullptr) return 0U;

    if (p->mode == DOG_LEG_KIN_CLOSED_CHAIN) {
        return leg_geom_to_motor_closed_chain(p, thigh_geom_deg, knee_geom_deg,
                                              q_thigh_motor_deg, q_shank_motor_deg);
    }
    return leg_geom_to_motor_linear(p, thigh_geom_deg, knee_geom_deg,
                                    q_thigh_motor_deg, q_shank_motor_deg);
}

uint8_t dog_leg_read_geom_from_motors(uint8_t leg, float *thigh_geom_deg, float *knee_geom_deg)
{
    uint8_t hip = leg_joint_index(leg, DOG_JOINT_HIP);
    uint8_t knee = leg_joint_index(leg, DOG_JOINT_KNEE);
    if ((hip >= DOG_MOTOR_COUNT) || (knee >= DOG_MOTOR_COUNT)) {
        return 0U;
    }

    return dog_leg_motor_to_geom(leg, user_deg(hip), user_deg(knee), thigh_geom_deg, knee_geom_deg);
}

void dog_leg_set_kin_mode(uint8_t leg, uint8_t mode)
{
    if (leg >= DOG_LEG_COUNT) return;
    g_dog_leg_kin_params[leg].mode = (mode == DOG_LEG_KIN_CLOSED_CHAIN) ?
                                     DOG_LEG_KIN_CLOSED_CHAIN : DOG_LEG_KIN_LINEAR;
}

void dog_leg_set_kin_linear_calib(uint8_t leg, float thigh_scale, float thigh_offset_deg,
                                  float shank_scale, float shank_offset_deg)
{
    if (leg >= DOG_LEG_COUNT) return;
    g_dog_leg_kin_params[leg].thigh_scale = thigh_scale;
    g_dog_leg_kin_params[leg].thigh_offset_deg = thigh_offset_deg;
    g_dog_leg_kin_params[leg].shank_scale = shank_scale;
    g_dog_leg_kin_params[leg].shank_offset_deg = shank_offset_deg;
}

static void prepare_stand_targets(void)
{
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        float q_thigh = 0.0f;
        float q_shank = 0.0f;
        if (dog_leg_foot_xz_to_motor_deg(leg, DOG_STAND_FOOT_X_MM, dog_leg_stand_foot_z_mm(leg),
                                         &q_thigh, &q_shank) == 0U) {
            q_thigh = 0.0f;
            q_shank = DOG_KNEE_MIN_DEG;
        }

        uint8_t hip_idx = leg_joint_index(leg, DOG_JOINT_HIP);
        uint8_t knee_idx = leg_joint_index(leg, DOG_JOINT_KNEE);
        if (hip_idx < DOG_MOTOR_COUNT) {
            s_zero_offset_turn[hip_idx] = g_mw_motor_data[hip_idx].encoderEstimates.encoderPosEstimate;
            s_start_turn[hip_idx] = s_zero_offset_turn[hip_idx];
            s_stand_turn[hip_idx] = command_turn_from_user_deg(hip_idx, q_thigh);
            s_target_turn[hip_idx] = s_start_turn[hip_idx];
        }
        if (knee_idx < DOG_MOTOR_COUNT) {
            s_zero_offset_turn[knee_idx] = g_mw_motor_data[knee_idx].encoderEstimates.encoderPosEstimate;
            s_start_turn[knee_idx] = s_zero_offset_turn[knee_idx];
            s_stand_turn[knee_idx] = command_turn_from_user_deg(knee_idx, q_shank);
            s_target_turn[knee_idx] = s_start_turn[knee_idx];
        }
    }
}

static uint8_t any_online(void)
{
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if (s_motor_online[i] != 0U) return 1U;
    }
    return 0U;
}

static uint8_t online_fault_present(void)
{
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((s_motor_online[i] != 0U) && (motor_has_fault(i) != 0U)) {
            return 1U;
        }
    }
    return 0U;
}

static void send_enabled_targets(uint32_t now)
{
    if ((uint32_t)(now - s_last_command_tick_ms) < DOG_CMD_PERIOD_MS) return;
    s_last_command_tick_ms = now;

    if (s_control_loop_mode == DOG_CTRL_LOOP_MIT_PID) {
        send_mit_torque_commands(now);
        return;
    }

    for (uint8_t i = 0U; i < selected_count(); ++i) {
        uint8_t idx = selected_index(i);
        if ((idx < DOG_MOTOR_COUNT) && (motor_ready(idx) != 0U)) {
            Dog_Motor_Config *cfg = &g_dog_motor_config[idx];
            MWPosControl(cfg->bus, cfg->node_id, s_target_turn[idx], 0, 0);
        }
    }
}

static void stand_state_tick(uint32_t now)
{
    if (s_remote_sample.estop_request != 0U) {
        DogStand_Estop();
        s_remote_sample.estop_request = 0U;
    }
    if (s_remote_sample.stand_request != 0U) {
        DogStand_Request();
        s_remote_sample.stand_request = 0U;
    }
    if ((s_auto_stand_enabled == 0U) || (s_stand_state == DOG_STAND_ESTOP)) {
        return;
    }
    if (online_fault_present() != 0U) {
        s_position_tx_enabled = 0U;
        s_stand_state = DOG_STAND_FAULT;
        return;
    }

    switch (s_stand_state) {
    case DOG_STAND_WAIT_HEARTBEAT:
        if (((uint32_t)(now - s_state_start_ms) >= DOG_STAND_WAIT_MS) || (any_online() != 0U)) {
            prepare_stand_targets();
            for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) configure_motor(i);
            s_state_start_ms = now;
            s_stand_state = DOG_STAND_CONFIGURE;
        }
        break;
    case DOG_STAND_CONFIGURE:
        if ((uint32_t)(now - s_state_start_ms) >= DOG_STAND_CONFIG_MS) {
            for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) request_closed_loop(i);
            s_state_start_ms = now;
            s_stand_state = DOG_STAND_CLOSED_LOOP;
        }
        break;
    case DOG_STAND_CLOSED_LOOP:
        if ((uint32_t)(now - s_state_start_ms) >= DOG_STAND_LOOP_MS) {
            s_position_tx_enabled = 1U;
            s_last_command_tick_ms = 0U;
            s_state_start_ms = now;
            s_stand_state = DOG_STAND_MOVING;
        }
        break;
    case DOG_STAND_MOVING: {
        uint32_t elapsed = now - s_state_start_ms;
        float ratio = (elapsed >= DOG_STAND_MOVE_MS) ? 1.0f : ((float)elapsed / (float)DOG_STAND_MOVE_MS);
        for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
            s_target_turn[i] = s_start_turn[i] + ((s_stand_turn[i] - s_start_turn[i]) * ratio);
        }
        if (ratio >= 1.0f) {
            s_stand_state = DOG_STAND_STANDING;
        }
        break;
    }
    case DOG_STAND_STANDING:
        for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
            s_target_turn[i] = s_stand_turn[i];
        }
        break;
    default:
        break;
    }
}

static uint32_t mw_can_id_from_header(const FDCAN_RxHeaderTypeDef &header)
{
    if ((header.IdType != FDCAN_STANDARD_ID) && (header.IdType != FDCAN_EXTENDED_ID)) {
        return 0xFFFFFFFFU;
    }
    return header.Identifier & 0x7FFU;
}

static void dispatch_mw_rx(uint8_t bus, FDCAN_RxHeaderTypeDef &header, uint8_t *buffer)
{
    uint8_t di = bus_to_diag_index(bus);
    uint8_t len = fdcan_dlc_to_bytes(header.DataLength);
    uint32_t can_id = mw_can_id_from_header(header);

    s_last_rx_id[di] = (uint16_t)can_id;
    s_last_rx_len[di] = len;
    s_last_rx_ext[di] = (header.IdType == FDCAN_EXTENDED_ID) ? 1U : 0U;

    if ((can_id > 0x7FFU) || (len != 8U)) {
        s_rx_reject_format[di]++;
        return;
    }

    uint8_t node_id = (uint8_t)(can_id >> 5);
    if (motor_index(bus, node_id) >= DOG_MOTOR_COUNT) {
        s_rx_reject_node[di]++;
        return;
    }

    if ((node_id >= MAX_MOTOR_NUM_PER_BUS) ||
        (motors[bus][node_id].motorData == nullptr)) {
        s_rx_reject_nodata[di]++;
        return;
    }

    s_last_parsed_id[di] = (uint16_t)can_id;
    MWReceiver(bus, can_id, buffer);
}

static void CAN1_Callback(FDCAN_RxHeaderTypeDef &header, uint8_t *buffer)
{
    DebugUart_LogCanRx(1U, header.Identifier, buffer, fdcan_dlc_to_bytes(header.DataLength));
    if (ArmMotor_OnCanRx(&hfdcan1, &header, buffer) != 0U) {
        return;
    }
    dispatch_mw_rx(DOG_CAN_FRONT_BUS, header, buffer);
}

static void CAN2_Callback(FDCAN_RxHeaderTypeDef &header, uint8_t *buffer)
{
    DebugUart_LogCanRx(2U, header.Identifier, buffer, fdcan_dlc_to_bytes(header.DataLength));
    if (ArmMotor_OnCanRx(&hfdcan2, &header, buffer) != 0U) {
        return;
    }
    dispatch_mw_rx(DOG_CAN_REAR_BUS, header, buffer);
}

static void fill_diag(uint8_t bus, Dog_Can_Diag *diag)
{
    if (diag == nullptr) return;
    FDCAN_HandleTypeDef *h = bus_handle(bus);
    if (h == nullptr) return;

    FDCAN_ProtocolStatusTypeDef ps = {};
    FDCAN_ErrorCountersTypeDef ec = {};
    HAL_FDCAN_GetProtocolStatus(h, &ps);
    HAL_FDCAN_GetErrorCounters(h, &ec);

    uint8_t di = bus_to_diag_index(bus);
    memset(diag, 0, sizeof(*diag));
    diag->bus_off = (uint8_t)ps.BusOff;
    diag->error_passive = (uint8_t)ps.ErrorPassive;
    diag->warning = (uint8_t)ps.Warning;
    diag->activity = (uint8_t)ps.Activity;
    diag->last_error_code = (uint8_t)ps.LastErrorCode;
    diag->data_last_error_code = (uint8_t)ps.DataLastErrorCode;
    diag->tx_error_count = (uint8_t)ec.TxErrorCnt;
    diag->rx_error_count = (uint8_t)ec.RxErrorCnt;
    diag->rx_error_passive = (uint8_t)ec.RxErrorPassive;
    diag->error_logging = (uint8_t)ec.ErrorLogging;
    diag->rx_fifo_fill = (uint8_t)HAL_FDCAN_GetRxFifoFillLevel(h, FDCAN_RX_FIFO0);
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((g_dog_motor_config[i].bus == bus) && (s_motor_online[i] != 0U)) {
            diag->parsed_node_mask |= (uint8_t)(1U << (g_dog_motor_config[i].node_id - 1U));
        }
    }
    diag->last_parsed_id = s_last_parsed_id[di];
    diag->last_parsed_node = s_last_parsed_node[di];
    diag->last_parsed_cmd = s_last_parsed_cmd[di];
    diag->rx_frame_count = fdcan_rx_count(h);
    diag->parsed_frame_count = s_parsed_frame_count[di];
    diag->last_rx_id = s_last_rx_id[di];
    diag->last_rx_len = s_last_rx_len[di];
    diag->last_rx_ext = s_last_rx_ext[di];
    diag->rx_reject_format = s_rx_reject_format[di];
    diag->rx_reject_node = s_rx_reject_node[di];
    diag->rx_reject_nodata = s_rx_reject_nodata[di];
}

static void restart_bus_off(FDCAN_HandleTypeDef *h)
{
    if (h == nullptr) return;
    FDCAN_ProtocolStatusTypeDef ps = {};
    HAL_FDCAN_GetProtocolStatus(h, &ps);
    if (ps.BusOff == 0U) return;
    HAL_FDCAN_Stop(h);
    HAL_FDCAN_Start(h);
    HAL_FDCAN_ActivateNotification(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

const char *dog_leg_name(uint8_t leg)
{
    switch (leg) {
    case DOG_LEG_LF: return "LF";
    case DOG_LEG_RF: return "RF";
    case DOG_LEG_LB: return "LB";
    case DOG_LEG_RB: return "RB";
    default: return "??";
    }
}

const char *dog_debug_target_name(void)
{
    if (s_debug_target == DOG_DEBUG_TARGET_SINGLE) return "SINGLE_HIP";
    if (s_debug_target == DOG_DEBUG_TARGET_SINGLE_KNEE) return "SINGLE_KNEE";
    if (s_debug_target == DOG_DEBUG_TARGET_LEG) return "LEG";
    if (s_debug_target == DOG_DEBUG_TARGET_FRONT_PAIR) return "FRONT_PAIR";
    if (s_debug_target == DOG_DEBUG_TARGET_REAR_PAIR) return "REAR_PAIR";
    return "ALL";
}

const char *dog_control_loop_mode_name(void)
{
    return (s_control_loop_mode == DOG_CTRL_LOOP_MIT_PID) ? "MIT_PID" : "POSITION";
}

uint8_t dog_control_get_loop_mode(void)
{
    return (uint8_t)s_control_loop_mode;
}

void dog_mit_reset_integrators(void)
{
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        mit_reset_motor_integrator(i);
    }
    s_mit_last_pid_ms = 0U;
}

void dog_mit_send_control_now(void)
{
    if ((s_position_tx_enabled == 0U) || (s_control_loop_mode != DOG_CTRL_LOOP_MIT_PID)) {
        return;
    }
    s_last_command_tick_ms = 0U;
    send_mit_torque_commands(HAL_GetTick());
    fdcan_poll_rx(&hfdcan1);
    fdcan_poll_rx(&hfdcan2);
}

void dog_mit_clamp_integrators(void)
{
}

float dog_mit_get_cmd_current_a(uint8_t motor_index)
{
    return (motor_index < DOG_MOTOR_COUNT) ? s_mit_cmd_current_a[motor_index] : 0.0f;
}

uint8_t dog_mit_get_pid_profile(void)
{
    return s_mit_pid_profile;
}

const char *dog_mit_pid_profile_name(void)
{
    return (s_mit_pid_profile == DOG_MIT_PID_STAND) ? "STAND" : "SWING";
}

void dog_mit_set_pid_profile(uint8_t profile)
{
    if (profile > DOG_MIT_PID_SWING) {
        return;
    }

    memset(&s_march, 0, sizeof(s_march));
    mit_clear_mixed_pid();
    s_mit_pid_profile = profile;
    dog_mit_reset_integrators();
}

const Dog_Mit_Ang_Pid *dog_mit_active_ang_pid(void)
{
    return mit_active_ang_pid();
}

static void mit_pump_control(uint32_t now)
{
    send_mit_torque_commands(now);
    fdcan_poll_rx(&hfdcan1);
    fdcan_poll_rx(&hfdcan2);
}

static uint8_t mit_wait_joint_settle(uint8_t joint, float err_threshold_deg, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - t0) < timeout_ms) {
        if (dog_mit_fault_hold_is_active() != 0U) {
            return 0U;
        }

        mit_pump_control(HAL_GetTick());

        uint8_t all_settled = 1U;
        for (uint8_t i = 0U; i < selected_count(); ++i) {
            uint8_t idx = selected_index(i);
            if (idx >= DOG_MOTOR_COUNT) {
                continue;
            }
            if (g_dog_motor_config[idx].joint != joint) {
                continue;
            }
            if (s_encoder_est_fresh[idx] == 0U) {
                all_settled = 0U;
                continue;
            }
            if (fabsf(s_target_deg[idx] - user_deg(idx)) > err_threshold_deg) {
                all_settled = 0U;
            }
        }

        if (all_settled != 0U) {
            return 1U;
        }
        HAL_Delay(5U);
    }

    return 1U;
}

uint8_t dog_mit_goto_motor_pose(float hip_motor_deg, float knee_motor_deg)
{
    if (dog_mit_debug_is_active() == 0U) {
        return 0U;
    }
    if (dog_mit_fault_hold_is_active() != 0U) {
        return 0U;
    }

    dog_leg_set_target_motor_user_deg(hip_motor_deg, knee_motor_deg);
    dog_mit_reset_integrators();
    dog_mit_send_control_now();
    return 1U;
}

uint8_t dog_mit_goto_stand_pose(void)
{
    return dog_mit_goto_motor_pose(DOG_STAND_POSE_HIP_MOTOR_DEG, DOG_STAND_POSE_KNEE_MOTOR_DEG);
}

static uint8_t mit_wait_joint_settle_for_leg(uint8_t leg, uint8_t joint, float err_threshold_deg,
                                             uint32_t timeout_ms)
{
    uint8_t idx = leg_joint_index(leg, joint);
    if (idx >= DOG_MOTOR_COUNT) {
        return 0U;
    }

    uint32_t t0 = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - t0) < timeout_ms) {
        if (dog_mit_fault_hold_is_active() != 0U) {
            return 0U;
        }

        mit_pump_control(HAL_GetTick());

        if (s_encoder_est_fresh[idx] == 0U) {
            HAL_Delay(5U);
            continue;
        }
        if (fabsf(s_target_deg[idx] - user_deg(idx)) <= err_threshold_deg) {
            return 1U;
        }
        HAL_Delay(5U);
    }

    return 1U;
}

static void dog_leg_set_motor_user_deg_for_leg(uint8_t leg, float hip_motor_deg, float knee_motor_deg)
{
    uint8_t hip = leg_joint_index(leg, DOG_JOINT_HIP);
    uint8_t knee = leg_joint_index(leg, DOG_JOINT_KNEE);

    teach_hold_stop();
    if (hip < DOG_MOTOR_COUNT) {
        (void)command_turn_from_user_deg(hip, hip_motor_deg);
    }
    if (knee < DOG_MOTOR_COUNT) {
        (void)command_turn_from_user_deg(knee, knee_motor_deg);
    }
}

static uint8_t mit_move_motor_pose_sequential_for_leg(uint8_t leg, float hip_motor_deg, float knee_motor_deg)
{
    dog_leg_set_motor_user_deg_for_leg(leg, hip_motor_deg, knee_motor_deg);
    dog_mit_reset_integrators();
    dog_mit_send_control_now();
    if (mit_wait_joint_settle_for_leg(leg, DOG_JOINT_HIP, DOG_STAND_HIP_SETTLE_ERR_DEG,
                                      DOG_STAND_HIP_MOVE_MS) == 0U) {
        return 0U;
    }

    dog_leg_set_motor_user_deg_for_leg(leg, hip_motor_deg, knee_motor_deg);
    dog_mit_reset_integrators();
    dog_mit_send_control_now();
    if (mit_wait_joint_settle_for_leg(leg, DOG_JOINT_KNEE, DOG_STAND_KNEE_SETTLE_ERR_DEG,
                                      DOG_STAND_KNEE_MOVE_MS) == 0U) {
        return 0U;
    }

    return 1U;
}

[[maybe_unused]] static uint8_t mit_move_motor_pose_sequential(float hip_motor_deg, float knee_motor_deg)
{
    if (s_debug_target != DOG_DEBUG_TARGET_SINGLE_KNEE) {
        float knee_cmd = knee_motor_deg;
        if (s_debug_target == DOG_DEBUG_TARGET_SINGLE) {
            uint8_t knee_idx = leg_joint_index(s_target_leg, DOG_JOINT_KNEE);
            knee_cmd = (knee_idx < DOG_MOTOR_COUNT) ? s_target_deg[knee_idx] : knee_motor_deg;
        }
        dog_leg_set_target_motor_user_deg(hip_motor_deg, knee_cmd);
        dog_mit_reset_integrators();
        dog_mit_send_control_now();
        if (mit_wait_joint_settle(DOG_JOINT_HIP, DOG_STAND_HIP_SETTLE_ERR_DEG, DOG_STAND_HIP_MOVE_MS) == 0U) {
            return 0U;
        }
    }

    if (s_debug_target != DOG_DEBUG_TARGET_SINGLE) {
        float hip_cmd = hip_motor_deg;
        if (s_debug_target == DOG_DEBUG_TARGET_SINGLE_KNEE) {
            uint8_t hip_idx = leg_joint_index(s_target_leg, DOG_JOINT_HIP);
            hip_cmd = (hip_idx < DOG_MOTOR_COUNT) ? s_target_deg[hip_idx] : hip_motor_deg;
        }
        dog_leg_set_target_motor_user_deg(hip_cmd, knee_motor_deg);
        dog_mit_reset_integrators();
        dog_mit_send_control_now();
        if (mit_wait_joint_settle(DOG_JOINT_KNEE, DOG_STAND_KNEE_SETTLE_ERR_DEG, DOG_STAND_KNEE_MOVE_MS) == 0U) {
            return 0U;
        }
    }

    return 1U;
}

static uint8_t march_leg_joints_settled(uint8_t leg, float err_threshold_deg);

static uint8_t leg_is_in_debug_target(uint8_t leg)
{
    if (leg >= DOG_LEG_COUNT) {
        return 0U;
    }

    switch (s_debug_target) {
    case DOG_DEBUG_TARGET_ALL:
        return 1U;
    case DOG_DEBUG_TARGET_LEG:
    case DOG_DEBUG_TARGET_SINGLE:
    case DOG_DEBUG_TARGET_SINGLE_KNEE:
        return (leg == s_target_leg) ? 1U : 0U;
    case DOG_DEBUG_TARGET_FRONT_PAIR:
        return ((leg == DOG_LEG_LF) || (leg == DOG_LEG_RF)) ? 1U : 0U;
    case DOG_DEBUG_TARGET_REAR_PAIR:
        return ((leg == DOG_LEG_LB) || (leg == DOG_LEG_RB)) ? 1U : 0U;
    default:
        return 0U;
    }
}

static uint8_t mit_stand_set_leg_foot_xz(uint8_t leg, float x_mm, float z_mm)
{
    if (leg_is_in_debug_target(leg) == 0U) {
        return 1U;
    }

    float hip_motor = 0.0f;
    float knee_motor = 0.0f;
    if (dog_leg_foot_xz_to_motor_deg(leg, x_mm, z_mm, &hip_motor, &knee_motor) == 0U) {
        DebugUart_Printf("Stand IK FAIL leg=%s foot=(%ld,%ld)mm\r\n",
                         dog_leg_name(leg),
                         (long)x_mm,
                         (long)z_mm);
        return 0U;
    }
    dog_leg_set_motor_user_deg_for_leg(leg, hip_motor, knee_motor);
    return 1U;
}

[[maybe_unused]] static uint8_t mit_stand_set_all_legs_foot_xz(float x_mm)
{
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        if (mit_stand_set_leg_foot_xz(leg, x_mm, dog_leg_stand_foot_z_mm(leg)) == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t mit_stand_set_rise_pose(float x_mm, float progress)
{
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        if (leg_is_in_debug_target(leg) == 0U) {
            continue;
        }

        const float z_start = dog_leg_stand_foot_z_start_mm(leg);
        const float z_end = dog_leg_stand_foot_z_mm(leg);
        const float z_mm = z_start + ((z_end - z_start) * progress);
        if (mit_stand_set_leg_foot_xz(leg, x_mm, z_mm) == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static float smoothstep01(float x)
{
    x = clampf(x, 0.0f, 1.0f);
    return x * x * (3.0f - (2.0f * x));
}

static uint8_t mit_stand_set_jump_land_pose(float x_mm, float progress)
{
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        if (leg_is_in_debug_target(leg) == 0U) {
            continue;
        }

        const float z_start = DOG_JUMP_APEX_Z_MM;
        const float z_end = dog_leg_stand_foot_z_mm(leg);
        const float z_mm = z_start + ((z_end - z_start) * progress);
        if (mit_stand_set_leg_foot_xz(leg, x_mm, z_mm) == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t mit_stand_wait_target_legs_settled(uint32_t timeout_ms)
{
    const uint32_t t0 = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - t0) < timeout_ms) {
        if (dog_mit_fault_hold_is_active() != 0U) {
            return 0U;
        }

        mit_pump_control(HAL_GetTick());

        uint8_t all_settled = 1U;
        for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
            if (leg_is_in_debug_target(leg) == 0U) {
                continue;
            }
            if (march_leg_joints_settled(leg, DOG_STAND_JOINT_SETTLE_ERR_DEG) == 0U) {
                all_settled = 0U;
            }
        }

        if (all_settled != 0U) {
            return 1U;
        }
        HAL_Delay(5U);
    }

    return 1U;
}

static uint8_t mit_stand_rise_interpolate(void)
{
    if (dog_leg_foot_xz_is_reachable(DOG_STAND_FOOT_X_MM, dog_leg_stand_foot_z_start_mm(DOG_LEG_LF)) == 0U) {
        DebugUart_Printf("Stand FAIL: front start foot (%ld,%ld)mm unreachable\r\n",
                         (long)DOG_STAND_FOOT_X_MM,
                         (long)dog_leg_stand_foot_z_start_mm(DOG_LEG_LF));
        return 0U;
    }
    if (dog_leg_foot_xz_is_reachable(DOG_STAND_FOOT_X_MM, dog_leg_stand_foot_z_mm(DOG_LEG_LF)) == 0U) {
        DebugUart_Printf("Stand FAIL: front end foot (%ld,%ld)mm unreachable\r\n",
                         (long)DOG_STAND_FOOT_X_MM,
                         (long)dog_leg_stand_foot_z_mm(DOG_LEG_LF));
        return 0U;
    }
    if (dog_leg_foot_xz_is_reachable(DOG_STAND_FOOT_X_MM, dog_leg_stand_foot_z_start_mm(DOG_LEG_LB)) == 0U) {
        DebugUart_Printf("Stand FAIL: rear start foot (%ld,%ld)mm unreachable\r\n",
                         (long)DOG_STAND_FOOT_X_MM,
                         (long)dog_leg_stand_foot_z_start_mm(DOG_LEG_LB));
        return 0U;
    }
    if (dog_leg_foot_xz_is_reachable(DOG_STAND_FOOT_X_MM, dog_leg_stand_foot_z_mm(DOG_LEG_LB)) == 0U) {
        DebugUart_Printf("Stand FAIL: rear end foot (%ld,%ld)mm unreachable\r\n",
                         (long)DOG_STAND_FOOT_X_MM,
                         (long)dog_leg_stand_foot_z_mm(DOG_LEG_LB));
        return 0U;
    }

    dog_mit_reset_integrators();
    if (mit_stand_set_rise_pose(DOG_STAND_FOOT_X_MM, 0.0f) == 0U) {
        return 0U;
    }
    dog_mit_send_control_now();

    const uint32_t t0 = HAL_GetTick();

    while (1) {
        if (dog_mit_fault_hold_is_active() != 0U) {
            return 0U;
        }

        const uint32_t now = HAL_GetTick();
        float progress = 1.0f;
        if (DOG_STAND_RISE_MS > 0U) {
            progress = clampf((float)(now - t0) / (float)DOG_STAND_RISE_MS, 0.0f, 1.0f);
        }

        if (mit_stand_set_rise_pose(DOG_STAND_FOOT_X_MM, progress) == 0U) {
            return 0U;
        }
        mit_pump_control(now);

        if (progress >= 1.0f) {
            break;
        }
        HAL_Delay(1U);
    }

    return mit_stand_wait_target_legs_settled(DOG_STAND_MOVE_MS);
}

[[maybe_unused]] static uint8_t dog_leg_stand_foot_ik_motor_deg(uint8_t leg, float *hip_motor_deg, float *knee_motor_deg)
{
    return dog_leg_foot_xz_to_motor_deg(leg, DOG_STAND_FOOT_X_MM, dog_leg_stand_foot_z_mm(leg),
                                        hip_motor_deg, knee_motor_deg);
}

static uint8_t mit_stand_move_sequential(void)
{
    return mit_stand_rise_interpolate();
}

static uint8_t mit_jump_test_move(void)
{
    if (dog_leg_foot_xz_is_reachable(DOG_JUMP_FOOT_X_MM, DOG_JUMP_APEX_Z_MM) == 0U) {
        DebugUart_Printf("Jump FAIL: apex foot (%ld,%ld)mm unreachable\r\n",
                         (long)DOG_JUMP_FOOT_X_MM,
                         (long)DOG_JUMP_APEX_Z_MM);
        return 0U;
    }
    if (dog_leg_foot_xz_is_reachable(DOG_JUMP_FOOT_X_MM, dog_leg_stand_foot_z_mm(DOG_LEG_LF)) == 0U) {
        DebugUart_Printf("Jump FAIL: front land foot (%ld,%ld)mm unreachable\r\n",
                         (long)DOG_JUMP_FOOT_X_MM,
                         (long)dog_leg_stand_foot_z_mm(DOG_LEG_LF));
        return 0U;
    }
    if (dog_leg_foot_xz_is_reachable(DOG_JUMP_FOOT_X_MM, dog_leg_stand_foot_z_mm(DOG_LEG_LB)) == 0U) {
        DebugUart_Printf("Jump FAIL: rear land foot (%ld,%ld)mm unreachable\r\n",
                         (long)DOG_JUMP_FOOT_X_MM,
                         (long)dog_leg_stand_foot_z_mm(DOG_LEG_LB));
        return 0U;
    }

    dog_mit_march_in_place_stop();
    dog_mit_diag_support_stop();
    mit_set_all_stand_pid_mode();
    s_mit_pid_profile = DOG_MIT_PID_STAND;
    dog_mit_reset_integrators();
    s_jump_active = 1U;

    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        if (mit_stand_set_leg_foot_xz(leg, DOG_JUMP_FOOT_X_MM, DOG_JUMP_APEX_Z_MM) == 0U) {
            s_jump_active = 0U;
            return 0U;
        }
    }
    dog_mit_send_control_now();

    const uint32_t apex_t0 = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - apex_t0) < DOG_JUMP_APEX_HOLD_MS) {
        if (dog_mit_fault_hold_is_active() != 0U) {
            s_jump_active = 0U;
            return 0U;
        }
        mit_pump_control(HAL_GetTick());
        HAL_Delay(1U);
    }

    const uint32_t land_t0 = HAL_GetTick();

    while (1) {
        if (dog_mit_fault_hold_is_active() != 0U) {
            s_jump_active = 0U;
            return 0U;
        }

        const uint32_t now = HAL_GetTick();
        float progress = 1.0f;
        if (DOG_JUMP_LAND_MS > 0U) {
            progress = clampf((float)(now - land_t0) / (float)DOG_JUMP_LAND_MS, 0.0f, 1.0f);
        }

        if (mit_stand_set_jump_land_pose(DOG_JUMP_FOOT_X_MM, progress) == 0U) {
            s_jump_active = 0U;
            return 0U;
        }
        mit_pump_control(now);

        if (progress >= 1.0f) {
            break;
        }
        HAL_Delay(1U);
    }

    s_jump_active = 0U;
    return mit_stand_wait_target_legs_settled(DOG_JUMP_SETTLE_MS);
}

static uint8_t march_all_motors_booted(void)
{
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if (s_mit_boot_ok[i] == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t march_leg_joints_settled(uint8_t leg, float err_threshold_deg)
{
    uint8_t hip = leg_joint_index(leg, DOG_JOINT_HIP);
    uint8_t knee = leg_joint_index(leg, DOG_JOINT_KNEE);

    if ((hip >= DOG_MOTOR_COUNT) || (knee >= DOG_MOTOR_COUNT)) {
        return 0U;
    }
    if ((s_encoder_est_fresh[hip] == 0U) || (s_encoder_est_fresh[knee] == 0U)) {
        return 0U;
    }
    if (fabsf(s_target_deg[hip] - user_deg(hip)) > err_threshold_deg) {
        return 0U;
    }
    if (fabsf(s_target_deg[knee] - user_deg(knee)) > err_threshold_deg) {
        return 0U;
    }
    return 1U;
}

static uint8_t march_foot_to_motor_deg(uint8_t leg, float x_mm, float z_mm,
                                       float *hip_motor_deg, float *knee_motor_deg)
{
    return dog_leg_foot_xz_to_motor_deg(leg, x_mm, z_mm, hip_motor_deg, knee_motor_deg);
}

static void march_set_leg_foot_xz(uint8_t leg, float x_mm, float z_mm)
{
    float hip = 0.0f;
    float knee = 0.0f;
    if (march_foot_to_motor_deg(leg, x_mm, z_mm, &hip, &knee) == 0U) {
        return;
    }
    dog_leg_set_motor_user_deg_for_leg(leg, hip, knee);
}

static void march_set_all_legs_stand_pose(void)
{
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        march_set_leg_foot_xz(leg, DOG_STAND_FOOT_X_MM, dog_leg_stand_foot_z_mm(leg));
    }
}

static void dog_imu_balance_refresh_stand_hold(uint32_t now)
{
    static uint32_t s_last_refresh_ms = 0U;
    if (s_imu_balance_stand_hold_active == 0U) {
        return;
    }
    if ((uint32_t)(now - s_last_refresh_ms) < DOG_IMU_BALANCE_STAND_REFRESH_MS) {
        return;
    }
    s_last_refresh_ms = now;

    if ((s_mit_debug_active == 0U) ||
        (s_debug_target != DOG_DEBUG_TARGET_ALL) ||
        (s_march.active != 0U) ||
        (s_diag_support_active != 0U) ||
        (s_jump_active != 0U) ||
        (s_teach_hold_active != 0U) ||
        (s_mit_torque_test_active != 0U) ||
        (s_mit_fault_hold_active != 0U)) {
        return;
    }

    mit_set_all_stand_pid_mode();
    march_set_all_legs_stand_pose();
}

static void march_get_swing_legs(uint8_t *leg_a, uint8_t *leg_b)
{
    if (march_mode_uses_cycloid(s_march.mode) != 0U) {
        *leg_a = s_trot_swing_pairs[s_march.leg][0U];
        *leg_b = s_trot_swing_pairs[s_march.leg][1U];
        return;
    }

    *leg_a = s_march.leg;
    *leg_b = DOG_LEG_COUNT;
}

static uint8_t march_trot_leg_corners(uint8_t leg, float *x1, float *x2, float *z1, float *z2);

static uint8_t march_gait_corners_reachable(void)
{
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        float x1 = 0.0f;
        float x2 = 0.0f;
        float z1 = 0.0f;
        float z2 = 0.0f;
        if (march_trot_leg_corners(leg, &x1, &x2, &z1, &z2) == 0U) {
            DebugUart_Printf("Gait FAIL: %s corner setup failed.\r\n", dog_leg_name(leg));
            return 0U;
        }

        if (dog_leg_foot_xz_is_reachable(x1, z2) == 0U) {
            DebugUart_Printf("Gait FAIL: %s unreachable swing peak (%.1f, %.1f) mm.\r\n",
                             dog_leg_name(leg), (double)x1, (double)z2);
            return 0U;
        }
        if (dog_leg_foot_xz_is_reachable(x2, z2) == 0U) {
            DebugUart_Printf("Gait FAIL: %s unreachable swing peak (%.1f, %.1f) mm.\r\n",
                             dog_leg_name(leg), (double)x2, (double)z2);
            return 0U;
        }
        if (dog_leg_foot_xz_is_reachable(x1, z1) == 0U) {
            DebugUart_Printf("Gait FAIL: %s unreachable touch-down (%.1f, %.1f) mm.\r\n",
                             dog_leg_name(leg), (double)x1, (double)z1);
            return 0U;
        }
        if (dog_leg_foot_xz_is_reachable(x2, z1) == 0U) {
            DebugUart_Printf("Gait FAIL: %s unreachable touch-down (%.1f, %.1f) mm.\r\n",
                             dog_leg_name(leg), (double)x2, (double)z1);
            return 0U;
        }
    }
    return 1U;
}

static uint8_t march_gait_stride_reachable(uint8_t mode)
{
    const uint8_t prev_mode = s_march.mode;
    s_march.mode = mode;
    const uint8_t ok = march_gait_corners_reachable();
    s_march.mode = prev_mode;
    return ok;
}

static uint8_t march_trot_leg_corners(uint8_t leg, float *x1, float *x2, float *z1, float *z2)
{
    const float base_x = s_leg_foot_x_offset[leg];
    const float x_delta = march_stride_x_delta(leg);
    if (x1 != nullptr) {
        *x1 = base_x + (DOG_TROT_FOOT_X1_MM * leg_ik_x_sign(leg) * DOG_TROT_FORWARD_X_SIGN);
    }
    if (x2 != nullptr) {
        *x2 = base_x + x_delta;
    }
    if (z1 != nullptr) {
        *z1 = dog_leg_stand_foot_z_mm(leg) + DOG_TROT_FOOT_Z1_MM;
    }
    if (z2 != nullptr) {
        *z2 = march_trot_swing_peak_z_mm(leg);
    }
    return 1U;
}

static void march_trot_set_swing_foot(uint8_t leg, float x_mm, float z_mm)
{
    march_set_leg_foot_xz(leg, x_mm, z_mm);
}

struct March_Trot_Traj_Params {
    float step_length;
    float step_height;
    float period_s;
    float start_x;
    float start_z;
};

static float march_trot_elapsed_s(uint32_t now, float *period_s_out)
{
    const float period_s = (float)dog_mit_gait_trot_swing_ms() * 0.001f;
    if (period_s_out != nullptr) {
        *period_s_out = period_s;
    }

    float t_s = (float)(now - s_march.swing_t0_ms) * 0.001f;
    if (t_s < 0.0f) {
        t_s = 0.0f;
    } else if (t_s > period_s) {
        t_s = period_s;
    }
    return t_s;
}

static void march_trot_trajectory1_cycloid(float t_s, const March_Trot_Traj_Params *params,
                                           float *x_mm, float *z_mm)
{
    if ((params == nullptr) || (x_mm == nullptr) || (z_mm == nullptr)) {
        return;
    }

    if (t_s <= 0.0f) {
        *x_mm = params->start_x;
        *z_mm = params->start_z;
        return;
    }

    if (t_s >= params->period_s) {
        *x_mm = params->start_x + params->step_length;
        *z_mm = params->start_z;
        return;
    }

    const float tau = t_s / params->period_s;
    const float two_pi_tau = 2.0f * DOG_PI * tau;
    *x_mm = params->start_x +
            (params->step_length * (tau - (sinf(two_pi_tau) / (2.0f * DOG_PI))));
    *z_mm = params->start_z + (params->step_height * (1.0f - cosf(two_pi_tau)) * 0.5f);
}

static uint8_t march_is_swing_leg(uint8_t leg, uint8_t swing_a, uint8_t swing_b)
{
    return ((leg == swing_a) || (leg == swing_b)) ? 1U : 0U;
}

static void march_trot_snap_swing_legs_start(uint8_t leg_a, uint8_t leg_b)
{
    const uint8_t swing_legs[2U] = {leg_a, leg_b};

    for (uint8_t i = 0U; i < 2U; ++i) {
        const uint8_t leg = swing_legs[i];
        float x1 = 0.0f;
        float z1 = 0.0f;

        if (leg >= DOG_LEG_COUNT) {
            continue;
        }
        if (march_trot_leg_corners(leg, &x1, nullptr, &z1, nullptr) == 0U) {
            continue;
        }
        march_trot_set_swing_foot(leg, x1, z1);
    }
}

static void march_trot_apply_swing_trajectory(uint32_t now)
{
    if (march_mode_uses_cycloid(s_march.mode) == 0U) {
        return;
    }

    uint8_t leg_a = 0U;
    uint8_t leg_b = DOG_LEG_COUNT;
    march_get_swing_legs(&leg_a, &leg_b);

    const uint8_t swing_legs[2U] = {leg_a, leg_b};
    float period_s = 0.0f;
    const float t_s = march_trot_elapsed_s(now, &period_s);

    for (uint8_t i = 0U; i < 2U; ++i) {
        const uint8_t leg = swing_legs[i];
        float x1 = 0.0f;
        float x2 = 0.0f;
        float z1 = 0.0f;
        float z2 = 0.0f;

        if (leg >= DOG_LEG_COUNT) {
            continue;
        }
        if (march_trot_leg_corners(leg, &x1, &x2, &z1, &z2) == 0U) {
            continue;
        }

        March_Trot_Traj_Params params = {
            x2 - x1,
            z2 - z1,
            period_s,
            x1,
            z1,
        };
        float foot_x = x1;
        float foot_z = z1;
        march_trot_trajectory1_cycloid(t_s, &params, &foot_x, &foot_z);
        march_trot_set_swing_foot(leg, foot_x, foot_z);
    }
}

static void march_trot_apply_support_stance(uint32_t now)
{
    if (march_mode_uses_cycloid(s_march.mode) == 0U) {
        return;
    }

    uint8_t swing_a = 0U;
    uint8_t swing_b = DOG_LEG_COUNT;
    march_get_swing_legs(&swing_a, &swing_b);

    float period_s = 0.0f;
    const float t_s = march_trot_elapsed_s(now, &period_s);

    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        if (march_is_swing_leg(leg, swing_a, swing_b) != 0U) {
            continue;
        }

        float x1 = 0.0f;
        float z1 = 0.0f;
        if (march_trot_leg_corners(leg, &x1, nullptr, &z1, nullptr) == 0U) {
            continue;
        }

        const float retract = march_stride_x_delta(leg);
        const float support_step_x = (march_mode_is_turn(s_march.mode) != 0U) ? 0.0f : (-retract);
        March_Trot_Traj_Params params = {
            support_step_x,
            0.0f,
            period_s,
            x1,
            z1,
        };
        float foot_x = x1;
        float foot_z = z1;
        march_trot_trajectory1_cycloid(t_s, &params, &foot_x, &foot_z);
        march_set_leg_foot_xz(leg, foot_x, foot_z);
    }
}

static void march_trot_finish_stride(uint32_t now)
{
    if (march_mode_uses_cycloid(s_march.mode) == 0U) {
        return;
    }
    if (s_march.trot_stride_applied != 0U) {
        return;
    }

    float period_s = 0.0f;
    const float t_s = march_trot_elapsed_s(now, &period_s);
    if (t_s < period_s) {
        return;
    }

    uint8_t swing_a = 0U;
    uint8_t swing_b = DOG_LEG_COUNT;
    march_get_swing_legs(&swing_a, &swing_b);

    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        const float delta = march_stride_x_delta(leg);
        if (march_is_swing_leg(leg, swing_a, swing_b) != 0U) {
            s_leg_foot_x_offset[leg] += delta;
        } else {
            s_leg_foot_x_offset[leg] -= delta;
        }
    }
    s_march.trot_stride_applied = 1U;
}

static void march_trot_support_leg(uint8_t leg)
{
    float x1 = 0.0f;
    float z1 = 0.0f;

    if (march_trot_leg_corners(leg, &x1, nullptr, &z1, nullptr) != 0U) {
        march_set_leg_foot_xz(leg, x1, z1);
    }
}

static void march_refresh_support_legs(uint8_t swing_a, uint8_t swing_b)
{
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        if (march_is_swing_leg(leg, swing_a, swing_b) != 0U) {
            continue;
        }
        march_trot_support_leg(leg);
    }
}

static uint8_t march_compute_lift_pose(uint8_t leg, float *hip_motor_deg, float *knee_motor_deg)
{
    const float lift_z = DOG_STAND_FOOT_Z_MM + DOG_MARCH_LIFT_Z_MM;
    return dog_leg_foot_xz_to_motor_deg(leg, DOG_STAND_FOOT_X_MM, lift_z, hip_motor_deg, knee_motor_deg);
}

static uint8_t march_swing_legs_settled(float err_threshold_deg)
{
    uint8_t leg_a = 0U;
    uint8_t leg_b = DOG_LEG_COUNT;
    march_get_swing_legs(&leg_a, &leg_b);

    if (march_leg_joints_settled(leg_a, err_threshold_deg) == 0U) {
        return 0U;
    }
    if (leg_b < DOG_LEG_COUNT) {
        return march_leg_joints_settled(leg_b, err_threshold_deg);
    }
    return 1U;
}

static void march_begin_swing_up(uint32_t now)
{
    uint8_t leg_a = 0U;
    uint8_t leg_b = DOG_LEG_COUNT;
    march_get_swing_legs(&leg_a, &leg_b);

    if (march_mode_uses_cycloid(s_march.mode) != 0U) {
        march_refresh_support_legs(leg_a, leg_b);
        mit_set_mixed_swing_legs(leg_a, leg_b);
        s_march.swing_t0_ms = now;
        s_march.trot_stride_applied = 0U;
        march_trot_snap_swing_legs_start(leg_a, leg_b);
        march_trot_apply_swing_trajectory(now);
    } else {
        mit_set_mixed_swing_legs(leg_a, leg_b);
        if (march_compute_lift_pose(leg_a, &s_march.lift_hip_deg, &s_march.lift_knee_deg) != 0U) {
            dog_leg_set_motor_user_deg_for_leg(leg_a, s_march.lift_hip_deg, s_march.lift_knee_deg);
        }
        if (leg_b < DOG_LEG_COUNT) {
            float lift_hip_b = 0.0f;
            float lift_knee_b = 0.0f;
            if (march_compute_lift_pose(leg_b, &lift_hip_b, &lift_knee_b) != 0U) {
                dog_leg_set_motor_user_deg_for_leg(leg_b, lift_hip_b, lift_knee_b);
            }
        }
    }

    s_march.phase = DOG_MARCH_PHASE_SWING_UP;
    s_march.phase_t0_ms = now;
}

static void march_begin_swing_down(uint32_t now)
{
    uint8_t leg_a = 0U;
    uint8_t leg_b = DOG_LEG_COUNT;
    march_get_swing_legs(&leg_a, &leg_b);

    if (march_mode_uses_cycloid(s_march.mode) != 0U) {
        return;
    }

    mit_set_all_stand_pid_mode();
    march_set_leg_foot_xz(leg_a, DOG_STAND_FOOT_X_MM, dog_leg_stand_foot_z_mm(leg_a));
    if (leg_b < DOG_LEG_COUNT) {
        march_set_leg_foot_xz(leg_b, DOG_STAND_FOOT_X_MM, dog_leg_stand_foot_z_mm(leg_b));
    }

    s_march.phase = DOG_MARCH_PHASE_SWING_DOWN;
    s_march.phase_t0_ms = now;
}

void dog_mit_march_in_place_stop(void)
{
    s_trot_direction_sign = 1.0f;
    if (s_march.active == 0U) {
        return;
    }

    memset(&s_march, 0, sizeof(s_march));
    memset(s_leg_foot_x_offset, 0, sizeof(s_leg_foot_x_offset));
    memset(s_leg_hip_offset_deg, 0, sizeof(s_leg_hip_offset_deg));
    mit_set_all_stand_pid_mode();
    march_set_all_legs_stand_pose();
    dog_mit_reset_integrators();
    s_imu_balance_stand_hold_active = ((s_debug_target == DOG_DEBUG_TARGET_ALL) &&
                                       (s_mit_debug_active != 0U)) ? 1U : 0U;
}

static void march_advance_step(uint32_t now)
{
    if (march_mode_uses_cycloid(s_march.mode) != 0U) {
        s_march.leg = (uint8_t)((s_march.leg + 1U) % 2U);
    } else {
        s_march.leg = (uint8_t)((s_march.leg + 1U) % DOG_LEG_COUNT);
    }

    if (s_march.leg == 0U) {
        if (s_march.cycles_remaining > 0U) {
            s_march.cycles_remaining--;
            if (s_march.cycles_remaining == 0U) {
                dog_mit_march_in_place_stop();
                DebugUart_Printf("March done.\r\n");
                return;
            }
        }
    }

    march_begin_swing_up(now);
}

static void march_in_place_tick(uint32_t now)
{
    if (s_march.active == 0U) {
        return;
    }
    if ((dog_mit_debug_is_active() == 0U) || (dog_mit_fault_hold_is_active() != 0U)) {
        dog_mit_march_in_place_stop();
        dog_mit_diag_support_stop();
        return;
    }

    const uint8_t cycloid_gait = march_mode_uses_cycloid(s_march.mode);

    if ((cycloid_gait != 0U) && (s_march.phase == DOG_MARCH_PHASE_SWING_UP)) {
        march_trot_apply_swing_trajectory(now);
        march_trot_apply_support_stance(now);
        march_trot_finish_stride(now);
    }

    switch (s_march.phase) {
    case DOG_MARCH_PHASE_SWING_UP:
        if (cycloid_gait != 0U) {
            const uint32_t elapsed_ms = (uint32_t)(now - s_march.swing_t0_ms);
            const uint32_t swing_ms = dog_mit_gait_trot_swing_ms();
            if (elapsed_ms >= swing_ms) {
                const uint8_t settled = march_swing_legs_settled(DOG_TROT_SETTLE_ERR_DEG);
                const uint8_t timeout = (elapsed_ms >= (swing_ms + DOG_TROT_SETTLE_EXTRA_MS)) ? 1U : 0U;
                if ((settled != 0U) || (timeout != 0U)) {
                    if ((settled == 0U) && (timeout != 0U)) {
                        DebugUart_Printf("Trot settle timeout pair=%u err>%lddeg\r\n",
                                         (unsigned)s_march.leg,
                                         (long)DOG_TROT_SETTLE_ERR_DEG);
                    }
                    s_march.phase = DOG_MARCH_PHASE_PAUSE;
                    s_march.phase_t0_ms = now;
                }
            }
        } else if (march_swing_legs_settled(DOG_MARCH_SETTLE_ERR_DEG) != 0U) {
            s_march.phase = DOG_MARCH_PHASE_HOLD;
            s_march.phase_t0_ms = now;
        } else if ((uint32_t)(now - s_march.phase_t0_ms) >= DOG_MARCH_SETTLE_TIMEOUT_MS) {
            DebugUart_Printf("March timeout swing-up leg=%s\r\n", dog_leg_name(s_march.leg));
            s_march.phase = DOG_MARCH_PHASE_HOLD;
            s_march.phase_t0_ms = now;
        }
        break;

    case DOG_MARCH_PHASE_HOLD:
        if ((uint32_t)(now - s_march.phase_t0_ms) >= ((cycloid_gait != 0U) ? DOG_TROT_HOLD_MS : DOG_MARCH_HOLD_MS)) {
            march_begin_swing_down(now);
        }
        break;

    case DOG_MARCH_PHASE_SWING_DOWN:
        if (cycloid_gait != 0U) {
            if ((uint32_t)(now - s_march.phase_t0_ms) >= DOG_TROT_SWING_DOWN_MS) {
                s_march.phase = DOG_MARCH_PHASE_PAUSE;
                s_march.phase_t0_ms = now;
            }
        } else if (march_swing_legs_settled(DOG_MARCH_SETTLE_ERR_DEG) != 0U) {
            s_march.phase = DOG_MARCH_PHASE_PAUSE;
            s_march.phase_t0_ms = now;
        } else if ((uint32_t)(now - s_march.phase_t0_ms) >= DOG_MARCH_SETTLE_TIMEOUT_MS) {
            DebugUart_Printf("March timeout swing-down leg=%s\r\n", dog_leg_name(s_march.leg));
            s_march.phase = DOG_MARCH_PHASE_PAUSE;
            s_march.phase_t0_ms = now;
        }
        break;

    case DOG_MARCH_PHASE_PAUSE:
        if ((uint32_t)(now - s_march.phase_t0_ms) >=
            ((cycloid_gait != 0U) ? DOG_TROT_PAIR_PAUSE_MS : DOG_MARCH_LEG_PAUSE_MS)) {
            march_advance_step(now);
        }
        break;

    default:
        break;
    }
}

uint8_t dog_mit_march_in_place_is_active(void)
{
    return s_march.active;
}

uint8_t dog_mit_trot_march_is_active(void)
{
    return ((s_march.active != 0U) && (s_march.mode == DOG_MARCH_MODE_TROT)) ? 1U : 0U;
}

uint8_t dog_mit_turn_march_is_active(void)
{
    return ((s_march.active != 0U) &&
            ((s_march.mode == DOG_MARCH_MODE_TURN_LEFT) ||
             (s_march.mode == DOG_MARCH_MODE_TURN_RIGHT))) ? 1U : 0U;
}

static uint8_t march_in_place_start_mode(uint8_t mode, uint8_t cycles, float trot_direction_sign)
{
    if (dog_mit_debug_is_active() == 0U) {
        DebugUart_Printf("March FAIL: send '8' then 's' first.\r\n");
        return 0U;
    }
    if (dog_mit_fault_hold_is_active() != 0U) {
        DebugUart_Printf("March FAIL: fault-hold active.\r\n");
        return 0U;
    }
    if (s_debug_target != DOG_DEBUG_TARGET_ALL) {
        DebugUart_Printf("March FAIL: send '8' to target all 8 motors.\r\n");
        return 0U;
    }
    if (march_all_motors_booted() == 0U) {
        DebugUart_Printf("March FAIL: need all 8 online/booted.\r\n");
        return 0U;
    }
    s_trot_direction_sign = ((mode == DOG_MARCH_MODE_TROT) && (trot_direction_sign < 0.0f)) ? -1.0f : 1.0f;
    if ((march_mode_uses_cycloid(mode) != 0U) && (march_gait_stride_reachable(mode) == 0U)) {
        return 0U;
    }

    dog_mit_march_in_place_stop();
    dog_mit_diag_support_stop();
    s_imu_balance_stand_hold_active = 0U;
    s_trot_direction_sign = ((mode == DOG_MARCH_MODE_TROT) && (trot_direction_sign < 0.0f)) ? -1.0f : 1.0f;
    mit_set_all_stand_pid_mode();
    march_set_all_legs_stand_pose();
    dog_mit_reset_integrators();

    memset(&s_march, 0, sizeof(s_march));
    memset(s_leg_foot_x_offset, 0, sizeof(s_leg_foot_x_offset));
    memset(s_leg_hip_offset_deg, 0, sizeof(s_leg_hip_offset_deg));
    s_march.active = 1U;
    s_march.mode = mode;
    s_march.cycles_remaining = cycles;
    march_begin_swing_up(HAL_GetTick());

    if (mode == DOG_MARCH_MODE_TROT) {
        DebugUart_Printf("Trot%s cycloid: speed=%s hz=%ld.%01ld step=%ldmm height=%ldmm swing=%lums LF+RB<->RF+LB, x=stop\r\n",
                         (s_trot_direction_sign < 0.0f) ? " reverse" : "",
                         dog_mit_gait_speed_profile_name(),
                         (long)dog_mit_gait_trot_hz(),
                         (long)(dog_mit_gait_trot_hz() * 10.0f) % 10L,
                         (long)dog_mit_gait_forward_stride_x_mm(),
                         (long)DOG_FORWARD_SWING_LIFT_Z_MM,
                         (long)dog_mit_gait_trot_swing_ms());
    } else if (mode == DOG_MARCH_MODE_TURN_LEFT) {
        DebugUart_Printf("TurnL cycloid: speed=%s hz=%ld.%01ld stride=%ldmm diag opposite LF+RB<->RF+LB, x=stop\r\n",
                         dog_mit_gait_speed_profile_name(),
                         (long)dog_mit_gait_trot_hz(),
                         (long)(dog_mit_gait_trot_hz() * 10.0f) % 10L,
                         (long)dog_mit_gait_turn_stride_x_mm());
    } else if (mode == DOG_MARCH_MODE_TURN_RIGHT) {
        DebugUart_Printf("TurnR cycloid: speed=%s hz=%ld.%01ld stride=%ldmm diag opposite LF+RB<->RF+LB, x=stop\r\n",
                         dog_mit_gait_speed_profile_name(),
                         (long)dog_mit_gait_trot_hz(),
                         (long)(dog_mit_gait_trot_hz() * 10.0f) % 10L,
                         (long)dog_mit_gait_turn_stride_x_mm());
    } else {
        DebugUart_Printf("March start: lift=%ldmm LF->RF->LB->RB, support STAND swing SWING, x=stop\r\n",
                         (long)DOG_MARCH_LIFT_Z_MM);
    }
    return 1U;
}

uint8_t dog_mit_march_in_place_start(uint8_t cycles)
{
    return march_in_place_start_mode(DOG_MARCH_MODE_WALK, cycles, 1.0f);
}

uint8_t dog_mit_trot_in_place_start(uint8_t cycles)
{
    return march_in_place_start_mode(DOG_MARCH_MODE_TROT, cycles, 1.0f);
}

uint8_t dog_mit_trot_reverse_in_place_start(uint8_t cycles)
{
    return march_in_place_start_mode(DOG_MARCH_MODE_TROT, cycles, -1.0f);
}

uint8_t dog_mit_turn_left_in_place_start(uint8_t cycles)
{
    return march_in_place_start_mode(DOG_MARCH_MODE_TURN_LEFT, cycles, 1.0f);
}

uint8_t dog_mit_turn_right_in_place_start(uint8_t cycles)
{
    return march_in_place_start_mode(DOG_MARCH_MODE_TURN_RIGHT, cycles, 1.0f);
}

static void diag_support_set_lift_leg(uint8_t leg)
{
    const float lift_z = DOG_STAND_FOOT_Z_MM + DOG_DIAG_SUPPORT_LIFT_Z_MM;
    march_set_leg_foot_xz(leg, DOG_STAND_FOOT_X_MM, lift_z);
}

void dog_mit_diag_support_stop(void)
{
    if (s_diag_support_active == 0U) {
        return;
    }

    s_diag_support_active = 0U;
    mit_set_all_stand_pid_mode();
    march_set_all_legs_stand_pose();
    dog_mit_reset_integrators();
}

uint8_t dog_mit_diag_support_is_active(void)
{
    return s_diag_support_active;
}

uint8_t dog_mit_diag_support_lf_rb_start(void)
{
    if (dog_mit_debug_is_active() == 0U) {
        return 0U;
    }
    if (dog_mit_fault_hold_is_active() != 0U) {
        return 0U;
    }
    if (s_debug_target != DOG_DEBUG_TARGET_ALL) {
        return 0U;
    }
    if (march_all_motors_booted() == 0U) {
        return 0U;
    }

    dog_mit_march_in_place_stop();
    dog_mit_diag_support_stop();
    s_imu_balance_stand_hold_active = 0U;

    march_set_leg_foot_xz(DOG_LEG_LF, DOG_STAND_FOOT_X_MM, dog_leg_stand_foot_z_mm(DOG_LEG_LF));
    march_set_leg_foot_xz(DOG_LEG_RB, DOG_STAND_FOOT_X_MM, dog_leg_stand_foot_z_mm(DOG_LEG_RB));
    mit_set_mixed_swing_legs(DOG_LEG_RF, DOG_LEG_LB);
    diag_support_set_lift_leg(DOG_LEG_RF);
    diag_support_set_lift_leg(DOG_LEG_LB);
    dog_mit_reset_integrators();
    s_diag_support_active = 1U;

    DebugUart_Printf("Diag LF+RB support STAND, RF+LB lift %ldmm SWING. D/x=stop, 1s log\r\n",
                     (long)DOG_DIAG_SUPPORT_LIFT_Z_MM);
    dog_diag_support_print_status();
    return 1U;
}

static long diag_current_ma(float current_a)
{
    return (long)(current_a * 1000.0f);
}

static long diag_abs_ma(long current_ma)
{
    return (current_ma < 0L) ? -current_ma : current_ma;
}

void dog_diag_support_print_status(void)
{
    const uint8_t support_legs[2U] = {DOG_LEG_LF, DOG_LEG_RB};
    long sum_cmd = 0L;
    long sum_iq = 0L;
    long sum_abs_cmd = 0L;
    long sum_abs_iq = 0L;

    DebugUart_Printf("DIAG ");
    for (uint8_t i = 0U; i < 2U; ++i) {
        uint8_t leg = support_legs[i];
        uint8_t hip = leg_joint_index(leg, DOG_JOINT_HIP);
        uint8_t knee = leg_joint_index(leg, DOG_JOINT_KNEE);
        long cmd_hip = (hip < DOG_MOTOR_COUNT) ? diag_current_ma(s_mit_cmd_current_a[hip]) : 0L;
        long cmd_knee = (knee < DOG_MOTOR_COUNT) ? diag_current_ma(s_mit_cmd_current_a[knee]) : 0L;
        long iq_hip = (hip < DOG_MOTOR_COUNT) ? diag_current_ma(g_mw_motor_data[hip].iq.iqMeasured) : 0L;
        long iq_knee = (knee < DOG_MOTOR_COUNT) ? diag_current_ma(g_mw_motor_data[knee].iq.iqMeasured) : 0L;

        sum_cmd += cmd_hip + cmd_knee;
        sum_iq += iq_hip + iq_knee;
        sum_abs_cmd += diag_abs_ma(cmd_hip) + diag_abs_ma(cmd_knee);
        sum_abs_iq += diag_abs_ma(iq_hip) + diag_abs_ma(iq_knee);

        DebugUart_Printf("%s cmdI=(%ld,%ld) iq=(%ld,%ld) ",
                         dog_leg_name(leg),
                         cmd_hip,
                         cmd_knee,
                         iq_hip,
                         iq_knee);
    }

    DebugUart_Printf("sum_cmdI=%ld sum_iq=%ld abs_cmdI=%ld abs_iq=%ld\r\n",
                     sum_cmd,
                     sum_iq,
                     sum_abs_cmd,
                     sum_abs_iq);
}

void dog_mit_print_all_motor_current(const char *tag)
{
    if (tag != nullptr) {
        DebugUart_Printf("%s", tag);
    }
    DebugUart_Printf("cmdI(mA):");
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        DebugUart_Printf(" M%u=%ld", (unsigned)i, diag_current_ma(s_mit_cmd_current_a[i]));
    }
    DebugUart_Printf(" iq(mA):");
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        DebugUart_Printf(" M%u=%ld", (unsigned)i, diag_current_ma(g_mw_motor_data[i].iq.iqMeasured));
    }
    DebugUart_Printf("\r\n");
}

uint8_t dog_mit_goto_foot_xz(float x_mm, float z_mm)
{
    s_imu_balance_stand_hold_active = 0U;
    if (dog_mit_debug_is_active() == 0U) {
        return 0U;
    }
    if (dog_mit_fault_hold_is_active() != 0U) {
        return 0U;
    }

    const uint8_t leg = DOG_FOOT_GOTO_LEG;
    float hip_motor = 0.0f;
    float knee_motor = 0.0f;
    if (dog_leg_foot_xz_to_motor_deg(leg, x_mm, z_mm, &hip_motor, &knee_motor) == 0U) {
        return 0U;
    }

    s_mit_pid_profile = DOG_MIT_PID_SWING;
    mit_clear_mixed_pid();
    dog_mit_reset_integrators();
    return mit_move_motor_pose_sequential_for_leg(leg, hip_motor, knee_motor);
}

uint8_t dog_mit_stand_sequence(void)
{
    if (dog_mit_fault_hold_is_active() != 0U) {
        return 0U;
    }

    uint8_t ok_count = dog_debug_mit_boot_sequence();
    if (ok_count == 0U) {
        return 0U;
    }

    s_mit_pid_profile = DOG_MIT_PID_STAND;
    dog_mit_reset_integrators();
    DebugUart_Printf("Stand STAND_PID foot IK front (%ld,%ld)->(%ld,%ld) rear (%ld,%ld)->(%ld,%ld)mm %lums Ol=%ldmA\r\n",
                     (long)DOG_STAND_FOOT_X_MM,
                     (long)DOG_STAND_FOOT_Z_START_MM,
                     (long)DOG_STAND_FOOT_X_MM,
                     (long)DOG_STAND_FOOT_Z_MM,
                     (long)DOG_STAND_FOOT_X_MM,
                     (long)dog_leg_stand_foot_z_start_mm(DOG_LEG_LB),
                     (long)DOG_STAND_FOOT_X_MM,
                     (long)dog_leg_stand_foot_z_mm(DOG_LEG_LB),
                     (long)DOG_STAND_RISE_MS,
                     (long)(g_dog_mit_stand_pid.output_limit_a * 1000.0f));

    if (mit_stand_move_sequential() == 0U) {
        s_imu_balance_stand_hold_active = 0U;
        return 0U;
    }

    s_imu_balance_stand_hold_active = (s_debug_target == DOG_DEBUG_TARGET_ALL) ? 1U : 0U;
    return ok_count;
}

uint8_t dog_mit_return_to_stand_start_pose(void)
{
    if (dog_mit_debug_is_active() == 0U) {
        DebugUart_Printf("Return start pose FAIL: MIT inactive.\r\n");
        return 0U;
    }
    if (dog_mit_fault_hold_is_active() != 0U) {
        DebugUart_Printf("Return start pose FAIL: fault-hold active.\r\n");
        return 0U;
    }
    if (s_debug_target != DOG_DEBUG_TARGET_ALL) {
        DebugUart_Printf("Return start pose FAIL: target all motors first.\r\n");
        return 0U;
    }
    if (dog_leg_foot_xz_is_reachable(DOG_STAND_FOOT_X_MM, dog_leg_stand_foot_z_start_mm(DOG_LEG_LF)) == 0U) {
        DebugUart_Printf("Return start pose FAIL: front start foot (%ld,%ld)mm unreachable\r\n",
                         (long)DOG_STAND_FOOT_X_MM,
                         (long)dog_leg_stand_foot_z_start_mm(DOG_LEG_LF));
        return 0U;
    }
    if (dog_leg_foot_xz_is_reachable(DOG_STAND_FOOT_X_MM, dog_leg_stand_foot_z_start_mm(DOG_LEG_LB)) == 0U) {
        DebugUart_Printf("Return start pose FAIL: rear start foot (%ld,%ld)mm unreachable\r\n",
                         (long)DOG_STAND_FOOT_X_MM,
                         (long)dog_leg_stand_foot_z_start_mm(DOG_LEG_LB));
        return 0U;
    }

    dog_mit_march_in_place_stop();
    mit_set_all_stand_pid_mode();
    dog_mit_reset_integrators();

    const uint32_t t0 = HAL_GetTick();
    while (1) {
        if (dog_mit_fault_hold_is_active() != 0U) {
            return 0U;
        }

        const uint32_t now = HAL_GetTick();
        float elapsed = 1.0f;
        if (DOG_STAND_RISE_MS > 0U) {
            elapsed = (float)(now - t0) / (float)DOG_STAND_RISE_MS;
        }
        const float progress = 1.0f - smoothstep01(elapsed);
        if (mit_stand_set_rise_pose(DOG_STAND_FOOT_X_MM, progress) == 0U) {
            return 0U;
        }
        mit_pump_control(now);

        if (elapsed >= 1.0f) {
            break;
        }
        HAL_Delay(1U);
    }

    if (mit_stand_set_rise_pose(DOG_STAND_FOOT_X_MM, 0.0f) == 0U) {
        return 0U;
    }
    dog_mit_send_control_now();
    dog_debug_rx_only();
    return 1U;
}

uint8_t dog_mit_jump_test_sequence(void)
{
    if (dog_mit_debug_is_active() == 0U) {
        DebugUart_Printf("Jump FAIL: send '8' then 's' first.\r\n");
        return 0U;
    }
    if (dog_mit_fault_hold_is_active() != 0U) {
        DebugUart_Printf("Jump FAIL: fault-hold active.\r\n");
        return 0U;
    }
    if (s_debug_target != DOG_DEBUG_TARGET_ALL) {
        DebugUart_Printf("Jump FAIL: send '8' to target all 8 motors.\r\n");
        return 0U;
    }
    s_imu_balance_stand_hold_active = 0U;

    DebugUart_Printf("Jump STAND_PID snap (%ld,%ld)->(%ld,%ld) land front=%ld rear=%ld hold=%ums land=%ums\r\n",
                     (long)DOG_JUMP_FOOT_X_MM,
                     (long)DOG_JUMP_LAND_Z_MM,
                     (long)DOG_JUMP_FOOT_X_MM,
                     (long)DOG_JUMP_APEX_Z_MM,
                     (long)dog_leg_stand_foot_z_mm(DOG_LEG_LF),
                     (long)dog_leg_stand_foot_z_mm(DOG_LEG_LB),
                     (unsigned)DOG_JUMP_APEX_HOLD_MS,
                     (unsigned)DOG_JUMP_LAND_MS);

    if (mit_jump_test_move() == 0U) {
        return 0U;
    }

    DebugUart_Printf("Jump done, foot IK=(%ld,%ld)mm\r\n",
                     (long)DOG_JUMP_FOOT_X_MM,
                     (long)DOG_JUMP_LAND_Z_MM);
    return 1U;
}

uint8_t dog_mit_get_pid_telemetry(uint8_t motor_index, Dog_Mit_Pid_Telemetry *telemetry)
{
    if ((motor_index >= DOG_MOTOR_COUNT) || (telemetry == nullptr)) {
        return 0U;
    }

    telemetry->target_deg = s_target_deg[motor_index];
    telemetry->user_deg = user_deg(motor_index);
    telemetry->err_deg = telemetry->target_deg - telemetry->user_deg;
    telemetry->cmd_a = s_mit_cmd_current_a[motor_index];
    telemetry->p_a = s_mit_pid_p_a[motor_index];
    telemetry->i_a = s_mit_pid_i_a[motor_index];
    telemetry->d_a = s_mit_pid_d_a[motor_index];
    return 1U;
}

uint8_t dog_mit_get_default_vofa_motor_index(void)
{
    uint8_t joint = (s_debug_target == DOG_DEBUG_TARGET_SINGLE_KNEE) ? DOG_JOINT_KNEE : DOG_JOINT_HIP;
    uint8_t idx = leg_joint_index(s_target_leg, joint);
    return (idx < DOG_MOTOR_COUNT) ? idx : 0U;
}

uint8_t dog_leg_target_leg(void) { return s_target_leg; }
uint8_t dog_debug_target(void) { return s_debug_target; }
uint8_t dog_debug_target_count(void) { return selected_count(); }
uint8_t dog_leg_target_expected_mask(void) { return (uint8_t)((1U << selected_count()) - 1U); }

void dog_debug_set_target(uint8_t target)
{
    dog_debug_rx_only();
    s_debug_target = (target <= DOG_DEBUG_TARGET_SINGLE_KNEE) ? target : DOG_DEBUG_TARGET_ALL;
    if (s_debug_target == DOG_DEBUG_TARGET_FRONT_PAIR) {
        s_target_leg = DOG_LEG_LF;
    } else if (s_debug_target == DOG_DEBUG_TARGET_REAR_PAIR) {
        s_target_leg = DOG_LEG_LB;
    }
}

void dog_debug_set_target_leg(uint8_t leg)
{
    dog_debug_rx_only();
    if (leg < DOG_LEG_COUNT) {
        s_target_leg = leg;
    }
}

void dog_debug_mit_torque_stop(void)
{
    if ((s_mit_torque_test_active != 0U) && (s_mit_torque_test_index < DOG_MOTOR_COUNT)) {
        send_mit_zero_effort(s_mit_torque_test_index, 0.0f);
    }
    s_mit_torque_test_active = 0U;
    s_mit_torque_test_index = DOG_MOTOR_COUNT;
    s_mit_torque_test_nm = 0.0f;
}

uint8_t dog_debug_mit_torque_test(uint8_t bus, uint8_t node_id, float torque_nm)
{
    uint8_t idx = motor_index(bus, node_id);
    if (idx >= DOG_MOTOR_COUNT) {
        return 0U;
    }

    dog_debug_mit_torque_stop();
    s_mit_debug_active = 0U;
    teach_hold_stop();
    s_position_tx_enabled = 0U;
    memset(s_motor_mit_probe_active, 0, sizeof(s_motor_mit_probe_active));

    Dog_Motor_Config *cfg = &g_dog_motor_config[idx];
    if (s_motor_online[idx] == 0U) {
        DebugUart_Printf("TqTest skip M%u(bus%u id%u): offline\r\n",
                         (unsigned)idx, (unsigned)bus, (unsigned)node_id);
        return 0U;
    }
    if (motor_has_fault(idx) != 0U) {
        DebugUart_Printf("TqTest skip M%u(bus%u id%u): fault\r\n",
                         (unsigned)idx, (unsigned)bus, (unsigned)node_id);
        return 0U;
    }

    MWClearErrors(cfg->bus, cfg->node_id);
    (void)enter_mit_probe_closed_loop(idx);
    HAL_Delay(20U);
    fdcan_poll_rx(&hfdcan1);
    fdcan_poll_rx(&hfdcan2);

    configure_motor_mit(idx);
    s_motor_mit_probe_active[idx] = 0U;

    s_mit_torque_test_index = idx;
    s_mit_torque_test_nm = torque_nm;
    s_mit_torque_test_active = 1U;

    send_mit_fixed_torque(idx, torque_nm);
    fdcan_poll_rx(&hfdcan1);
    fdcan_poll_rx(&hfdcan2);

    DebugUart_Printf("TqTest M%u(bus%u id%u) MIT torque=%+.2fNm(out) loop=%u iq=%ldmA\r\n",
                     (unsigned)idx,
                     (unsigned)cfg->bus,
                     (unsigned)cfg->node_id,
                     (double)torque_nm,
                     (unsigned)motor_closed_loop(idx),
                     (long)(g_mw_motor_data[idx].iq.iqMeasured * 1000.0f));
    return 1U;
}

void dog_debug_rx_only(void)
{
    dog_mit_diag_support_stop();
    dog_mit_march_in_place_stop();
    dog_debug_mit_torque_stop();
    mit_debug_stop_tx();
    s_imu_balance_stand_hold_active = 0U;
    s_auto_stand_enabled = 0U;
    memset(s_motor_mit_probe_active, 0, sizeof(s_motor_mit_probe_active));
    memset(s_encoder_turn_valid, 0, sizeof(s_encoder_turn_valid));
    if (s_stand_state != DOG_STAND_ESTOP) {
        s_stand_state = DOG_STAND_IDLE;
    }
}

void dog_debug_clear_errors(void)
{
    for (uint8_t i = 0U; i < selected_count(); ++i) {
        uint8_t idx = selected_index(i);
        if (idx < DOG_MOTOR_COUNT) {
            MWClearErrors(g_dog_motor_config[idx].bus, g_dog_motor_config[idx].node_id);
        }
    }
}

void dog_debug_position_setup(void)
{
    teach_hold_stop();
    s_mit_debug_active = 0U;
    s_control_loop_mode = DOG_CTRL_LOOP_POSITION;
    for (uint8_t i = 0U; i < selected_count(); ++i) configure_motor_position(selected_index(i));
}

void dog_debug_mit_setup(void)
{
    teach_hold_stop();
    s_control_loop_mode = DOG_CTRL_LOOP_MIT_PID;
    dog_mit_reset_integrators();
    for (uint8_t i = 0U; i < selected_count(); ++i) configure_motor_mit(selected_index(i));
}

uint8_t dog_mit_debug_is_active(void)
{
    return s_mit_debug_active;
}

uint8_t dog_mit_fault_hold_is_active(void)
{
    return s_mit_fault_hold_active;
}

static uint8_t capture_encoder_zero_turn(uint8_t index)
{
    if (index >= DOG_MOTOR_COUNT) {
        return 0U;
    }

    for (uint8_t n = 0U; n < 20U; ++n) {
        send_mit_zero_effort(index, 0.0f);
        mw_query_encoder(index);
        fdcan_poll_rx(&hfdcan1);
        fdcan_poll_rx(&hfdcan2);
        HAL_Delay(5U);
    }

    if ((motor_closed_loop(index) == 0U) || (s_encoder_est_fresh[index] == 0U)) {
        return 0U;
    }

    s_zero_offset_turn[index] = g_mw_motor_data[index].encoderEstimates.encoderPosEstimate;
    s_encoder_turn_filt[index] = s_zero_offset_turn[index];
    s_encoder_turn_valid[index] = 1U;
    return 1U;
}

static void mit_debug_settle_and_arm(void)
{
    uint32_t t0 = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - t0) < DOG_MIT_DEBUG_SETTLE_MS) {
        for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
            if (s_mit_boot_ok[i] == 0U) {
                continue;
            }
            send_mit_zero_effort(i, 0.0f);
            mw_query_encoder(i);
        }
        fdcan_poll_rx(&hfdcan1);
        fdcan_poll_rx(&hfdcan2);
        HAL_Delay(5U);
    }

    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if (s_mit_boot_ok[i] == 0U) {
            continue;
        }
        s_target_deg[i] = user_deg(i);
        s_target_turn[i] = g_mw_motor_data[i].encoderEstimates.encoderPosEstimate;
        mit_reset_motor_integrator(i);
    }
}

uint8_t dog_debug_mit_boot_sequence(void)
{
    uint8_t ok_count = 0U;

    s_position_tx_enabled = 0U;
    s_mit_debug_active = 0U;
    s_mit_fault_hold_active = 0U;
    teach_hold_stop();
    memset(s_mit_boot_ok, 0, sizeof(s_mit_boot_ok));

    for (uint8_t i = 0U; i < selected_count(); ++i) {
        uint8_t idx = selected_index(i);
        if (idx >= DOG_MOTOR_COUNT) {
            continue;
        }

        Dog_Motor_Config *cfg = &g_dog_motor_config[idx];
        if (s_motor_online[idx] == 0U) {
            DebugUart_Printf("BOOT skip M%u(bus%u id%u): offline\r\n",
                             (unsigned)idx, (unsigned)cfg->bus, (unsigned)cfg->node_id);
            continue;
        }
        if (motor_has_fault(idx) != 0U) {
            DebugUart_Printf("BOOT skip M%u(bus%u id%u): fault\r\n",
                             (unsigned)idx, (unsigned)cfg->bus, (unsigned)cfg->node_id);
            continue;
        }

        uint8_t cl_result = request_mit_probe_for_angle(idx, DOG_ENCODER_TORQUE_WAIT_MS);
        if (cl_result != 1U) {
            DebugUart_Printf("BOOT probe-fail M%u(bus%u id%u) cl=%u loop=%u enc=%u\r\n",
                             (unsigned)idx,
                             (unsigned)cfg->bus,
                             (unsigned)cfg->node_id,
                             (unsigned)cl_result,
                             (unsigned)motor_closed_loop(idx),
                             (unsigned)s_encoder_est_fresh[idx]);
            continue;
        }

        if (capture_encoder_zero_turn(idx) == 0U) {
            DebugUart_Printf("BOOT zero-fail M%u(bus%u id%u)\r\n",
                             (unsigned)idx, (unsigned)cfg->bus, (unsigned)cfg->node_id);
            continue;
        }

        s_target_turn[idx] = s_zero_offset_turn[idx];
        s_target_deg[idx] = 0.0f;
        s_motor_mit_probe_active[idx] = 0U;
        configure_motor_mit(idx);
        s_mit_boot_ok[idx] = 1U;
        ok_count++;

        long real_mdeg = (long)(raw_deg(idx) * 1000.0f);
        long user_mdeg = (long)(user_deg(idx) * 1000.0f);
        long abs_mdeg = (real_mdeg < 0L) ? -real_mdeg : real_mdeg;
        long abs_user = (user_mdeg < 0L) ? -user_mdeg : user_mdeg;
        DebugUart_Printf("BOOT ok M%u(bus%u id%u) real_angle=%c%ld.%03ld user=%c%ld.%03ld\r\n",
                         (unsigned)idx,
                         (unsigned)cfg->bus,
                         (unsigned)cfg->node_id,
                         (real_mdeg < 0L) ? '-' : '+',
                         abs_mdeg / 1000L,
                         abs_mdeg % 1000L,
                         (user_mdeg < 0L) ? '-' : '+',
                         abs_user / 1000L,
                         abs_user % 1000L);
    }

    if (ok_count == 0U) {
        return 0U;
    }
    if (ok_count != selected_count()) {
        DebugUart_Printf("Stand FAIL: boot OK %u/%u, wait all selected motors online/ready.\r\n",
                         (unsigned)ok_count,
                         (unsigned)selected_count());
        dog_debug_rx_only();
        return 0U;
    }

    mit_debug_settle_and_arm();

    s_control_loop_mode = DOG_CTRL_LOOP_MIT_PID;
    s_mit_debug_active = 1U;
    s_mit_fault_hold_active = 0U;
    s_position_tx_enabled = 1U;
    s_last_command_tick_ms = 0U;
    return ok_count;
}

uint8_t dog_debug_teach_hold_start(void)
{
    uint8_t ok_count = 0U;
    uint32_t now = HAL_GetTick();

    s_position_tx_enabled = 0U;
    s_auto_stand_enabled = 0U;
    s_mit_debug_active = 0U;
    s_mit_fault_hold_active = 0U;
    teach_hold_stop();
    memset(s_mit_boot_ok, 0, sizeof(s_mit_boot_ok));

    for (uint8_t i = 0U; i < selected_count(); ++i) {
        uint8_t idx = selected_index(i);
        if (idx >= DOG_MOTOR_COUNT) {
            continue;
        }

        Dog_Motor_Config *cfg = &g_dog_motor_config[idx];
        if (s_motor_online[idx] == 0U) {
            DebugUart_Printf("HOLD skip M%u(bus%u id%u): offline\r\n",
                             (unsigned)idx, (unsigned)cfg->bus, (unsigned)cfg->node_id);
            continue;
        }
        if (motor_has_fault(idx) != 0U) {
            DebugUart_Printf("HOLD skip M%u(bus%u id%u): fault\r\n",
                             (unsigned)idx, (unsigned)cfg->bus, (unsigned)cfg->node_id);
            continue;
        }

        uint8_t cl_result = request_mit_probe_for_angle(idx, DOG_ENCODER_TORQUE_WAIT_MS);
        if (cl_result != 1U) {
            DebugUart_Printf("HOLD probe-fail M%u(bus%u id%u) cl=%u loop=%u enc=%u\r\n",
                             (unsigned)idx,
                             (unsigned)cfg->bus,
                             (unsigned)cfg->node_id,
                             (unsigned)cl_result,
                             (unsigned)motor_closed_loop(idx),
                             (unsigned)s_encoder_est_fresh[idx]);
            continue;
        }

        s_target_deg[idx] = user_deg(idx);
        s_target_turn[idx] = g_mw_motor_data[idx].encoderEstimates.encoderPosEstimate;
        s_teach_hold_following[idx] = 1U;
        s_teach_hold_last_motion_ms[idx] = now;
        mit_reset_motor_integrator(idx);
        configure_motor_mit(idx);
        s_motor_mit_probe_active[idx] = 0U;
        s_mit_boot_ok[idx] = 1U;
        ok_count++;

        long user_mdeg = (long)(s_target_deg[idx] * 1000.0f);
        long abs_user = (user_mdeg < 0L) ? -user_mdeg : user_mdeg;
        DebugUart_Printf("HOLD ok M%u(bus%u id%u) user=%c%ld.%03ld deg\r\n",
                         (unsigned)idx,
                         (unsigned)cfg->bus,
                         (unsigned)cfg->node_id,
                         (user_mdeg < 0L) ? '-' : '+',
                         abs_user / 1000L,
                         abs_user % 1000L);
    }

    if (ok_count == 0U) {
        return 0U;
    }

    s_control_loop_mode = DOG_CTRL_LOOP_MIT_PID;
    s_mit_debug_active = 1U;
    s_mit_fault_hold_active = 0U;
    s_teach_hold_active = 1U;
    s_position_tx_enabled = 1U;
    s_last_command_tick_ms = 0U;
    s_mit_last_pid_ms = 0U;
    return ok_count;
}

void dog_debug_enter_closed_loop(void)
{
    s_position_tx_enabled = 0U;
    s_mit_debug_active = 0U;
    s_mit_fault_hold_active = 0U;
    teach_hold_stop();

    for (uint8_t i = 0U; i < selected_count(); ++i) {
        uint8_t idx = selected_index(i);
        if (idx >= DOG_MOTOR_COUNT) {
            continue;
        }

        Dog_Motor_Config *cfg = &g_dog_motor_config[idx];
        if (s_motor_online[idx] == 0U) {
            DebugUart_Printf("CL skip M%u(bus%u id%u): offline\r\n",
                             (unsigned)idx,
                             (unsigned)cfg->bus,
                             (unsigned)cfg->node_id);
            continue;
        }
        if (motor_has_fault(idx) != 0U) {
            DebugUart_Printf("CL skip M%u(bus%u id%u): fault axisErr=0x%08lX state=%u\r\n",
                             (unsigned)idx,
                             (unsigned)cfg->bus,
                             (unsigned)cfg->node_id,
                             (unsigned long)g_mw_motor_data[idx].heartBeat.ErrorStatus.axisError,
                             (unsigned)g_mw_motor_data[idx].heartBeat.currentState);
            continue;
        }

        uint8_t cl_result = request_mit_probe_for_angle(idx, DOG_ENCODER_TORQUE_WAIT_MS);
        if (cl_result == 0U) {
            DebugUart_Printf("CL FAIL M%u(bus%u id%u): offline/fault\r\n",
                             (unsigned)idx,
                             (unsigned)cfg->bus,
                             (unsigned)cfg->node_id);
            continue;
        }
        if (cl_result == 2U) {
            DebugUart_Printf("CL MIT-probe M%u(bus%u id%u) loop=%u enc=%u Ilim=%ldmA (no encoder RX yet)\r\n",
                             (unsigned)idx,
                             (unsigned)cfg->bus,
                             (unsigned)cfg->node_id,
                             (unsigned)motor_closed_loop(idx),
                             (unsigned)s_encoder_est_fresh[idx],
                             (long)(DOG_MIT_PROBE_CURRENT_LIMIT_A * 1000.0f));
            continue;
        }

        {
            long real_mdeg = (long)(raw_deg(idx) * 1000.0f);
            long abs_mdeg = (real_mdeg < 0L) ? -real_mdeg : real_mdeg;
            DebugUart_Printf("CL enc-ok M%u(bus%u id%u) loop=%u enc=%u real_angle=%c%ld.%03ld deg (MIT probe, no pos TX)\r\n",
                             (unsigned)idx,
                             (unsigned)cfg->bus,
                             (unsigned)cfg->node_id,
                             (unsigned)motor_closed_loop(idx),
                             (unsigned)s_encoder_est_fresh[idx],
                             (real_mdeg < 0L) ? '-' : '+',
                             abs_mdeg / 1000L,
                             abs_mdeg % 1000L);
        }
    }
}

void dog_debug_start_position_tx(void)
{
    for (uint8_t i = 0U; i < selected_count(); ++i) {
        uint8_t idx = selected_index(i);
        if ((idx < DOG_MOTOR_COUNT) && (motor_ready(idx) == 0U)) {
            DebugUart_Printf("Start ignored: target index=%u not ready.\r\n", (unsigned)idx);
            return;
        }
    }
    s_position_tx_enabled = 1U;
    s_last_command_tick_ms = 0U;
    if (s_control_loop_mode == DOG_CTRL_LOOP_MIT_PID) {
        dog_mit_reset_integrators();
    }
}

void dog_debug_idle(void)
{
    dog_debug_mit_torque_stop();
    for (uint8_t i = 0U; i < selected_count(); ++i) {
        uint8_t idx = selected_index(i);
        if (idx < DOG_MOTOR_COUNT) {
            MWSetAxisState(g_dog_motor_config[idx].bus, g_dog_motor_config[idx].node_id, MW_AXIS_STATE_IDLE);
        }
    }
    dog_debug_rx_only();
}

void dog_leg_set_target_leg_deg(float hip_deg, float knee_deg)
{
    teach_hold_stop();
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        if (leg_selected(leg) == 0U) continue;

        float q_thigh = hip_deg;
        float q_shank = knee_deg;
        (void)dog_leg_geom_to_motor(leg, hip_deg, knee_deg, &q_thigh, &q_shank);

        uint8_t hip = leg_joint_index(leg, DOG_JOINT_HIP);
        uint8_t knee = leg_joint_index(leg, DOG_JOINT_KNEE);
        if ((hip < DOG_MOTOR_COUNT) && (s_debug_target != DOG_DEBUG_TARGET_SINGLE_KNEE)) {
            s_target_turn[hip] = command_turn_from_user_deg(hip, q_thigh);
        }
        if ((knee < DOG_MOTOR_COUNT) && (s_debug_target != DOG_DEBUG_TARGET_SINGLE)) {
            s_target_turn[knee] = command_turn_from_user_deg(knee, q_shank);
        }
    }
}

void dog_leg_set_target_motor_user_deg(float hip_motor_deg, float knee_motor_deg)
{
    teach_hold_stop();
    for (uint8_t i = 0U; i < selected_count(); ++i) {
        uint8_t idx = selected_index(i);
        if (idx >= DOG_MOTOR_COUNT) {
            continue;
        }

        if (g_dog_motor_config[idx].joint == DOG_JOINT_HIP) {
            if (s_debug_target != DOG_DEBUG_TARGET_SINGLE_KNEE) {
                (void)command_turn_from_user_deg(idx, hip_motor_deg);
            }
        } else if (s_debug_target != DOG_DEBUG_TARGET_SINGLE) {
            (void)command_turn_from_user_deg(idx, knee_motor_deg);
        }
    }
}

void dog_leg_get_target_leg_raw_angles(float *hip_deg, float *knee_deg)
{
    uint8_t hip = leg_joint_index(s_target_leg, DOG_JOINT_HIP);
    uint8_t knee = leg_joint_index(s_target_leg, DOG_JOINT_KNEE);
    if (hip_deg != nullptr) *hip_deg = raw_deg(hip);
    if (knee_deg != nullptr) *knee_deg = raw_deg(knee);
}

void dog_leg_get_target_leg_angles(float *hip_deg, float *knee_deg)
{
    uint8_t hip = leg_joint_index(s_target_leg, DOG_JOINT_HIP);
    uint8_t knee = leg_joint_index(s_target_leg, DOG_JOINT_KNEE);
    if (hip_deg != nullptr) *hip_deg = user_deg(hip);
    if (knee_deg != nullptr) *knee_deg = user_deg(knee);
}

void dog_leg_get_target_leg_cmd_deg(float *hip_deg, float *knee_deg)
{
    uint8_t hip = leg_joint_index(s_target_leg, DOG_JOINT_HIP);
    uint8_t knee = leg_joint_index(s_target_leg, DOG_JOINT_KNEE);
    if (hip_deg != nullptr) {
        *hip_deg = (hip < DOG_MOTOR_COUNT) ? s_target_deg[hip] : 0.0f;
    }
    if (knee_deg != nullptr) {
        *knee_deg = (knee < DOG_MOTOR_COUNT) ? s_target_deg[knee] : 0.0f;
    }
}

void dog_leg_get_target_encoder_fresh(uint8_t *hip_fresh, uint8_t *knee_fresh)
{
    uint8_t hip = leg_joint_index(s_target_leg, DOG_JOINT_HIP);
    uint8_t knee = leg_joint_index(s_target_leg, DOG_JOINT_KNEE);
    if (hip_fresh != nullptr) {
        *hip_fresh = (hip < DOG_MOTOR_COUNT) ? s_encoder_est_fresh[hip] : 0U;
    }
    if (knee_fresh != nullptr) {
        *knee_fresh = (knee < DOG_MOTOR_COUNT) ? s_encoder_est_fresh[knee] : 0U;
    }
}

void dog_leg_get_target_zero_offsets(float *hip_deg, float *knee_deg)
{
    uint8_t hip = leg_joint_index(s_target_leg, DOG_JOINT_HIP);
    uint8_t knee = leg_joint_index(s_target_leg, DOG_JOINT_KNEE);
    if (hip_deg != nullptr) *hip_deg = (hip < DOG_MOTOR_COUNT) ? s_zero_offset_turn[hip] * DOG_TURN_TO_DEG : 0.0f;
    if (knee_deg != nullptr) *knee_deg = (knee < DOG_MOTOR_COUNT) ? s_zero_offset_turn[knee] * DOG_TURN_TO_DEG : 0.0f;
}

uint8_t dog_leg_set_target_zero_current(void)
{
    uint8_t ok_count = 0U;
    for (uint8_t i = 0U; i < selected_count(); ++i) {
        uint8_t idx = selected_index(i);
        if ((idx < DOG_MOTOR_COUNT) && (s_encoder_est_fresh[idx] != 0U)) {
            s_zero_offset_turn[idx] = g_mw_motor_data[idx].encoderEstimates.encoderPosEstimate;
            s_encoder_turn_filt[idx] = s_zero_offset_turn[idx];
            s_encoder_turn_valid[idx] = 1U;
            s_target_turn[idx] = s_zero_offset_turn[idx];
            s_target_deg[idx] = 0.0f;
            mit_reset_motor_integrator(idx);
            ok_count++;
        }
    }
    return ok_count;
}

uint8_t dog_leg_target_motor_mode_mask(void)
{
    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < selected_count(); ++i) {
        if (motor_closed_loop(selected_index(i)) != 0U) mask |= (uint8_t)(1U << i);
    }
    return mask;
}

uint8_t dog_leg_target_online_mask(void)
{
    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < selected_count(); ++i) {
        uint8_t idx = selected_index(i);
        if ((idx < DOG_MOTOR_COUNT) && (s_motor_online[idx] != 0U)) mask |= (uint8_t)(1U << i);
    }
    return mask;
}

uint8_t dog_leg_target_ready_mask(void)
{
    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < selected_count(); ++i) {
        if (motor_ready(selected_index(i)) != 0U) mask |= (uint8_t)(1U << i);
    }
    return mask;
}

uint8_t dog_leg_target_online_ids(uint16_t *ids, uint8_t max_count)
{
    uint8_t count = 0U;
    if ((ids == nullptr) || (max_count == 0U)) return 0U;
    for (uint8_t i = 0U; (i < selected_count()) && (count < max_count); ++i) {
        uint8_t idx = selected_index(i);
        if ((idx < DOG_MOTOR_COUNT) && (s_motor_online[idx] != 0U)) ids[count++] = g_dog_motor_config[idx].node_id;
    }
    return count;
}

uint8_t dog_leg_seen_online_ids(Dog_Motor_Seen *seen, uint8_t max_count)
{
    uint8_t count = 0U;
    if ((seen == nullptr) || (max_count == 0U)) return 0U;
    for (uint8_t i = 0U; (i < DOG_MOTOR_COUNT) && (count < max_count); ++i) {
        if (s_motor_online[i] != 0U) {
            seen[count].bus = g_dog_motor_config[i].bus;
            seen[count].motor_id = g_dog_motor_config[i].node_id;
            count++;
        }
    }
    return count;
}

void dog_can1_get_diag(Dog_Can_Diag *diag) { fill_diag(DOG_CAN_FRONT_BUS, diag); }
void dog_can2_get_diag(Dog_Can_Diag *diag) { fill_diag(DOG_CAN_REAR_BUS, diag); }

static long angle_to_mdeg(float deg)
{
    return (long)((deg >= 0.0f) ? (deg * 1000.0f + 0.5f) : (deg * 1000.0f - 0.5f));
}

static void print_real_angle_mdeg(long mdeg)
{
    long abs_value = (mdeg < 0L) ? -mdeg : mdeg;
    DebugUart_Printf("%c%ld.%03ld",
                     (mdeg < 0L) ? '-' : '+',
                     abs_value / 1000L,
                     abs_value % 1000L);
}

static void print_mm_mdeg(long mdmm)
{
    long abs_value = (mdmm < 0L) ? -mdmm : mdmm;
    DebugUart_Printf("%c%ld.%03ld",
                     (mdmm < 0L) ? '-' : '+',
                     abs_value / 1000L,
                     abs_value % 1000L);
}

static void print_leg_foot_xz(uint8_t leg, float hip_motor_deg, float knee_motor_deg,
                              const char *label)
{
    float x_mm = 0.0f;
    float z_mm = 0.0f;
    if (dog_leg_foot_xz_from_motor_deg(leg, hip_motor_deg, knee_motor_deg, &x_mm, &z_mm) == 0U) {
        DebugUart_Printf("%s=(?,?)mm ", label);
        return;
    }

    DebugUart_Printf("%s=(", label);
    print_mm_mdeg((long)(x_mm * 1000.0f));
    DebugUart_Printf(",");
    print_mm_mdeg((long)(z_mm * 1000.0f));
    DebugUart_Printf(")mm ");
}

static void print_motor_pid_debug(uint8_t index)
{
    if (index >= DOG_MOTOR_COUNT) {
        DebugUart_Printf("P=0 I=0 D=0 F=0");
        return;
    }

    DebugUart_Printf("P=%ld I=%ld D=%ld F=%ldmA",
                     (long)(s_mit_pid_p_a[index] * 1000.0f),
                     (long)(s_mit_pid_i_a[index] * 1000.0f),
                     (long)(s_mit_pid_d_a[index] * 1000.0f),
                     (long)(s_mit_pid_ff_a[index] * 1000.0f));
}

void dog_leg_print_angle_status_for_leg(const char *tag, uint8_t leg)
{
    uint8_t hip = leg_joint_index(leg, DOG_JOINT_HIP);
    uint8_t knee = leg_joint_index(leg, DOG_JOINT_KNEE);
    float user_hip = (hip < DOG_MOTOR_COUNT) ? user_deg(hip) : 0.0f;
    float user_knee = (knee < DOG_MOTOR_COUNT) ? user_deg(knee) : 0.0f;
    float tgt_hip = (hip < DOG_MOTOR_COUNT) ? s_target_deg[hip] : 0.0f;
    float tgt_knee = (knee < DOG_MOTOR_COUNT) ? s_target_deg[knee] : 0.0f;

    if (tag != nullptr) {
        DebugUart_Printf("%s", tag);
    }
    DebugUart_Printf("%s ", dog_leg_name(leg));
    if (s_mit_debug_active != 0U) {
        DebugUart_Printf("pid=%s ", dog_mit_pid_profile_name());
    }
    DebugUart_Printf("real_angle=(");
    print_real_angle_mdeg((hip < DOG_MOTOR_COUNT) ? angle_to_mdeg(raw_deg(hip)) : 0L);
    DebugUart_Printf(",");
    print_real_angle_mdeg((knee < DOG_MOTOR_COUNT) ? angle_to_mdeg(raw_deg(knee)) : 0L);
    DebugUart_Printf(") enc=(");
    DebugUart_Printf("%u,%u",
                     (unsigned)((hip < DOG_MOTOR_COUNT) ? s_encoder_est_fresh[hip] : 0U),
                     (unsigned)((knee < DOG_MOTOR_COUNT) ? s_encoder_est_fresh[knee] : 0U));
    DebugUart_Printf(") loop=(");
    DebugUart_Printf("%u,%u",
                     (unsigned)((hip < DOG_MOTOR_COUNT) ? motor_closed_loop(hip) : 0U),
                     (unsigned)((knee < DOG_MOTOR_COUNT) ? motor_closed_loop(knee) : 0U));
    DebugUart_Printf(") user_angle=(");
    print_real_angle_mdeg(angle_to_mdeg(user_hip));
    DebugUart_Printf(",");
    print_real_angle_mdeg(angle_to_mdeg(user_knee));
    DebugUart_Printf(") tgt=(");
    print_real_angle_mdeg(angle_to_mdeg(tgt_hip));
    DebugUart_Printf(",");
    print_real_angle_mdeg(angle_to_mdeg(tgt_knee));
    DebugUart_Printf(") hip ");
    print_motor_pid_debug(hip);
    DebugUart_Printf(" knee ");
    print_motor_pid_debug(knee);
    DebugUart_Printf(" cmdI=(");
    DebugUart_Printf("%ld,%ld",
                     (long)((hip < DOG_MOTOR_COUNT) ? (s_mit_cmd_current_a[hip] * 1000.0f) : 0.0f),
                     (long)((knee < DOG_MOTOR_COUNT) ? (s_mit_cmd_current_a[knee] * 1000.0f) : 0.0f));
    DebugUart_Printf(")mA ");
    print_leg_foot_xz(leg, user_hip, user_knee, "user_xz");
    print_leg_foot_xz(leg, tgt_hip, tgt_knee, "tgt_xz");
    DebugUart_Printf("\r\n");
}

void dog_leg_print_angle_status(const char *tag)
{
    dog_leg_print_angle_status_for_leg(tag, s_target_leg);
}

void dog_lf_print_periodic_status(void)
{
    dog_leg_print_angle_status_for_leg("LEG1s ", DOG_LEG_LF);
}

void dog_leg_dump_target_status(void)
{
    DebugUart_Printf("TARGET %s leg=%s count=%u\r\n",
                     dog_debug_target_name(),
                     dog_leg_name(s_target_leg),
                     (unsigned)selected_count());
    for (uint8_t i = 0U; i < selected_count(); ++i) {
        uint8_t idx = selected_index(i);
        if (idx >= DOG_MOTOR_COUNT) {
            DebugUart_Printf("  sel%u: invalid\r\n", (unsigned)i);
            continue;
        }

        Dog_Motor_Config *cfg = &g_dog_motor_config[idx];
        DebugUart_Printf("  sel%u M%u %s %s bus%u id%u online=%u loop=%u enc=%u fault=%u ready=%u user=",
                         (unsigned)i,
                         (unsigned)idx,
                         dog_leg_name(cfg->leg),
                         joint_name(cfg->joint),
                         (unsigned)cfg->bus,
                         (unsigned)cfg->node_id,
                         (unsigned)s_motor_online[idx],
                         (unsigned)motor_closed_loop(idx),
                         (unsigned)s_encoder_est_fresh[idx],
                         (unsigned)motor_has_fault(idx),
                         (unsigned)motor_ready(idx));
        print_real_angle_mdeg(angle_to_mdeg(user_deg(idx)));
        DebugUart_Printf(" tgt=");
        print_real_angle_mdeg(angle_to_mdeg(s_target_deg[idx]));
        DebugUart_Printf(" ");
        print_motor_pid_debug(idx);
        DebugUart_Printf(" cmdI=%ldmA dir=%ld tdir=%ld\r\n",
                         (long)(s_mit_cmd_current_a[idx] * 1000.0f),
                         (long)cfg->direction,
                         (long)cfg->torque_direction);
    }
}

uint8_t dog_motor_encoder_fresh(uint8_t motor_index)
{
    return (motor_index < DOG_MOTOR_COUNT) ? s_encoder_est_fresh[motor_index] : 0U;
}

void dog_motor_poll_can(void)
{
    fdcan_poll_rx(&hfdcan1);
    fdcan_poll_rx(&hfdcan2);
}

void dog_motor_query_encoder(uint8_t motor_index)
{
    mw_query_encoder(motor_index);
}

void dog_motor_query_online_encoders(void)
{
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if (s_motor_online[i] != 0U) {
            mw_query_encoder(i);
        }
    }
}

void DogImu_Update(const Dog_Imu_Sample *sample)
{
    if (sample != nullptr) s_imu_sample = *sample;
}

void DogRemote_Update(const Dog_Remote_Sample *sample)
{
    if (sample != nullptr) s_remote_sample = *sample;
}

void DogStand_Request(void)
{
    mit_debug_stop_tx();
    s_auto_stand_enabled = 1U;
    s_state_start_ms = HAL_GetTick();
    s_stand_state = DOG_STAND_WAIT_HEARTBEAT;
}

void DogStand_Estop(void)
{
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        MWEstop(g_dog_motor_config[i].bus, g_dog_motor_config[i].node_id);
    }
    mit_debug_stop_tx();
    s_auto_stand_enabled = 0U;
    s_stand_state = DOG_STAND_ESTOP;
}

Dog_Stand_State DogStand_GetState(void) { return s_stand_state; }

uint8_t DogStand_GetOnlineMask(void)
{
    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if (s_motor_online[i] != 0U) mask |= (uint8_t)(1U << i);
    }
    return mask;
}

uint8_t DogStand_GetReadyMask(void)
{
    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if (motor_ready(i) != 0U) mask |= (uint8_t)(1U << i);
    }
    return mask;
}

void motor_task_init(void)
{
    bsp_can_init(&hfdcan1, CAN1_Callback);
    bsp_can_init(&hfdcan2, CAN2_Callback);
    ArmMotor_Init(&hfdcan1,
                  ARM_J0_DM_CAN_ID,
                  ARM_J0_DM_FEEDBACK_ID,
                  &hfdcan1,
                  ARM_J1_EL05_CAN_ID,
                  ARM_J1_EL05_INIT_MODEL_IGNORED);

    memset(g_mw_motor_data, 0, sizeof(g_mw_motor_data));
    memset(s_last_rx_tick_ms, 0, sizeof(s_last_rx_tick_ms));
    memset(s_motor_online, 0, sizeof(s_motor_online));
    memset(s_motor_configured, 0, sizeof(s_motor_configured));
    memset(s_motor_loop_requested, 0, sizeof(s_motor_loop_requested));
    memset(s_encoder_est_fresh, 0, sizeof(s_encoder_est_fresh));
    memset(s_encoder_rx_count, 0, sizeof(s_encoder_rx_count));
    memset(s_motor_mit_probe_active, 0, sizeof(s_motor_mit_probe_active));
    memset(s_mit_boot_ok, 0, sizeof(s_mit_boot_ok));
    memset(s_encoder_turn_valid, 0, sizeof(s_encoder_turn_valid));
    s_mit_debug_active = 0U;
    teach_hold_stop();

    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        g_mw_node_ids[i] = g_dog_motor_config[i].node_id;
        MW_MOTOR_ACCESS_INFO motor = {};
        motor.busId = g_dog_motor_config[i].bus;
        motor.nodeId = g_dog_motor_config[i].node_id;
        motor.motorData = &g_mw_motor_data[i];
        motor.sender = mw_sender;
        motor.notifier = mw_notifier;
        (void)MWRegisterMotor(motor);
    }

    dog_debug_rx_only();
    DebugUart_Printf("quadruped SDK debug: CAN1 front nodes=1..4 CAN2 rear nodes=1..4 arm J0_DM=0x01/0x10 J1_EL05=0x7F disabled\r\n");
}

void motor_task(void)
{
    uint32_t now = HAL_GetTick();

    fdcan_poll_rx(&hfdcan1);
    fdcan_poll_rx(&hfdcan2);

    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((s_motor_online[i] != 0U) && ((uint32_t)(now - s_last_rx_tick_ms[i]) > DOG_RX_TIMEOUT_MS)) {
            s_motor_online[i] = 0U;
            s_encoder_est_fresh[i] = 0U;
            if ((s_mit_fault_hold_active != 0U) && (s_mit_boot_ok[i] != 0U)) {
                mit_debug_abort_fault_hold("rx timeout");
                break;
            }
        }
        if (motor_closed_loop(i) == 0U) {
            s_encoder_est_fresh[i] = 0U;
        }
        if ((s_motor_loop_requested[i] != 0U) && (motor_has_fault(i) != 0U)) {
            s_motor_loop_requested[i] = 0U;
        }
    }

    stand_state_tick(now);
    march_in_place_tick(now);
    dog_imu_balance_refresh_stand_hold(now);

    send_mit_probe_keepalive(now);
    send_mit_torque_test_keepalive(now);

    static uint32_t s_arm_cmd_last_ms = 0U;
    if ((uint32_t)(now - s_arm_cmd_last_ms) >= ARM_CMD_PERIOD_MS) {
        s_arm_cmd_last_ms = now;
        ArmMotor_Send();
    }

    static uint32_t s_closed_loop_retry_last_ms = 0U;
    if ((uint32_t)(now - s_closed_loop_retry_last_ms) >= DOG_CLOSED_LOOP_RETRY_MS) {
        s_closed_loop_retry_last_ms = now;
        for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
            if ((s_motor_loop_requested[i] != 0U) &&
                (s_motor_online[i] != 0U) &&
                (motor_has_fault(i) == 0U) &&
                (motor_closed_loop(i) == 0U)) {
                if (s_motor_mit_probe_active[i] != 0U) {
                    (void)enter_mit_probe_closed_loop(i);
                } else {
                    request_closed_loop(i);
                }
            }
        }
    }

    if (s_position_tx_enabled != 0U) {
        send_enabled_targets(now);
    }

    static uint32_t s_encoder_req_last_ms = 0U;
    if ((uint32_t)(now - s_encoder_req_last_ms) >= DOG_ENCODER_FEEDBACK_PERIOD_MS) {
        s_encoder_req_last_ms = now;
        for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
            if (s_motor_online[i] == 0U) {
                continue;
            }
            mw_query_encoder(i);
        }
    }

    static uint32_t s_slow_feedback_last_ms = 0U;
    if ((uint32_t)(now - s_slow_feedback_last_ms) >= DOG_SLOW_FEEDBACK_PERIOD_MS) {
        s_slow_feedback_last_ms = now;
        for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
            if (s_motor_online[i] == 0U) {
                continue;
            }
            MWGetIq(g_dog_motor_config[i].bus, g_dog_motor_config[i].node_id);
            MWGetBusVoltageCurrent(g_dog_motor_config[i].bus, g_dog_motor_config[i].node_id);
        }
    }

    static uint32_t s_diag_last_ms = 0U;
    if ((uint32_t)(now - s_diag_last_ms) >= DOG_DIAG_PERIOD_MS) {
        s_diag_last_ms = now;
        restart_bus_off(&hfdcan1);
        restart_bus_off(&hfdcan2);
    }

    DebugUart_Process();
}

extern "C" void motor_task_tick(void)
{
    motor_task();
}
