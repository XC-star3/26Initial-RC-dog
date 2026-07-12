#include "motor_task.h"

#include "arm_motor_task.h"
#include "bsp_fdcan.h"
#include "control_task.h"
#include "debug_uart.h"
#include "vofa_pid.h"
#include "wheel_motor_task.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include <string.h>
#include <math.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

#define ARM_J0_DM_CAN_ID               0x01U
#define ARM_J0_DM_FEEDBACK_ID          0x10U
#define ARM_J1_EL05_CAN_ID             0x7FU
#define ARM_J1_EL05_INIT_MODEL_IGNORED 0U
#define ARM_CMD_PERIOD_MS              5U

#define DOG_CTRL_HZ                    500U
#define DOG_CMD_PERIOD_MS              (1000U / DOG_CTRL_HZ)
#define DOG_RX_TIMEOUT_MS              250U
#define DOG_HEARTBEAT_TIMEOUT_MS        250U
#define DOG_ENCODER_FEEDBACK_TIMEOUT_MS 120U
#define DOG_STAND_WAIT_MS              1500U
#define DOG_STAND_CONFIG_MS            200U
#define DOG_STAND_LOOP_MS              300U
#define DOG_STAND_READY_TIMEOUT_MS     800U
#define DOG_POSITION_ARM_TIMEOUT_MS   2000U
#define DOG_STAND_CONFIG_SLOT_MS       (DOG_STAND_CONFIG_MS / DOG_MOTOR_COUNT)
#define DOG_STAND_LOOP_SLOT_MS         (DOG_STAND_LOOP_MS / DOG_MOTOR_COUNT)
#define DOG_STAND_MOVE_MS              2500U
#define DOG_GAIT_STOP_NEUTRAL_MS        200U
#define DOG_DRIVE_COMMAND_SLEW_PER_HALF_STEP 0.25f
#define DOG_GAIT_WHEEL_MAX_CONTRIBUTION 0.30f
#define DOG_WHEEL_DIAMETER_MM           116.0f
#define DOG_GAIT_SWING_APEX_PROGRESS    0.45f
#define DOG_JUMP_SETTLE_MS             500U
#define DOG_ENCODER_FEEDBACK_PERIOD_MS  DOG_CMD_PERIOD_MS
#define DOG_SLOW_FEEDBACK_SLOT_MS       25U
#define DOG_CAN_RECOVERY_PERIOD_MS      10U
#define DOG_ENCODER_WAIT_MS            300U
#define DOG_ENCODER_TORQUE_WAIT_MS     500U
#define DOG_MIT_BOOT_TX_TIMEOUT_MS     100U
#define DOG_MIT_BOOT_BUS_QUIET_MS        6U
#define DOG_CLOSED_LOOP_RETRY_MS       200U
#define DOG_CLOSED_LOOP_RETRY_SLOT_MS  (DOG_CLOSED_LOOP_RETRY_MS / DOG_MOTOR_COUNT)
#define DOG_SAFETY_RETRY_PERIOD_MS     100U
#define DOG_SAFETY_CLEAR_CONFIRM_MS    200U
#define DOG_MECHANICAL_IDLE_SETTLE_MS  500U

#define DOG_FINAL_MODE_NONE                    0U
#define DOG_FINAL_POSITION_WAIT_IDLE           1U
#define DOG_FINAL_POSITION_WAIT_IDLE_ENCODER   2U
#define DOG_FINAL_POSITION_WAIT_CLOSED         3U

#define DOG_CAN_ID_UNKNOWN             0U
#define DOG_CAN_ID_STANDARD            1U
#define DOG_CAN_ID_EXTENDED            2U

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
#define DOG_STAND_OUTPUT_LIMIT_A        18.0f
#define DOG_MIT_ANG_PID_INTEGRAL_LIMIT_A 3.0f
#define DOG_GAIT_STABILITY_VEL_TAU_MS      40.0f

#define DOG_SWING_KP_A_PER_DEG          1.7f
#define DOG_SWING_KI_A_PER_DEG_S        0.00f
#define DOG_SWING_KD_A_PER_DPS          0.017f
#define DOG_SWING_OUTPUT_LIMIT_A        18.0f

#define DOG_MIT_CURRENT_LIMIT_A         18.0f
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
static uint8_t s_position_tx_arm_pending_mask = 0U;
static uint32_t s_position_tx_arm_started_ms = 0U;
static uint8_t s_auto_stand_enabled = 0U;
static Dog_Control_Loop_Mode s_control_loop_mode = DOG_CTRL_LOOP_POSITION;
static Dog_Stand_State s_stand_state = DOG_STAND_IDLE;
static uint32_t s_state_start_ms = 0U;
static uint8_t s_stand_motor_cursor = 0U;
static uint32_t s_last_command_tick_ms = 0U;
static uint32_t s_last_rx_tick_ms[DOG_MOTOR_COUNT];
static uint8_t s_motor_online[DOG_MOTOR_COUNT];
static uint8_t s_motor_configured[DOG_MOTOR_COUNT];
static uint8_t s_motor_loop_requested[DOG_MOTOR_COUNT];
static uint8_t s_motor_final_mode_pending[DOG_MOTOR_COUNT];
static float s_zero_offset_turn[DOG_MOTOR_COUNT];
static float s_target_turn[DOG_MOTOR_COUNT];
static float s_position_hold_turn[DOG_MOTOR_COUNT];
static uint32_t s_position_idle_encoder_rx_baseline[DOG_MOTOR_COUNT];
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
static volatile uint8_t s_lower_state = DOG_LOWER_IDLE;
static uint32_t s_lower_t0_ms = 0U;
static uint32_t s_lower_settle_since_ms = 0U;

enum Dog_March_Phase {
    DOG_MARCH_PHASE_ENTRY_SETTLE = 0U,
    DOG_MARCH_PHASE_SWING_UP,
    DOG_MARCH_PHASE_HOLD,
    DOG_MARCH_PHASE_SWING_DOWN,
    DOG_MARCH_PHASE_TOUCHDOWN,
    DOG_MARCH_PHASE_STOP_NEUTRAL,
    DOG_MARCH_PHASE_PAUSE,
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
    uint8_t stop_requested;
    uint32_t entry_stable_since_ms;
    float contact_iq_baseline_a[DOG_LEG_COUNT];
    float contact_iq_filtered_a[DOG_LEG_COUNT];
    uint32_t contact_candidate_since_ms[DOG_LEG_COUNT];
    uint32_t contact_stable_since_ms[DOG_LEG_COUNT];
    uint32_t search_limit_since_ms[DOG_LEG_COUNT];
    float swing_progress[DOG_LEG_COUNT];
    float contact_search_mm[DOG_LEG_COUNT];
    float frozen_x_mm[DOG_LEG_COUNT];
    float frozen_z_mm[DOG_LEG_COUNT];
    uint8_t swing_mask;
    uint8_t contact_mask;
    uint8_t contact_search_mask;
    uint8_t contact_failure_mask;
    uint32_t half_step_generation;
    uint32_t active_swing_ms;
    uint32_t active_touchdown_dwell_ms;
    uint32_t active_diagonal_stagger_ms;
    float active_forward_stride_x_mm;
    float active_turn_stride_x_mm;
    float active_swing_height_mm;
    float requested_forward;
    float requested_yaw;
    float applied_forward;
    float applied_yaw;
    float active_forward;
    float active_yaw;
    float active_wheel_contribution;
    float active_leg_contribution;
    float compatible_wheel_rpm;
    float stop_progress;
    float active_stride_delta_mm[DOG_LEG_COUNT];
    uint8_t active_speed_profile;
    uint32_t touchdown_stable_since_ms;
    float stop_start_x_mm[DOG_LEG_COUNT];
    float stop_start_z_mm[DOG_LEG_COUNT];
    float stable_velocity_lpf_dps[DOG_MOTOR_COUNT];
    uint32_t stable_velocity_filter_ms;
    uint32_t contact_iq_query_ms;
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
    float swing_height_mm;
    uint32_t touchdown_dwell_ms;
    uint32_t diagonal_stagger_ms;
};

static const Dog_Gait_Speed_Profile s_gait_speed_profiles[] = {
    {"LOW",  1.0f, 25.0f, DOG_TURN_STRIDE_X_MM, 30.0f, 160U, 40U},
    {"MID",  2.0f, 55.0f, DOG_TURN_STRIDE_X_MM, 60.0f,  60U,  0U},
    {"HIGH", 2.5f, 70.0f, DOG_TURN_STRIDE_X_MM, 100.0f, 30U,  0U},
};

static float s_leg_foot_x_offset[DOG_LEG_COUNT] = {};
static float s_leg_touchdown_z_offset[DOG_LEG_COUNT] = {};
static float s_leg_command_x_mm[DOG_LEG_COUNT] = {};
static float s_leg_command_z_mm[DOG_LEG_COUNT] = {};
static float s_leg_hip_offset_deg[DOG_LEG_COUNT] = {};
static uint8_t s_gait_speed_profile = DOG_GAIT_SPEED_DEFAULT;
static volatile float s_requested_gait_wheel_contribution = 0.0f;
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
static volatile uint32_t s_can_tx_drop_count[2];
static uint32_t s_last_encoder_tick_ms[DOG_MOTOR_COUNT];
static uint32_t s_last_heartbeat_tick_ms[DOG_MOTOR_COUNT];
static uint32_t s_heartbeat_rx_count[DOG_MOTOR_COUNT];
static uint8_t s_motor_can_id_type[DOG_MOTOR_COUNT];
static uint8_t s_encoder_query_cursor[2];
static uint8_t s_slow_query_cursor[2];
static uint8_t s_slow_query_kind[DOG_MOTOR_COUNT];
static uint8_t s_mit_probe_keepalive_cursor[2];
static uint32_t s_encoder_query_last_ms = 0U;
static volatile uint8_t s_mit_probe_tx_busy_mask = 0U;
static volatile uint8_t s_safety_latched = 0U;
static volatile uint8_t s_safety_external_inhibit = 0U;
static volatile uint8_t s_control_disabled = 0U;
static volatile uint8_t s_mechanical_idle_requested = 0U;
static volatile uint8_t s_mechanical_idle_ready = 0U;
static volatile uint8_t s_mechanical_idle_mask = 0U;
static uint32_t s_mechanical_idle_settle_since_ms = 0U;
static uint32_t s_mechanical_idle_heartbeat_baseline[DOG_MOTOR_COUNT];
static volatile uint8_t s_mechanical_pose_requested = 0U;
static volatile uint8_t s_mechanical_pose_ready = 0U;
static volatile uint8_t s_mechanical_pose_mask = 0U;
static volatile uint8_t s_safety_rearm_requested = 0U;
static volatile uint32_t s_safety_generation = 0U;
static uint32_t s_safety_latched_ms = 0U;
static uint32_t s_safety_clear_started_ms = 0U;
static uint32_t s_safety_clear_generation = 0U;
static uint32_t s_safety_last_action_ms = 0U;
static uint8_t s_estop_pending_mask = 0U;
static StaticSemaphore_t s_motor_tx_guard_storage;
static SemaphoreHandle_t s_motor_tx_guard = nullptr;
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
static Dog_Remote_Sample s_remote_sample;

static void motor_safety_tick(uint32_t now);
static uint8_t motor_feedback_health_tick(uint32_t now);
static void encoder_feedback_query_tick(uint32_t now);
static uint8_t motor_blocking_service(uint32_t *now_out);
static uint8_t motor_closed_loop(uint8_t index);
static uint8_t motor_has_fault(uint8_t index);
static uint8_t mit_probe_bus_tx_busy(uint8_t bus);
static void queue_motor_estop(uint8_t index);
static void DogStand_Estop(void);

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static const Dog_Gait_Speed_Profile *gait_speed_profile(void)
{
    if (s_gait_speed_profile > DOG_GAIT_SPEED_HIGH) {
        s_gait_speed_profile = DOG_GAIT_SPEED_DEFAULT;
    }
    return &s_gait_speed_profiles[s_gait_speed_profile];
}

static uint32_t march_walk_hold_ms(void)
{
    switch (s_march.active_speed_profile) {
    case DOG_GAIT_SPEED_LOW:
        return 250U;
    case DOG_GAIT_SPEED_HIGH:
        return 100U;
    case DOG_GAIT_SPEED_MID:
    default:
        return 125U;
    }
}

static uint32_t march_walk_leg_pause_ms(void)
{
    switch (s_march.active_speed_profile) {
    case DOG_GAIT_SPEED_LOW:
        return 150U;
    case DOG_GAIT_SPEED_HIGH:
        return 50U;
    case DOG_GAIT_SPEED_MID:
    default:
        return 75U;
    }
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

float dog_mit_gait_swing_height_mm(void)
{
    return gait_speed_profile()->swing_height_mm;
}

uint32_t dog_mit_gait_touchdown_dwell_ms(void)
{
    return gait_speed_profile()->touchdown_dwell_ms;
}

uint32_t dog_mit_gait_diagonal_stagger_ms(void)
{
    return gait_speed_profile()->diagonal_stagger_ms;
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

static uint8_t motor_tx_guard_take(void)
{
    if (s_motor_tx_guard == nullptr) {
        return 0U;
    }
    return (xSemaphoreTake(s_motor_tx_guard, portMAX_DELAY) == pdTRUE) ? 1U : 0U;
}

static void motor_tx_guard_give(void)
{
    if (s_motor_tx_guard != nullptr) {
        (void)xSemaphoreGive(s_motor_tx_guard);
    }
}

static uint8_t motor_safety_token_acquire(uint32_t *generation)
{
    if ((generation == nullptr) || (motor_tx_guard_take() == 0U)) {
        return 0U;
    }
    const uint8_t allowed = ((s_safety_latched == 0U) &&
                             (s_safety_external_inhibit == 0U) &&
                             (s_control_disabled == 0U) &&
                             (s_mechanical_idle_requested == 0U)) ? 1U : 0U;
    *generation = s_safety_generation;
    motor_tx_guard_give();
    return allowed;
}

static uint8_t motor_safety_token_valid(uint32_t generation)
{
    if (motor_tx_guard_take() == 0U) {
        return 0U;
    }
    const uint8_t valid = ((s_safety_latched == 0U) &&
                           (s_safety_external_inhibit == 0U) &&
                           (s_control_disabled == 0U) &&
                           (s_mechanical_idle_requested == 0U) &&
                           (s_safety_generation == generation)) ? 1U : 0U;
    motor_tx_guard_give();
    return valid;
}

static uint8_t motor_safety_token_guard_take(uint32_t generation)
{
    if (motor_tx_guard_take() == 0U) {
        return 0U;
    }
    if ((s_safety_latched != 0U) || (s_safety_external_inhibit != 0U) ||
        (s_control_disabled != 0U) || (s_mechanical_idle_requested != 0U) ||
        (s_safety_generation != generation)) {
        motor_tx_guard_give();
        return 0U;
    }
    return 1U;
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

static uint8_t selected_motor_mask(void)
{
    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < selected_count(); ++i) {
        const uint8_t index = selected_index(i);
        if (index < DOG_MOTOR_COUNT) {
            mask |= (uint8_t)(1U << index);
        }
    }
    return mask;
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

static uint8_t mit_swing_motor_mask_for_legs(uint8_t leg_mask)
{
    uint8_t motor_mask = 0U;
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        if ((leg_mask & (uint8_t)(1U << leg)) == 0U) {
            continue;
        }
        const uint8_t hip = leg_joint_index(leg, DOG_JOINT_HIP);
        const uint8_t knee = leg_joint_index(leg, DOG_JOINT_KNEE);
        if (hip < DOG_MOTOR_COUNT) {
            motor_mask |= (uint8_t)(1U << hip);
        }
        if (knee < DOG_MOTOR_COUNT) {
            motor_mask |= (uint8_t)(1U << knee);
        }
    }
    return motor_mask;
}

static void mit_set_mixed_swing_leg_mask(uint8_t leg_mask)
{
    const uint8_t new_motor_mask = mit_swing_motor_mask_for_legs(leg_mask);
    const uint8_t changed_motor_mask = s_mit_swing_motor_mask ^ new_motor_mask;
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((changed_motor_mask & (uint8_t)(1U << i)) != 0U) {
            mit_reset_motor_integrator(i);
        }
    }
    s_mit_swing_motor_mask = new_motor_mask;
    s_mit_mixed_pid_active = (new_motor_mask != 0U) ? 1U : 0U;
}

static void mit_set_mixed_swing_legs(uint8_t leg_a, uint8_t leg_b)
{
    uint8_t leg_mask = 0U;
    if (leg_a < DOG_LEG_COUNT) {
        leg_mask |= (uint8_t)(1U << leg_a);
    }
    if (leg_b < DOG_LEG_COUNT) {
        leg_mask |= (uint8_t)(1U << leg_b);
    }
    mit_set_mixed_swing_leg_mask(leg_mask);
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
    s_mechanical_pose_requested = 0U;
    s_mechanical_pose_ready = 0U;
    s_mechanical_pose_mask = 0U;
    s_lower_state = DOG_LOWER_IDLE;
    s_lower_t0_ms = 0U;
    s_lower_settle_since_ms = 0U;
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((s_mit_boot_ok[i] != 0U) && (s_encoder_turn_valid[i] != 0U)) {
            send_mit_zero_effort(i, user_rad(i));
        }
    }

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
    uint32_t safety_generation = 0U;
    uint8_t idle_mask = 0U;
    uint8_t mit_hold_mask = 0U;
    const Dog_Control_Loop_Mode previous_loop_mode = s_control_loop_mode;
    WheelDrive_HoldIfEnabled();
    if (motor_safety_token_acquire(&safety_generation) == 0U) {
        return;
    }

    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        const float current_deg = user_deg(i);
        const float current_turn = g_mw_motor_data[i].encoderEstimates.encoderPosEstimate;
        const uint8_t controller_ready = ((s_mit_boot_ok[i] != 0U) ||
                                          (s_motor_configured[i] != 0U)) ? 1U : 0U;
        if ((controller_ready == 0U) || (s_motor_online[i] == 0U) ||
            (s_encoder_est_fresh[i] == 0U) || (motor_closed_loop(i) == 0U) ||
            (motor_has_fault(i) != 0U) || (!isfinite(current_deg)) ||
            (!isfinite(current_turn))) {
            s_mit_boot_ok[i] = 0U;
            s_motor_configured[i] = 0U;
            idle_mask |= (uint8_t)(1U << i);
            continue;
        }
        s_target_deg[i] = current_deg;
        s_target_turn[i] = current_turn;
        mit_reset_motor_integrator(i);
        if (s_mit_boot_ok[i] != 0U) {
            mit_hold_mask |= (uint8_t)(1U << i);
        }
    }

    if (motor_safety_token_guard_take(safety_generation) == 0U) {
        return;
    }
    s_mit_debug_active = 0U;
    s_mit_fault_hold_active = 1U;
    if (mit_hold_mask != 0U) {
        s_control_loop_mode = DOG_CTRL_LOOP_MIT_PID;
        s_position_tx_enabled = 1U;
    } else {
        s_position_tx_enabled = (previous_loop_mode == DOG_CTRL_LOOP_MIT_PID) ? 1U : 0U;
    }
    s_last_command_tick_ms = 0U;
    teach_hold_stop();
    motor_tx_guard_give();

    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((idle_mask & (uint8_t)(1U << i)) != 0U) {
            MWSetAxisState(g_dog_motor_config[i].bus,
                           g_dog_motor_config[i].node_id,
                           MW_AXIS_STATE_IDLE);
        }
    }
}

static void mit_debug_abort_control(const char *reason)
{
    if (s_mit_fault_hold_active == 0U) {
        DebugUart_Printf("PROTECT %s -> hold healthy legs, idle failed axes\r\n", reason);
    }
    mit_debug_fault_hold();
}

void dog_mit_protect_hold(void)
{
    mit_debug_fault_hold();
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

static uint8_t motor_heartbeat_fresh(uint8_t index, uint32_t now)
{
    return ((index < DOG_MOTOR_COUNT) && (s_motor_online[index] != 0U) &&
            (s_last_heartbeat_tick_ms[index] != 0U) &&
            ((uint32_t)(now - s_last_heartbeat_tick_ms[index]) <= DOG_HEARTBEAT_TIMEOUT_MS)) ? 1U : 0U;
}

static uint8_t motor_encoder_fresh(uint8_t index, uint32_t now)
{
    return ((index < DOG_MOTOR_COUNT) && (s_encoder_est_fresh[index] != 0U) &&
            (s_last_encoder_tick_ms[index] != 0U) &&
            ((uint32_t)(now - s_last_encoder_tick_ms[index]) <= DOG_ENCODER_FEEDBACK_TIMEOUT_MS)) ? 1U : 0U;
}

static uint8_t motor_ready(uint8_t index)
{
    const uint32_t now = HAL_GetTick();
    return ((index < DOG_MOTOR_COUNT) &&
            (motor_heartbeat_fresh(index, now) != 0U) &&
            (motor_closed_loop(index) != 0U) &&
            (motor_has_fault(index) == 0U)) ? 1U : 0U;
}

static float command_turn_from_user_deg(uint8_t index, float deg)
{
    if (index >= DOG_MOTOR_COUNT) return 0.0f;
    Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    float limited = deg;
    if (!isfinite(limited)) {
        limited = isfinite(s_target_deg[index]) ? s_target_deg[index] : 0.0f;
    }
    limited = clampf(limited, cfg->min_deg, cfg->max_deg);
    s_target_deg[index] = limited;
    return s_zero_offset_turn[index] +
           ((limited * cfg->direction + cfg->zero_offset_deg) * DOG_DEG_TO_TURN * encoder_gear_ratio(index));
}

static uint8_t mw_send_can_frame(FDCAN_HandleTypeDef *h, uint8_t id_type,
                                 uint32_t can_id, uint8_t *data, uint8_t data_size)
{
    if (id_type == DOG_CAN_ID_STANDARD) {
        return fdcan_send_data_stand(h, can_id, data, data_size);
    }
    return fdcan_send_data_Exten(h, can_id, data, data_size);
}

static uint8_t mw_command_is_read_only(uint8_t cmd)
{
    switch ((MW_CMD_ID)cmd) {
    case MW_GET_ERROR_CMD:
    case MW_RXSDO_CMD:
    case MW_GET_ENCODER_ESTIMATES_CMD:
    case MW_GET_ENCODER_COUNT_CMD:
    case MW_GET_IQ_CMD:
    case MW_GET_BUS_VOLTAGE_CURRENT_CMD:
    case MW_GET_TORQUES_CMD:
    case MW_GET_POWERS_CMD:
        return 1U;
    default:
        return 0U;
    }
}

static uint8_t mw_tx_allowed_locked(uint8_t cmd)
{
    if (cmd == (uint8_t)MW_ESTOP_CMD) {
        return 1U;
    }
    if (s_mechanical_idle_requested != 0U) {
        return (cmd == (uint8_t)MW_SET_AXIS_STATE_CMD) ? 1U : 0U;
    }
    if (s_estop_pending_mask != 0U) {
        return 0U;
    }
    if (cmd == (uint8_t)MW_CLEAR_ERRORS_CMD) {
        if (s_safety_external_inhibit != 0U) {
            return 0U;
        }
        return ((s_safety_latched == 0U) ||
                (s_safety_rearm_requested != 0U)) ? 1U : 0U;
    }
    if (mw_command_is_read_only(cmd) != 0U) {
        return 1U;
    }
    return ((s_safety_latched == 0U) &&
            (s_safety_external_inhibit == 0U)) ? 1U : 0U;
}

static void mw_record_tx_result(uint8_t bus, uint8_t status)
{
    const uint8_t di = bus_to_diag_index(bus);
    if ((status != 0U) && (di < 2U)) {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_can_tx_drop_count[di]++;
        __set_PRIMASK(primask);
    }
}

static void mw_sender(uint8_t busId, uint8_t canId, uint8_t *data, uint8_t dataSize)
{
    FDCAN_HandleTypeDef *h = bus_handle(busId);
    if (h == nullptr) return;

    const uint8_t index = motor_index(busId, (uint8_t)(canId >> 5));
    uint8_t id_type = DOG_CAN_ID_EXTENDED;
    if ((index < DOG_MOTOR_COUNT) && (s_motor_can_id_type[index] != DOG_CAN_ID_UNKNOWN)) {
        id_type = s_motor_can_id_type[index];
    }

    if (motor_tx_guard_take() == 0U) {
        mw_record_tx_result(busId, 1U);
        return;
    }
    const uint8_t cmd = (uint8_t)(canId & 0x1FU);
    uint8_t status = 1U;
    if (mw_tx_allowed_locked(cmd) != 0U) {
        status = mw_send_can_frame(h, id_type, canId, data, dataSize);
    }
    motor_tx_guard_give();
    mw_record_tx_result(busId, status);
}

static void mw_query_encoder_frames(uint8_t index, uint8_t include_count)
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
    const uint8_t id_type = s_motor_can_id_type[index];

    if (id_type == DOG_CAN_ID_UNKNOWN) {
        mw_record_tx_result(cfg->bus, mw_send_can_frame(h, DOG_CAN_ID_EXTENDED, id_est, tx, 8U));
        mw_record_tx_result(cfg->bus, mw_send_can_frame(h, DOG_CAN_ID_STANDARD, id_est, tx, 8U));
        if (include_count != 0U) {
            mw_record_tx_result(cfg->bus, mw_send_can_frame(h, DOG_CAN_ID_EXTENDED, id_cnt, tx, 8U));
            mw_record_tx_result(cfg->bus, mw_send_can_frame(h, DOG_CAN_ID_STANDARD, id_cnt, tx, 8U));
        }
        return;
    }

    mw_record_tx_result(cfg->bus, mw_send_can_frame(h, id_type, id_est, tx, 8U));
    if (include_count != 0U) {
        mw_record_tx_result(cfg->bus, mw_send_can_frame(h, id_type, id_cnt, tx, 8U));
    }
}

static void mw_query_encoder(uint8_t index)
{
    mw_query_encoder_frames(index, 1U);
}

static void mw_query_encoder_estimate(uint8_t index)
{
    mw_query_encoder_frames(index, 0U);
}

static uint8_t reject_encoder_estimate(uint8_t index)
{
    if ((index < DOG_MOTOR_COUNT) && (s_encoder_turn_valid[index] != 0U)) {
        g_mw_motor_data[index].encoderEstimates.encoderPosEstimate = s_encoder_turn_filt[index];
    }
    if (index < DOG_MOTOR_COUNT) {
        g_mw_motor_data[index].encoderEstimates.encoderVelEstimate = 0.0f;
    }
    return 0U;
}

static uint8_t encoder_filter_estimate(uint8_t index)
{
    if (index >= DOG_MOTOR_COUNT) {
        return 0U;
    }

    float new_turn = g_mw_motor_data[index].encoderEstimates.encoderPosEstimate;
    const float new_vel = g_mw_motor_data[index].encoderEstimates.encoderVelEstimate;

    const float velocity_plausibility_limit =
        (g_dog_mit_motor_limits.vel_limit_turn_s * 2.0f) + 1.0f;
    if ((!isfinite(new_turn)) || (!isfinite(new_vel)) ||
        (fabsf(new_turn) > 10000.0f) ||
        (fabsf(new_vel) > velocity_plausibility_limit)) {
        return reject_encoder_estimate(index);
    }

    if (s_encoder_turn_valid[index] == 0U) {
        s_encoder_turn_filt[index] = new_turn;
        s_encoder_turn_valid[index] = 1U;
        return 1U;
    }

    float delta_turn = new_turn - s_encoder_turn_filt[index];
    if (fabsf(delta_turn) > 0.5f) {
        new_turn -= roundf(delta_turn);
        delta_turn = new_turn - s_encoder_turn_filt[index];
    }
    if ((!isfinite(new_turn)) || (!isfinite(delta_turn)) || (fabsf(delta_turn) > 0.5001f)) {
        return reject_encoder_estimate(index);
    }

    if (s_last_encoder_tick_ms[index] != 0U) {
        const uint32_t age_ms = (uint32_t)(HAL_GetTick() - s_last_encoder_tick_ms[index]);
        const float max_delta_turn =
            (g_dog_mit_motor_limits.vel_limit_turn_s * (float)age_ms * 0.001f) + 0.05f;
        if (fabsf(delta_turn) > max_delta_turn) {
            return reject_encoder_estimate(index);
        }
    }

    s_encoder_turn_filt[index] = new_turn;
    g_mw_motor_data[index].encoderEstimates.encoderPosEstimate = new_turn;
    return 1U;
}

static void mw_notifier(uint8_t busId, uint8_t nodeId, MW_CMD_ID cmdId)
{
    uint8_t index = motor_index(busId, nodeId);
    if (index >= DOG_MOTOR_COUNT) return;

    const uint32_t now = HAL_GetTick();
    if (cmdId == MW_HEARTBEAT_CMD) {
        s_last_heartbeat_tick_ms[index] = now;
        s_heartbeat_rx_count[index]++;
    }

    if ((cmdId == MW_GET_ENCODER_ESTIMATES_CMD) || (cmdId == MW_GET_ENCODER_COUNT_CMD)) {
        s_encoder_rx_count[index]++;
        if ((cmdId == MW_GET_ENCODER_ESTIMATES_CMD) && (motor_closed_loop(index) != 0U) &&
            (encoder_filter_estimate(index) != 0U)) {
            s_encoder_est_fresh[index] = 1U;
            s_last_encoder_tick_ms[index] = now;
        }
    }

    uint8_t diag = bus_to_diag_index(busId);
    s_motor_online[index] = 1U;
    s_last_rx_tick_ms[index] = now;
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
    const uint8_t di = bus_to_diag_index(cfg->bus);
    const uint32_t drops_before = s_can_tx_drop_count[di];
    MWSetControllerMode(cfg->bus, cfg->node_id, MW_POSITION_CONTROL, MW_TRAPEZOIDAL_CURVE_INPUT);
    MWSetTrajVelLimit(cfg->bus, cfg->node_id, DOG_TRAJ_VEL_LIMIT);
    MWSetTrajAccelLimits(cfg->bus, cfg->node_id, DOG_TRAJ_ACCEL_LIMIT, DOG_TRAJ_DECEL_LIMIT);
    MWSetTrajInertia(cfg->bus, cfg->node_id, DOG_TRAJ_INERTIA);
    MWSetLimits(cfg->bus, cfg->node_id, DOG_TRAJ_VEL_LIMIT, g_dog_mit_motor_limits.current_limit_a);
    s_motor_configured[index] = (s_can_tx_drop_count[di] == drops_before) ? 1U : 0U;
}

static void configure_motor_mit(uint8_t index)
{
    if ((index >= DOG_MOTOR_COUNT) || (s_motor_online[index] == 0U) || (motor_has_fault(index) != 0U)) {
        return;
    }
    Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    const uint8_t di = bus_to_diag_index(cfg->bus);
    const uint32_t drops_before = s_can_tx_drop_count[di];
    MWSetControllerMode(cfg->bus, cfg->node_id, MW_TORQUE_CONTROL, MW_MIT_INPUT);
    MWSetLimits(cfg->bus, cfg->node_id, g_dog_mit_motor_limits.vel_limit_turn_s, g_dog_mit_motor_limits.current_limit_a);
    s_motor_configured[index] = (s_can_tx_drop_count[di] == drops_before) ? 1U : 0U;
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
    if ((!isfinite(err_deg)) || (!isfinite(dt_s)) || (dt_s <= 0.0f) ||
        (!isfinite(pid->kp_a_per_deg)) || (!isfinite(pid->ki_a_per_deg_s)) ||
        (!isfinite(pid->kd_a_per_dps)) || (!isfinite(pid->output_limit_a))) {
        mit_reset_motor_integrator(index);
        return 0.0f;
    }
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
    if (!isfinite(current_a)) {
        mit_reset_motor_integrator(index);
        return 0.0f;
    }

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
    uint32_t safety_generation = 0U;
    if ((index >= DOG_MOTOR_COUNT) ||
        (motor_safety_token_acquire(&safety_generation) == 0U)) {
        return;
    }

    Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    if (!isfinite(torque_nm)) {
        torque_nm = 0.0f;
    }
    const float current_torque_limit_nm =
        g_dog_mit_motor_limits.current_limit_a * g_dog_mit_motor_limits.torque_nm_per_a;
    const float torque_limit_nm = (current_torque_limit_nm < DOG_MIT_TORQUE_LIMIT_NM) ?
                                  current_torque_limit_nm : DOG_MIT_TORQUE_LIMIT_NM;
    torque_nm = clampf(torque_nm, -torque_limit_nm, torque_limit_nm);
    MW_MIT_CTRL mit = {};
    mit.pos = 0.0;
    mit.vel = 0.0;
    mit.kp = 0.0;
    mit.kd = 0.0;
    mit.torque = torque_nm;
    (void)MWMitControl(cfg->bus, cfg->node_id, &mit);
    if (motor_safety_token_valid(safety_generation) == 0U) {
        queue_motor_estop(index);
    }
}

static void send_mit_zero_effort(uint8_t index, float pos_rad)
{
    uint32_t safety_generation = 0U;
    if ((index >= DOG_MOTOR_COUNT) ||
        (motor_safety_token_acquire(&safety_generation) == 0U)) {
        return;
    }

    Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    if (!isfinite(pos_rad)) {
        pos_rad = 0.0f;
    }
    MW_MIT_CTRL mit = {};
    mit.pos = clampf(pos_rad, -DOG_MIT_POS_RAD_LIMIT, DOG_MIT_POS_RAD_LIMIT);
    mit.vel = 0.0;
    mit.kp = 0.0;
    mit.kd = 0.0;
    mit.torque = 0.0;
    (void)MWMitControl(cfg->bus, cfg->node_id, &mit);
    if (motor_safety_token_valid(safety_generation) == 0U) {
        queue_motor_estop(index);
    }
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
    uint32_t safety_generation = 0U;
    if ((motor_safety_token_acquire(&safety_generation) == 0U) ||
        ((s_mit_debug_active == 0U) && (s_mit_fault_hold_active == 0U))) {
        return;
    }

    uint8_t command_mask = 0U;
    float torque_nm_cmd[DOG_MOTOR_COUNT] = {};
    float pos_rad_cmd[DOG_MOTOR_COUNT] = {};

    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        uint8_t commanded = 0U;
        if (s_mit_debug_active != 0U) {
            commanded = motor_is_selected(i);
        } else if ((s_mit_fault_hold_active != 0U) && (s_mit_boot_ok[i] != 0U)) {
            commanded = 1U;
        }
        if (commanded == 0U) {
            continue;
        }

        const uint8_t heartbeat_fresh = ((s_last_heartbeat_tick_ms[i] != 0U) &&
            ((uint32_t)(now - s_last_heartbeat_tick_ms[i]) <= DOG_HEARTBEAT_TIMEOUT_MS)) ? 1U : 0U;
        const uint8_t encoder_fresh = ((s_encoder_est_fresh[i] != 0U) &&
            ((uint32_t)(now - s_last_encoder_tick_ms[i]) <= DOG_ENCODER_FEEDBACK_TIMEOUT_MS)) ? 1U : 0U;
        if ((s_mit_boot_ok[i] == 0U) || (s_motor_online[i] == 0U) ||
            (heartbeat_fresh == 0U) || (encoder_fresh == 0U) ||
            (motor_closed_loop(i) == 0U) || (motor_has_fault(i) != 0U)) {
            mit_debug_fault_hold();
            return;
        }

        teach_hold_update_target(i, now);
        const float user_now_deg = user_deg(i);
        const float user_now_rad = user_rad(i);
        const float err_deg = s_target_deg[i] - user_now_deg;
        if ((!isfinite(user_now_deg)) || (!isfinite(user_now_rad)) ||
            (!isfinite(s_target_deg[i])) || (!isfinite(err_deg))) {
            mit_debug_fault_hold();
            return;
        }

        if ((s_mit_debug_active != 0U) && (fabsf(err_deg) > DOG_MIT_DEBUG_SAFETY_ERR_DEG)) {
            if (VofaPid_IsEnabled() == 0U) {
                Dog_Motor_Config *cfg = &g_dog_motor_config[i];
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
            mit_debug_fault_hold();
            return;
        }

        pos_rad_cmd[i] = clampf(user_now_rad, -DOG_MIT_POS_RAD_LIMIT, DOG_MIT_POS_RAD_LIMIT);
        command_mask |= (uint8_t)(1U << i);
    }

    float dt_s = (float)DOG_CMD_PERIOD_MS * 0.001f;
    if (s_mit_last_pid_ms != 0U) {
        dt_s = (float)(now - s_mit_last_pid_ms) * 0.001f;
        if ((dt_s <= 0.0f) || (dt_s > 0.1f)) {
            dt_s = (float)DOG_CMD_PERIOD_MS * 0.001f;
        }
    }
    s_mit_last_pid_ms = now;

    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((command_mask & (uint8_t)(1U << i)) == 0U) {
            continue;
        }
        Dog_Motor_Config *cfg = &g_dog_motor_config[i];
        const float current_a = mit_ang_pid_compute_current_a(i, dt_s);
        torque_nm_cmd[i] = clampf(current_a * g_dog_mit_motor_limits.torque_nm_per_a * cfg->torque_direction,
                                  -DOG_MIT_TORQUE_LIMIT_NM, DOG_MIT_TORQUE_LIMIT_NM);
        if (!isfinite(torque_nm_cmd[i])) {
            mit_debug_fault_hold();
            return;
        }
    }

    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((command_mask & (uint8_t)(1U << i)) == 0U) {
            continue;
        }
        if (motor_safety_token_valid(safety_generation) == 0U) {
            queue_motor_estop(i);
            return;
        }
        Dog_Motor_Config *cfg = &g_dog_motor_config[i];
        MW_MIT_CTRL mit = {};
        mit.pos = pos_rad_cmd[i];
        mit.vel = 0.0;
        mit.kp = 0.0;
        mit.kd = 0.0;
        mit.torque = torque_nm_cmd[i];
        (void)MWMitControl(cfg->bus, cfg->node_id, &mit);
        if (motor_safety_token_valid(safety_generation) == 0U) {
            queue_motor_estop(i);
            return;
        }
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
    if ((s_mit_torque_test_active == 0U) || (s_safety_latched != 0U)) {
        return;
    }

    const uint8_t index = s_mit_torque_test_index;
    if ((index < DOG_MOTOR_COUNT) &&
        (mit_probe_bus_tx_busy(g_dog_motor_config[index].bus) != 0U)) {
        return;
    }
    const uint8_t heartbeat_fresh = ((index < DOG_MOTOR_COUNT) &&
        (s_last_heartbeat_tick_ms[index] != 0U) &&
        ((uint32_t)(now - s_last_heartbeat_tick_ms[index]) <= DOG_HEARTBEAT_TIMEOUT_MS)) ? 1U : 0U;
    const uint8_t encoder_fresh = ((index < DOG_MOTOR_COUNT) &&
        (s_encoder_est_fresh[index] != 0U) &&
        ((uint32_t)(now - s_last_encoder_tick_ms[index]) <= DOG_ENCODER_FEEDBACK_TIMEOUT_MS)) ? 1U : 0U;
    if ((index >= DOG_MOTOR_COUNT) || (s_motor_online[index] == 0U) ||
        (heartbeat_fresh == 0U) || (encoder_fresh == 0U) ||
        (motor_closed_loop(index) == 0U) || (motor_has_fault(index) != 0U)) {
        mit_debug_abort_control("torque test feedback/fault");
        return;
    }

    static uint32_t s_last_ms = 0U;
    if ((uint32_t)(now - s_last_ms) < DOG_CMD_PERIOD_MS) {
        return;
    }
    s_last_ms = now;

    send_mit_fixed_torque(index, s_mit_torque_test_nm);
}

static void send_mit_probe_keepalive(uint32_t now)
{
    if (s_safety_latched != 0U) {
        return;
    }

    static uint32_t s_last_ms = 0U;
    if ((uint32_t)(now - s_last_ms) < DOG_CMD_PERIOD_MS) {
        return;
    }
    s_last_ms = now;

    const uint8_t buses[2U] = {DOG_CAN_FRONT_BUS, DOG_CAN_REAR_BUS};
    for (uint8_t di = 0U; di < 2U; ++di) {
        const uint8_t bus_mask = (uint8_t)(1U << di);
        if ((s_mit_probe_tx_busy_mask & bus_mask) != 0U) {
            continue;
        }

        const uint8_t start = (uint8_t)(s_mit_probe_keepalive_cursor[di] % DOG_MOTOR_COUNT);
        for (uint8_t offset = 0U; offset < DOG_MOTOR_COUNT; ++offset) {
            const uint8_t index = (uint8_t)((start + offset) % DOG_MOTOR_COUNT);
            if ((g_dog_motor_config[index].bus == buses[di]) &&
                (s_motor_mit_probe_active[index] != 0U) &&
                (motor_closed_loop(index) != 0U)) {
                s_mit_probe_keepalive_cursor[di] = (uint8_t)((index + 1U) % DOG_MOTOR_COUNT);
                send_mit_zero_effort(index, 0.0f);
                break;
            }
        }
    }
}

static uint8_t motor_blocking_service(uint32_t *now_out)
{
    WheelDrive_HoldIfEnabled();
    control_task_safety_poll();
    fdcan_poll_rx(&hfdcan1);
    fdcan_poll_rx(&hfdcan2);

    const uint32_t now = HAL_GetTick();
    const uint8_t feedback_ok = motor_feedback_health_tick(now);
    motor_safety_tick(now);
    send_mit_probe_keepalive(now);

    if (now_out != nullptr) {
        *now_out = now;
    }
    return ((feedback_ok != 0U) && (s_safety_latched == 0U) &&
            (s_safety_external_inhibit == 0U) &&
            (s_control_disabled == 0U)) ? 1U : 0U;
}

static uint8_t mit_probe_bus_mask(uint8_t bus)
{
    if (bus == DOG_CAN_FRONT_BUS) {
        return 0x01U;
    }
    if (bus == DOG_CAN_REAR_BUS) {
        return 0x02U;
    }
    return 0U;
}

static uint8_t mit_probe_bus_tx_busy(uint8_t bus)
{
    const uint8_t mask = mit_probe_bus_mask(bus);
    return ((mask != 0U) && ((s_mit_probe_tx_busy_mask & mask) != 0U)) ? 1U : 0U;
}

static void mit_probe_tx_window_end(uint8_t bus)
{
    const uint8_t mask = mit_probe_bus_mask(bus);
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_mit_probe_tx_busy_mask &= (uint8_t)~mask;
    __set_PRIMASK(primask);
}

static uint8_t mit_probe_tx_window_wait(uint8_t bus, uint32_t quiet_ms)
{
    FDCAN_HandleTypeDef *h = bus_handle(bus);
    if (h == nullptr) {
        return 0U;
    }

    const uint32_t fifo_capacity = h->Init.TxFifoQueueElmtsNbr;
    if (fifo_capacity == 0U) {
        return 0U;
    }

    const uint32_t wait_started_ms = HAL_GetTick();
    while (fdcan_tx_free_level(h) < fifo_capacity) {
        if ((uint32_t)(HAL_GetTick() - wait_started_ms) >= DOG_MIT_BOOT_TX_TIMEOUT_MS) {
            return 0U;
        }
        if (motor_blocking_service(nullptr) == 0U) {
            return 0U;
        }
        HAL_Delay(1U);
    }

    const uint32_t quiet_started_ms = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - quiet_started_ms) < quiet_ms) {
        if (motor_blocking_service(nullptr) == 0U) {
            return 0U;
        }
        HAL_Delay(1U);
    }
    return 1U;
}

static uint8_t mit_probe_tx_window_begin(uint8_t bus)
{
    const uint8_t mask = mit_probe_bus_mask(bus);
    if (mask == 0U) {
        return 0U;
    }

    const uint32_t acquire_started_ms = HAL_GetTick();
    while (1) {
        uint8_t acquired = 0U;
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        if ((s_mit_probe_tx_busy_mask & mask) == 0U) {
            s_mit_probe_tx_busy_mask |= mask;
            acquired = 1U;
        }
        __set_PRIMASK(primask);

        if (acquired != 0U) {
            break;
        }
        if ((uint32_t)(HAL_GetTick() - acquire_started_ms) >= DOG_MIT_BOOT_TX_TIMEOUT_MS) {
            return 0U;
        }
        if (motor_blocking_service(nullptr) == 0U) {
            return 0U;
        }
        HAL_Delay(1U);
    }

    if (mit_probe_tx_window_wait(bus, 0U) == 0U) {
        mit_probe_tx_window_end(bus);
        return 0U;
    }
    return 1U;
}

static uint8_t mit_probe_tx_window_finish(uint8_t bus, uint32_t quiet_ms)
{
    const uint8_t ok = mit_probe_tx_window_wait(bus, quiet_ms);
    mit_probe_tx_window_end(bus);
    return ok;
}

static uint8_t configure_motor_mit_for_boot(uint8_t index)
{
    if (index >= DOG_MOTOR_COUNT) {
        return 0U;
    }

    const uint8_t bus = g_dog_motor_config[index].bus;
    if (mit_probe_tx_window_begin(bus) == 0U) {
        return 0U;
    }

    configure_motor_mit(index);
    const uint8_t configured = s_motor_configured[index];
    const uint8_t drained = mit_probe_tx_window_finish(bus, DOG_MIT_BOOT_BUS_QUIET_MS);
    return ((configured != 0U) && (drained != 0U)) ? 1U : 0U;
}

static uint8_t wait_motor_encoder(uint8_t index, uint32_t timeout_ms)
{
    if ((index >= DOG_MOTOR_COUNT) || (s_safety_latched != 0U)) {
        return 0U;
    }

    uint32_t t0 = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - t0) < timeout_ms) {
        if (motor_blocking_service(nullptr) == 0U) {
            return 0U;
        }
        send_mit_zero_effort(index, 0.0f);
        mw_query_encoder_estimate(index);
        if ((motor_closed_loop(index) != 0U) && (s_encoder_est_fresh[index] != 0U)) {
            return 1U;
        }
        HAL_Delay(5U);
    }

    if (motor_blocking_service(nullptr) == 0U) {
        return 0U;
    }
    return ((motor_closed_loop(index) != 0U) && (s_encoder_est_fresh[index] != 0U)) ? 1U : 0U;
}

static uint8_t enter_mit_probe_closed_loop(uint8_t index)
{
    uint32_t safety_generation = 0U;
    if ((index >= DOG_MOTOR_COUNT) ||
        (motor_safety_token_acquire(&safety_generation) == 0U) ||
        (motor_heartbeat_fresh(index, HAL_GetTick()) == 0U) ||
        (motor_has_fault(index) != 0U)) {
        return 0U;
    }

    Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    const uint8_t di = bus_to_diag_index(cfg->bus);
    if (mit_probe_tx_window_begin(cfg->bus) == 0U) {
        return 0U;
    }
    if (motor_safety_token_valid(safety_generation) == 0U) {
        mit_probe_tx_window_end(cfg->bus);
        return 0U;
    }
    const uint32_t drops_before = s_can_tx_drop_count[di];
    MWSetLimits(cfg->bus, cfg->node_id,
                g_dog_mit_motor_limits.vel_limit_turn_s,
                DOG_MIT_PROBE_CURRENT_LIMIT_A);
    MWSetControllerMode(cfg->bus, cfg->node_id, MW_TORQUE_CONTROL, MW_MIT_INPUT);
    send_mit_zero_effort(index, 0.0f);
    if ((s_can_tx_drop_count[di] != drops_before) ||
        (motor_safety_token_valid(safety_generation) == 0U)) {
        if (motor_safety_token_valid(safety_generation) == 0U) {
            queue_motor_estop(index);
        }
        s_motor_mit_probe_active[index] = 0U;
        mit_probe_tx_window_end(cfg->bus);
        return 0U;
    }

    MWSetAxisState(cfg->bus, cfg->node_id, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
    if ((s_can_tx_drop_count[di] != drops_before) ||
        (motor_safety_token_valid(safety_generation) == 0U)) {
        queue_motor_estop(index);
        s_motor_mit_probe_active[index] = 0U;
        mit_probe_tx_window_end(cfg->bus);
        return 0U;
    }
    if (mit_probe_tx_window_finish(cfg->bus, 0U) == 0U) {
        s_motor_mit_probe_active[index] = 0U;
        return 0U;
    }
    if (motor_safety_token_guard_take(safety_generation) == 0U) {
        queue_motor_estop(index);
        return 0U;
    }
    s_motor_configured[index] = 0U;
    s_encoder_est_fresh[index] = 0U;
    s_motor_final_mode_pending[index] = DOG_FINAL_MODE_NONE;
    s_motor_mit_probe_active[index] = 1U;
    motor_tx_guard_give();
    fdcan_poll_rx(&hfdcan1);
    fdcan_poll_rx(&hfdcan2);
    return 1U;
}

static uint8_t request_mit_probe_for_angle(uint8_t index, uint32_t encoder_wait_ms)
{
    if ((index >= DOG_MOTOR_COUNT) || (s_safety_latched != 0U) ||
        (s_motor_online[index] == 0U) || (motor_has_fault(index) != 0U)) {
        return 0U;
    }

    s_encoder_est_fresh[index] = 0U;
    s_encoder_turn_valid[index] = 0U;
    if ((s_motor_mit_probe_active[index] == 0U) || (motor_closed_loop(index) == 0U)) {
        if (enter_mit_probe_closed_loop(index) == 0U) {
            return 0U;
        }
    }

    uint8_t have_encoder = ((s_encoder_est_fresh[index] != 0U) && (motor_closed_loop(index) != 0U)) ? 1U : 0U;
    if ((have_encoder == 0U) && (encoder_wait_ms > 0U)) {
        have_encoder = wait_motor_encoder(index, encoder_wait_ms);
    }

    if (motor_blocking_service(nullptr) == 0U) {
        return 0U;
    }
    if ((have_encoder == 0U) || (motor_closed_loop(index) == 0U) ||
        (motor_heartbeat_fresh(index, HAL_GetTick()) == 0U)) {
        return 2U;
    }

    return 1U;
}

static uint8_t request_closed_loop_with_position_hold(uint8_t index)
{
    uint32_t safety_generation = 0U;
    const uint32_t now = HAL_GetTick();
    if ((index >= DOG_MOTOR_COUNT) ||
        (motor_safety_token_acquire(&safety_generation) == 0U) ||
        (motor_heartbeat_fresh(index, now) == 0U) ||
        (motor_has_fault(index) != 0U)) {
        return 0U;
    }
    Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    const uint8_t di = bus_to_diag_index(cfg->bus);

    if ((s_control_loop_mode == DOG_CTRL_LOOP_POSITION) &&
        (s_motor_final_mode_pending[index] == DOG_FINAL_POSITION_WAIT_CLOSED)) {
        if ((motor_closed_loop(index) != 0U) &&
            (motor_encoder_fresh(index, now) != 0U)) {
            if (motor_safety_token_guard_take(safety_generation) == 0U) {
                queue_motor_estop(index);
                return 0U;
            }
            s_motor_mit_probe_active[index] = 0U;
            s_motor_final_mode_pending[index] = DOG_FINAL_MODE_NONE;
            s_motor_loop_requested[index] = 1U;
            motor_tx_guard_give();
            return 1U;
        }

        if (motor_closed_loop(index) == 0U) {
            if (motor_safety_token_guard_take(safety_generation) == 0U) {
                queue_motor_estop(index);
                return 0U;
            }
            s_motor_configured[index] = 0U;
            s_motor_final_mode_pending[index] = DOG_FINAL_POSITION_WAIT_IDLE;
            motor_tx_guard_give();
        }
        return 2U;
    }

    if ((s_control_loop_mode == DOG_CTRL_LOOP_POSITION) &&
        (s_motor_final_mode_pending[index] == DOG_FINAL_POSITION_WAIT_IDLE)) {
        if (g_mw_motor_data[index].heartBeat.currentState != MW_AXIS_STATE_IDLE) {
            if (motor_safety_token_valid(safety_generation) == 0U) {
                queue_motor_estop(index);
                return 0U;
            }
            MWSetAxisState(cfg->bus, cfg->node_id, MW_AXIS_STATE_IDLE);
            if (motor_safety_token_valid(safety_generation) == 0U) {
                queue_motor_estop(index);
                return 0U;
            }
            return 2U;
        }

        if (motor_safety_token_guard_take(safety_generation) == 0U) {
            queue_motor_estop(index);
            return 0U;
        }
        s_position_idle_encoder_rx_baseline[index] = s_encoder_rx_count[index];
        s_motor_final_mode_pending[index] = DOG_FINAL_POSITION_WAIT_IDLE_ENCODER;
        motor_tx_guard_give();
        mw_query_encoder_estimate(index);
        return 2U;
    }

    if ((s_control_loop_mode == DOG_CTRL_LOOP_POSITION) &&
        (s_motor_final_mode_pending[index] == DOG_FINAL_POSITION_WAIT_IDLE_ENCODER)) {
        if (g_mw_motor_data[index].heartBeat.currentState != MW_AXIS_STATE_IDLE) {
            if (motor_safety_token_guard_take(safety_generation) == 0U) {
                queue_motor_estop(index);
                return 0U;
            }
            s_motor_configured[index] = 0U;
            s_motor_final_mode_pending[index] = DOG_FINAL_POSITION_WAIT_IDLE;
            motor_tx_guard_give();
            return 2U;
        }
        if (s_encoder_rx_count[index] == s_position_idle_encoder_rx_baseline[index]) {
            return 2U;
        }
        if (encoder_filter_estimate(index) == 0U) {
            s_position_idle_encoder_rx_baseline[index] = s_encoder_rx_count[index];
            mw_query_encoder_estimate(index);
            return 2U;
        }

        const float idle_hold_turn = g_mw_motor_data[index].encoderEstimates.encoderPosEstimate;
        if ((!isfinite(idle_hold_turn)) ||
            (fdcan_tx_free_level(bus_handle(cfg->bus)) < 7U)) {
            return 2U;
        }
        s_position_hold_turn[index] = idle_hold_turn;

        s_motor_mit_probe_active[index] = 0U;
        configure_motor_position(index);
        if ((s_motor_configured[index] == 0U) ||
            (motor_safety_token_valid(safety_generation) == 0U)) {
            if (motor_safety_token_valid(safety_generation) == 0U) {
                queue_motor_estop(index);
            }
            return (s_safety_latched != 0U) ? 0U : 2U;
        }

        uint32_t drops_before = s_can_tx_drop_count[di];
        MWPosControl(cfg->bus, cfg->node_id, s_position_hold_turn[index], 0, 0);
        if ((s_can_tx_drop_count[di] != drops_before) ||
            (motor_safety_token_valid(safety_generation) == 0U)) {
            if (motor_safety_token_valid(safety_generation) == 0U) {
                queue_motor_estop(index);
                return 0U;
            }
            return 2U;
        }

        s_encoder_est_fresh[index] = 0U;
        drops_before = s_can_tx_drop_count[di];
        MWSetAxisState(cfg->bus, cfg->node_id, MW_AXIS_STATE_CLOSED_LOOP_CONTROL);
        if ((s_can_tx_drop_count[di] != drops_before) ||
            (motor_safety_token_valid(safety_generation) == 0U)) {
            if (motor_safety_token_valid(safety_generation) == 0U) {
                queue_motor_estop(index);
                return 0U;
            }
            return 2U;
        }
        if (motor_safety_token_guard_take(safety_generation) == 0U) {
            queue_motor_estop(index);
            return 0U;
        }
        s_motor_final_mode_pending[index] = DOG_FINAL_POSITION_WAIT_CLOSED;
        motor_tx_guard_give();
        return 2U;
    }

    if ((s_motor_mit_probe_active[index] == 0U) || (motor_closed_loop(index) == 0U)) {
        if (enter_mit_probe_closed_loop(index) == 0U) {
            return 0U;
        }
    }

    if (s_encoder_est_fresh[index] == 0U) {
        if (motor_safety_token_guard_take(safety_generation) == 0U) {
            queue_motor_estop(index);
            return 0U;
        }
        s_motor_loop_requested[index] = 1U;
        motor_tx_guard_give();
        return 2U;
    }

    float hold_turn = g_mw_motor_data[index].encoderEstimates.encoderPosEstimate;
    if (!isfinite(hold_turn)) {
        return 0U;
    }

    if (s_control_loop_mode == DOG_CTRL_LOOP_POSITION) {
        if (motor_safety_token_guard_take(safety_generation) == 0U) {
            queue_motor_estop(index);
            return 0U;
        }
        s_position_hold_turn[index] = hold_turn;
        s_motor_configured[index] = 0U;
        s_motor_final_mode_pending[index] = DOG_FINAL_POSITION_WAIT_IDLE;
        s_motor_loop_requested[index] = 1U;
        motor_tx_guard_give();
        if (motor_safety_token_valid(safety_generation) == 0U) {
            queue_motor_estop(index);
            return 0U;
        }
        MWSetAxisState(cfg->bus, cfg->node_id, MW_AXIS_STATE_IDLE);
        if (motor_safety_token_valid(safety_generation) == 0U) {
            queue_motor_estop(index);
            return 0U;
        }
        return 2U;
    }

    s_target_turn[index] = hold_turn;
    s_target_deg[index] = user_deg(index);
    configure_motor_mit(index);
    if (s_motor_configured[index] == 0U) {
        if (motor_safety_token_guard_take(safety_generation) == 0U) {
            queue_motor_estop(index);
            return 0U;
        }
        s_motor_loop_requested[index] = 1U;
        motor_tx_guard_give();
        return 2U;
    }
    const uint32_t drops_before = s_can_tx_drop_count[di];
    send_mit_zero_effort(index, user_rad(index));
    if ((s_can_tx_drop_count[di] != drops_before) ||
        (motor_safety_token_valid(safety_generation) == 0U)) {
        if (motor_safety_token_valid(safety_generation) == 0U) {
            queue_motor_estop(index);
            return 0U;
        }
        if (motor_safety_token_guard_take(safety_generation) == 0U) {
            queue_motor_estop(index);
            return 0U;
        }
        s_motor_loop_requested[index] = 1U;
        motor_tx_guard_give();
        return 2U;
    }
    if (motor_safety_token_valid(safety_generation) == 0U) {
        queue_motor_estop(index);
        return 0U;
    }
    fdcan_poll_rx(&hfdcan1);
    fdcan_poll_rx(&hfdcan2);
    if (motor_safety_token_guard_take(safety_generation) == 0U) {
        queue_motor_estop(index);
        return 0U;
    }
    s_motor_mit_probe_active[index] = 0U;
    s_motor_final_mode_pending[index] = DOG_FINAL_MODE_NONE;
    s_motor_loop_requested[index] = 1U;
    motor_tx_guard_give();
    return 1U;
}

static uint8_t request_closed_loop(uint8_t index)
{
    return request_closed_loop_with_position_hold(index);
}

static void position_finalization_fast_tick(void)
{
    if ((s_safety_latched != 0U) || (s_control_loop_mode != DOG_CTRL_LOOP_POSITION)) {
        return;
    }
    for (uint8_t index = 0U; index < DOG_MOTOR_COUNT; ++index) {
        const uint8_t stage = s_motor_final_mode_pending[index];
        const uint8_t idle_ready =
            ((stage == DOG_FINAL_POSITION_WAIT_IDLE) &&
             (g_mw_motor_data[index].heartBeat.currentState == MW_AXIS_STATE_IDLE)) ? 1U : 0U;
        const uint8_t encoder_ready =
            ((stage == DOG_FINAL_POSITION_WAIT_IDLE_ENCODER) &&
             (s_encoder_rx_count[index] != s_position_idle_encoder_rx_baseline[index])) ? 1U : 0U;
        if ((idle_ready != 0U) || (encoder_ready != 0U)) {
            (void)request_closed_loop(index);
        }
    }
}

static uint8_t foot_xz_workspace_ok(float x_mm, float z_mm, float *d_out)
{
    float l1 = DOG_THIGH_MM;
    float l2 = DOG_SHANK_MM;

    if ((!isfinite(x_mm)) || (!isfinite(z_mm)) || (z_mm <= 0.0f)) {
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

static float march_slew_command(float current, float requested)
{
    const float delta = requested - current;
    return current + clampf(delta,
                            -DOG_DRIVE_COMMAND_SLEW_PER_HALF_STEP,
                            DOG_DRIVE_COMMAND_SLEW_PER_HALF_STEP);
}

static void march_compute_drive_stride_deltas(float forward, float yaw,
                                               float forward_stride_x_mm,
                                               float turn_stride_x_mm,
                                               float *deltas_mm)
{
    if (deltas_mm == nullptr) {
        return;
    }

    const float envelope_mm = fmaxf(fabsf(forward_stride_x_mm), fabsf(turn_stride_x_mm));
    float max_abs_delta_mm = 0.0f;
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        const float forward_delta_mm = march_trot_apply_yaw_trim(
            forward * forward_stride_x_mm * leg_ik_x_sign(leg) * DOG_TROT_FORWARD_X_SIGN,
            leg);
        const float yaw_delta_mm = yaw * turn_stride_x_mm * DOG_TURN_X_SIGN;
        deltas_mm[leg] = forward_delta_mm + yaw_delta_mm;
        max_abs_delta_mm = fmaxf(max_abs_delta_mm, fabsf(deltas_mm[leg]));
    }

    if ((envelope_mm > 0.0f) && (max_abs_delta_mm > envelope_mm)) {
        const float scale = envelope_mm / max_abs_delta_mm;
        for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
            deltas_mm[leg] *= scale;
        }
    }
}

static float march_compatible_wheel_rpm(float forward, float yaw,
                                        float forward_stride_x_mm,
                                        float turn_stride_x_mm,
                                        uint32_t swing_ms)
{
    if (swing_ms == 0U) {
        return 0.0f;
    }
    const float left_stride_mm = forward * forward_stride_x_mm + yaw * turn_stride_x_mm;
    const float right_stride_mm = forward * forward_stride_x_mm - yaw * turn_stride_x_mm;
    const float travel_mm = fmaxf(fabsf(left_stride_mm), fabsf(right_stride_mm));
    const float travel_mm_s = travel_mm * 1000.0f / (float)swing_ms;
    return travel_mm_s * 60.0f / (DOG_PI * DOG_WHEEL_DIAMETER_MM);
}

static void march_snapshot_drive_command(void)
{
    s_march.applied_forward = march_slew_command(s_march.applied_forward,
                                                  s_march.requested_forward);
    s_march.applied_yaw = march_slew_command(s_march.applied_yaw,
                                              s_march.requested_yaw);
    s_march.active_forward = s_march.applied_forward;
    s_march.active_yaw = s_march.applied_yaw;
    s_march.active_speed_profile = dog_mit_get_gait_speed_profile();
    s_march.active_swing_ms = dog_mit_gait_trot_swing_ms();
    s_march.active_touchdown_dwell_ms = dog_mit_gait_touchdown_dwell_ms();
    s_march.active_diagonal_stagger_ms = dog_mit_gait_diagonal_stagger_ms();
    s_march.active_forward_stride_x_mm = dog_mit_gait_forward_stride_x_mm();
    s_march.active_turn_stride_x_mm = dog_mit_gait_turn_stride_x_mm();
    s_march.active_swing_height_mm = dog_mit_gait_swing_height_mm();
    taskENTER_CRITICAL();
    s_march.active_wheel_contribution = clampf(
        s_requested_gait_wheel_contribution, 0.0f,
        DOG_GAIT_WHEEL_MAX_CONTRIBUTION);
    taskEXIT_CRITICAL();
    s_march.active_leg_contribution = 1.0f - s_march.active_wheel_contribution;
    s_march.compatible_wheel_rpm = march_compatible_wheel_rpm(
        s_march.active_forward, s_march.active_yaw,
        s_march.active_forward_stride_x_mm,
        s_march.active_turn_stride_x_mm,
        s_march.active_swing_ms);
    s_march.half_step_generation++;
    march_compute_drive_stride_deltas(s_march.active_forward *
                                          s_march.active_leg_contribution,
                                      s_march.active_yaw *
                                          s_march.active_leg_contribution,
                                      s_march.active_forward_stride_x_mm,
                                      s_march.active_turn_stride_x_mm,
                                      s_march.active_stride_delta_mm);
}

static uint8_t march_mode_uses_cycloid(uint8_t mode)
{
    return ((mode == DOG_MARCH_MODE_TROT) || (mode == DOG_MARCH_MODE_TURN_LEFT) ||
            (mode == DOG_MARCH_MODE_TURN_RIGHT)) ? 1U : 0U;
}

static float march_stride_x_delta(uint8_t leg)
{
    return (leg < DOG_LEG_COUNT) ? s_march.active_stride_delta_mm[leg] : 0.0f;
}

static float march_trot_swing_peak_z_mm(uint8_t leg)
{
    const float swing_height_mm = (s_march.active_swing_ms != 0U) ?
        s_march.active_swing_height_mm : dog_mit_gait_swing_height_mm();
    return dog_leg_stand_foot_z_mm(leg) + DOG_TROT_FOOT_Z1_MM +
           s_leg_touchdown_z_offset[leg] - swing_height_mm;
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
    if ((!isfinite(hip_motor_deg)) || (!isfinite(knee_motor_deg))) {
        return 0U;
    }

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

static uint8_t all_leg_motors_heartbeat_online(uint32_t now)
{
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((s_motor_online[i] == 0U) || (s_last_heartbeat_tick_ms[i] == 0U) ||
            ((uint32_t)(now - s_last_heartbeat_tick_ms[i]) > DOG_HEARTBEAT_TIMEOUT_MS) ||
            (motor_has_fault(i) != 0U)) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t all_leg_motors_control_ready(void)
{
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((motor_ready(i) == 0U) || (s_encoder_est_fresh[i] == 0U) ||
            (s_motor_mit_probe_active[i] != 0U) || (s_motor_configured[i] == 0U)) {
            return 0U;
        }
    }
    return 1U;
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
    uint32_t safety_generation = 0U;
    if (motor_safety_token_acquire(&safety_generation) == 0U) {
        return;
    }
    if ((uint32_t)(now - s_last_command_tick_ms) < DOG_CMD_PERIOD_MS) return;
    s_last_command_tick_ms = now;

    if (s_control_loop_mode == DOG_CTRL_LOOP_MIT_PID) {
        send_mit_torque_commands(now);
        return;
    }

    const uint8_t count = (s_auto_stand_enabled != 0U) ? DOG_MOTOR_COUNT : selected_count();
    for (uint8_t i = 0U; i < count; ++i) {
        const uint8_t index = (s_auto_stand_enabled != 0U) ? i : selected_index(i);
        if ((index >= DOG_MOTOR_COUNT) || (motor_ready(index) == 0U) ||
            (motor_encoder_fresh(index, now) == 0U) ||
            (s_motor_configured[index] == 0U) ||
            (s_motor_mit_probe_active[index] != 0U) ||
            (s_motor_final_mode_pending[index] != DOG_FINAL_MODE_NONE) ||
            (!isfinite(s_target_turn[index]))) {
            mit_debug_abort_control("position target feedback/state");
            return;
        }
    }

    for (uint8_t i = 0U; i < count; ++i) {
        if (motor_safety_token_valid(safety_generation) == 0U) {
            queue_motor_estop((s_auto_stand_enabled != 0U) ? i : selected_index(i));
            return;
        }
        const uint8_t index = (s_auto_stand_enabled != 0U) ? i : selected_index(i);
        Dog_Motor_Config *cfg = &g_dog_motor_config[index];
        MWPosControl(cfg->bus, cfg->node_id, s_target_turn[index], 0, 0);
        if (motor_safety_token_valid(safety_generation) == 0U) {
            queue_motor_estop(index);
            return;
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
    if ((s_auto_stand_enabled == 0U) || (s_stand_state == DOG_STAND_ESTOP) ||
        (s_safety_latched != 0U)) {
        return;
    }
    if (online_fault_present() != 0U) {
        mit_debug_abort_control("automatic stand motor fault");
        return;
    }

    switch (s_stand_state) {
    case DOG_STAND_WAIT_HEARTBEAT:
        if (all_leg_motors_heartbeat_online(now) != 0U) {
            memset(s_motor_configured, 0, sizeof(s_motor_configured));
            s_stand_motor_cursor = 0U;
            s_state_start_ms = now;
            s_stand_state = DOG_STAND_CONFIGURE;
        } else if ((uint32_t)(now - s_state_start_ms) >= DOG_STAND_WAIT_MS) {
            mit_debug_abort_control("automatic stand heartbeat timeout");
        }
        break;
    case DOG_STAND_CONFIGURE:
        if ((s_stand_motor_cursor < DOG_MOTOR_COUNT) &&
            ((uint32_t)(now - s_state_start_ms) >= DOG_STAND_CONFIG_SLOT_MS)) {
            configure_motor(s_stand_motor_cursor);
            s_stand_motor_cursor++;
            s_state_start_ms = now;
        } else if (s_stand_motor_cursor >= DOG_MOTOR_COUNT) {
            s_stand_motor_cursor = 0U;
            s_state_start_ms = now;
            s_stand_state = DOG_STAND_CLOSED_LOOP;
        }
        break;
    case DOG_STAND_CLOSED_LOOP:
        if ((s_stand_motor_cursor < DOG_MOTOR_COUNT) &&
            ((uint32_t)(now - s_state_start_ms) >= DOG_STAND_LOOP_SLOT_MS)) {
            (void)request_closed_loop(s_stand_motor_cursor);
            s_stand_motor_cursor++;
            s_state_start_ms = now;
        } else if ((s_stand_motor_cursor >= DOG_MOTOR_COUNT) &&
                   ((uint32_t)(now - s_state_start_ms) >= DOG_STAND_READY_TIMEOUT_MS)) {
            if (all_leg_motors_control_ready() == 0U) {
                mit_debug_abort_control("automatic stand closed-loop timeout");
                break;
            }
            uint32_t safety_generation = 0U;
            if (motor_safety_token_acquire(&safety_generation) == 0U) {
                break;
            }
            prepare_stand_targets();
            if (motor_safety_token_guard_take(safety_generation) == 0U) {
                break;
            }
            s_position_tx_enabled = 1U;
            s_last_command_tick_ms = 0U;
            s_state_start_ms = now;
            s_stand_state = DOG_STAND_MOVING;
            motor_tx_guard_give();
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
    if (header.Identifier > 0x7FFU) {
        return 0xFFFFFFFFU;
    }
    return header.Identifier;
}

static uint8_t mw_cmd_is_valid_response(uint8_t cmd)
{
    switch ((MW_CMD_ID)cmd) {
    case MW_HEARTBEAT_CMD:
    case MW_GET_ERROR_CMD:
    case MW_RXSDO_CMD:
    case MW_MIT_CONTROL_CMD:
    case MW_GET_ENCODER_ESTIMATES_CMD:
    case MW_GET_ENCODER_COUNT_CMD:
    case MW_GET_IQ_CMD:
    case MW_GET_BUS_VOLTAGE_CURRENT_CMD:
    case MW_GET_TORQUES_CMD:
    case MW_GET_POWERS_CMD:
        return 1U;
    default:
        return 0U;
    }
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
    const uint8_t cmd = (uint8_t)(can_id & 0x1FU);
    const uint8_t index = motor_index(bus, node_id);
    if ((index >= DOG_MOTOR_COUNT) || (mw_cmd_is_valid_response(cmd) == 0U)) {
        s_rx_reject_node[di]++;
        return;
    }

    if ((node_id >= MAX_MOTOR_NUM_PER_BUS) ||
        (motors[bus][node_id].motorData == nullptr)) {
        s_rx_reject_nodata[di]++;
        return;
    }

    if ((cmd == (uint8_t)MW_HEARTBEAT_CMD) || (cmd == (uint8_t)MW_GET_ENCODER_ESTIMATES_CMD)) {
        s_motor_can_id_type[index] = (header.IdType == FDCAN_STANDARD_ID) ?
                                     DOG_CAN_ID_STANDARD : DOG_CAN_ID_EXTENDED;
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

static void CAN3_Callback(FDCAN_RxHeaderTypeDef &header, uint8_t *buffer)
{
    DebugUart_LogCanRx(3U, header.Identifier, buffer, fdcan_dlc_to_bytes(header.DataLength));
    (void)WheelDrive_OnCanRx(&header, buffer);
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
    diag->tx_drop_count = s_can_tx_drop_count[di];
}

static void restart_bus_off(FDCAN_HandleTypeDef *h)
{
    const uint8_t recovery = fdcan_recover_bus_off(h);
    if (recovery != FDCAN_RECOVERY_RESTARTED) {
        return;
    }
    DogStand_ExitMechanicalLimitIdle();
    if (s_safety_latched == 0U) {
        return;
    }

    const uint8_t bus = (h == &hfdcan2) ? DOG_CAN_REAR_BUS : DOG_CAN_FRONT_BUS;
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if (g_dog_motor_config[i].bus == bus) {
            queue_motor_estop(i);
        }
    }
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

static uint8_t mit_pump_control(void)
{
    uint32_t now = 0U;
    if (motor_blocking_service(&now) == 0U) {
        return 0U;
    }

    encoder_feedback_query_tick(now);
    if ((s_safety_latched != 0U) || (s_position_tx_enabled == 0U) ||
        (s_control_loop_mode != DOG_CTRL_LOOP_MIT_PID) ||
        (s_mit_debug_active == 0U) || (s_mit_fault_hold_active != 0U)) {
        return 0U;
    }

    send_enabled_targets(now);
    return ((s_safety_latched == 0U) && (s_position_tx_enabled != 0U) &&
            (s_mit_debug_active != 0U) && (s_mit_fault_hold_active == 0U)) ? 1U : 0U;
}

static uint8_t mit_wait_joint_settle(uint8_t joint, float err_threshold_deg, uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - t0) < timeout_ms) {
        if (dog_mit_fault_hold_is_active() != 0U) {
            return 0U;
        }

        if (mit_pump_control() == 0U) {
            return 0U;
        }

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

    DebugUart_Printf("MIT settle timeout joint=%s err>%ldmdeg\r\n",
                     joint_name(joint), (long)(err_threshold_deg * 1000.0f));
    return 0U;
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

        if (mit_pump_control() == 0U) {
            return 0U;
        }

        if (s_encoder_est_fresh[idx] == 0U) {
            HAL_Delay(5U);
            continue;
        }
        if (fabsf(s_target_deg[idx] - user_deg(idx)) <= err_threshold_deg) {
            return 1U;
        }
        HAL_Delay(5U);
    }

    DebugUart_Printf("MIT settle timeout leg=%s joint=%s err>%ldmdeg\r\n",
                     dog_leg_name(leg), joint_name(joint), (long)(err_threshold_deg * 1000.0f));
    return 0U;
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

static float smoothstep5_01(float x)
{
    x = clampf(x, 0.0f, 1.0f);
    return x * x * x * ((x * ((6.0f * x) - 15.0f)) + 10.0f);
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

        if (mit_pump_control() == 0U) {
            return 0U;
        }

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

    DebugUart_Printf("Stand settle timeout err>%ldmdeg\r\n",
                     (long)(DOG_STAND_JOINT_SETTLE_ERR_DEG * 1000.0f));
    return 0U;
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
        if (mit_pump_control() == 0U) {
            return 0U;
        }

        if (progress >= 1.0f) {
            break;
        }
        HAL_Delay(1U);
    }

    return mit_stand_wait_target_legs_settled(DOG_STAND_MOVE_MS);
}

uint8_t dog_mit_lower_to_start_pose_start(void)
{
    if (s_lower_state == DOG_LOWER_ACTIVE) {
        return 1U;
    }
    if ((dog_mit_debug_is_active() == 0U) ||
        (dog_mit_fault_hold_is_active() != 0U) ||
        (s_debug_target != DOG_DEBUG_TARGET_ALL)) {
        s_lower_state = DOG_LOWER_FAILED;
        DebugUart_Printf("Lower stand FAIL: need active all-leg MIT stand.\r\n");
        return 0U;
    }
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        if ((dog_leg_foot_xz_is_reachable(DOG_STAND_FOOT_X_MM,
                                           dog_leg_stand_foot_z_start_mm(leg)) == 0U) ||
            (dog_leg_foot_xz_to_motor_deg(leg, DOG_STAND_FOOT_X_MM,
                                          dog_leg_stand_foot_z_start_mm(leg),
                                          nullptr, nullptr) == 0U)) {
            s_lower_state = DOG_LOWER_FAILED;
            DebugUart_Printf("Lower stand FAIL: %s target (%ld,%ld)mm unreachable.\r\n",
                             dog_leg_name(leg),
                             (long)DOG_STAND_FOOT_X_MM,
                             (long)dog_leg_stand_foot_z_start_mm(leg));
            return 0U;
        }
    }

    dog_mit_march_in_place_stop();
    mit_set_all_stand_pid_mode();
    dog_mit_reset_integrators();
    s_lower_t0_ms = HAL_GetTick();
    s_lower_settle_since_ms = 0U;
    s_lower_state = DOG_LOWER_ACTIVE;
    DebugUart_Printf("Lower stand start: front %ld->%ld rear %ld->%ldmm smooth=%lums.\r\n",
                     (long)DOG_STAND_FOOT_Z_MM,
                     (long)DOG_STAND_FOOT_Z_START_MM,
                     (long)dog_leg_stand_foot_z_mm(DOG_LEG_LB),
                     (long)dog_leg_stand_foot_z_start_mm(DOG_LEG_LB),
                     (unsigned long)DOG_STAND_RISE_MS);
    return 1U;
}

uint8_t dog_mit_lower_to_start_pose_state(void)
{
    return s_lower_state;
}

void dog_mit_lower_to_start_pose_cancel(void)
{
    s_lower_state = DOG_LOWER_IDLE;
    s_lower_t0_ms = 0U;
    s_lower_settle_since_ms = 0U;
}

static void dog_mit_lower_to_start_pose_tick(uint32_t now)
{
    if (s_lower_state != DOG_LOWER_ACTIVE) {
        return;
    }
    if ((dog_mit_debug_is_active() == 0U) ||
        (dog_mit_fault_hold_is_active() != 0U) ||
        (s_debug_target != DOG_DEBUG_TARGET_ALL)) {
        s_lower_state = DOG_LOWER_FAILED;
        DebugUart_Printf("Lower stand FAIL: leg closed loop lost.\r\n");
        return;
    }

    const uint32_t elapsed_ms = (uint32_t)(now - s_lower_t0_ms);
    const float raw_progress = (DOG_STAND_RISE_MS == 0U) ? 1.0f :
        clampf((float)elapsed_ms / (float)DOG_STAND_RISE_MS, 0.0f, 1.0f);
    const float stand_progress = 1.0f - smoothstep5_01(raw_progress);
    if (mit_stand_set_rise_pose(DOG_STAND_FOOT_X_MM, stand_progress) == 0U) {
        s_lower_state = DOG_LOWER_FAILED;
        DebugUart_Printf("Lower stand FAIL: IK update rejected.\r\n");
        return;
    }

    if (raw_progress < 1.0f) {
        s_lower_settle_since_ms = 0U;
        return;
    }

    uint8_t all_settled = 1U;
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        const uint8_t hip = leg_joint_index(leg, DOG_JOINT_HIP);
        const uint8_t knee = leg_joint_index(leg, DOG_JOINT_KNEE);
        if ((march_leg_joints_settled(leg, DOG_STAND_JOINT_SETTLE_ERR_DEG) == 0U) ||
            (hip >= DOG_MOTOR_COUNT) || (knee >= DOG_MOTOR_COUNT) ||
            (fabsf(user_vel_dps(hip)) > DOG_STAND_LOWER_SETTLE_VEL_DPS) ||
            (fabsf(user_vel_dps(knee)) > DOG_STAND_LOWER_SETTLE_VEL_DPS)) {
            all_settled = 0U;
        }
    }
    if (all_settled != 0U) {
        if (s_lower_settle_since_ms == 0U) {
            s_lower_settle_since_ms = now;
        } else if ((uint32_t)(now - s_lower_settle_since_ms) >=
                   DOG_STAND_LOWER_SETTLE_MS) {
            s_lower_state = DOG_LOWER_COMPLETE;
            DebugUart_Printf("Lower stand complete; LOW mode may release leg torque.\r\n");
        }
    } else {
        s_lower_settle_since_ms = 0U;
    }

    if ((s_lower_state == DOG_LOWER_ACTIVE) &&
        (elapsed_ms >= (DOG_STAND_RISE_MS + DOG_STAND_MOVE_MS))) {
        s_lower_state = DOG_LOWER_FAILED;
        DebugUart_Printf("Lower stand FAIL: settle timeout; keeping leg control.\r\n");
    }
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
        if (mit_pump_control() == 0U) {
            s_jump_active = 0U;
            return 0U;
        }
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
        if (mit_pump_control() == 0U) {
            s_jump_active = 0U;
            return 0U;
        }

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

static void march_stability_velocity_filter_reset(uint32_t now)
{
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        const float velocity_dps = user_vel_dps(i);
        s_march.stable_velocity_lpf_dps[i] =
            isfinite(velocity_dps) ? fabsf(velocity_dps) : INFINITY;
    }
    s_march.stable_velocity_filter_ms = now;
}

static void march_stability_velocity_filter_update(uint32_t now)
{
    uint32_t dt_ms = (uint32_t)(now - s_march.stable_velocity_filter_ms);
    if (dt_ms == 0U) {
        return;
    }
    if (dt_ms > 20U) {
        dt_ms = 20U;
    }
    const float alpha = (float)dt_ms /
        (DOG_GAIT_STABILITY_VEL_TAU_MS + (float)dt_ms);
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        const float velocity_dps = user_vel_dps(i);
        if (!isfinite(velocity_dps)) {
            s_march.stable_velocity_lpf_dps[i] = INFINITY;
            continue;
        }
        const float abs_velocity_dps = fabsf(velocity_dps);
        s_march.stable_velocity_lpf_dps[i] += alpha *
            (abs_velocity_dps - s_march.stable_velocity_lpf_dps[i]);
    }
    s_march.stable_velocity_filter_ms = now;
}

static uint8_t march_leg_joints_stable(uint8_t leg, float err_threshold_deg,
                                       float velocity_threshold_dps)
{
    if (march_leg_joints_settled(leg, err_threshold_deg) == 0U) {
        return 0U;
    }

    const uint8_t hip = leg_joint_index(leg, DOG_JOINT_HIP);
    const uint8_t knee = leg_joint_index(leg, DOG_JOINT_KNEE);
    if ((hip >= DOG_MOTOR_COUNT) || (knee >= DOG_MOTOR_COUNT)) {
        return 0U;
    }
    const float hip_velocity_dps = s_march.stable_velocity_lpf_dps[hip];
    const float knee_velocity_dps = s_march.stable_velocity_lpf_dps[knee];
    if ((!isfinite(hip_velocity_dps)) || (!isfinite(knee_velocity_dps))) {
        return 0U;
    }
    return ((fabsf(hip_velocity_dps) <= velocity_threshold_dps) &&
            (fabsf(knee_velocity_dps) <= velocity_threshold_dps)) ? 1U : 0U;
}

static void march_get_swing_legs(uint8_t *leg_a, uint8_t *leg_b);

static uint8_t march_all_legs_stable(float err_threshold_deg, float velocity_threshold_dps)
{
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        if (march_leg_joints_stable(leg, err_threshold_deg, velocity_threshold_dps) == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t march_current_pair_has_support_margin(float current_limit_a)
{
    uint8_t leg_a = 0U;
    uint8_t leg_b = DOG_LEG_COUNT;
    march_get_swing_legs(&leg_a, &leg_b);
    const uint8_t legs[2U] = {leg_a, leg_b};
    for (uint8_t i = 0U; i < 2U; ++i) {
        if (legs[i] >= DOG_LEG_COUNT) {
            continue;
        }
        const uint8_t hip = leg_joint_index(legs[i], DOG_JOINT_HIP);
        const uint8_t knee = leg_joint_index(legs[i], DOG_JOINT_KNEE);
        if ((hip >= DOG_MOTOR_COUNT) || (knee >= DOG_MOTOR_COUNT) ||
            (!isfinite(s_mit_cmd_current_a[hip])) ||
            (!isfinite(s_mit_cmd_current_a[knee])) ||
            (fabsf(s_mit_cmd_current_a[hip]) >= current_limit_a) ||
            (fabsf(s_mit_cmd_current_a[knee]) >= current_limit_a)) {
            return 0U;
        }
    }
    return 1U;
}

static float march_leg_abs_iq_a(uint8_t leg)
{
    const uint8_t hip = leg_joint_index(leg, DOG_JOINT_HIP);
    const uint8_t knee = leg_joint_index(leg, DOG_JOINT_KNEE);
    if ((hip >= DOG_MOTOR_COUNT) || (knee >= DOG_MOTOR_COUNT)) {
        return 0.0f;
    }
    return fabsf(g_mw_motor_data[hip].iq.iqMeasured) +
           fabsf(g_mw_motor_data[knee].iq.iqMeasured);
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
    s_leg_command_x_mm[leg] = x_mm;
    s_leg_command_z_mm[leg] = z_mm;
}

static void march_set_all_legs_stand_pose(void)
{
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        march_set_leg_foot_xz(leg, DOG_STAND_FOOT_X_MM, dog_leg_stand_foot_z_mm(leg));
    }
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

static uint8_t march_gait_point_valid(uint8_t leg, float x_mm, float z_mm, const char *label)
{
    float hip_deg = 0.0f;
    float knee_deg = 0.0f;
    if (dog_leg_foot_xz_to_motor_deg(leg, x_mm, z_mm, &hip_deg, &knee_deg) != 0U) {
        return 1U;
    }

    DebugUart_Printf("Gait FAIL: %s %s IK/limit (%.1f, %.1f) mm.\r\n",
                     dog_leg_name(leg), label, (double)x_mm, (double)z_mm);
    return 0U;
}

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

        if (march_gait_point_valid(leg, x1, z2, "swing peak") == 0U) {
            return 0U;
        }
        if (march_gait_point_valid(leg, x2, z2, "swing peak") == 0U) {
            return 0U;
        }
        if (march_gait_point_valid(leg, x1, z1, "touch-down") == 0U) {
            return 0U;
        }
        if (march_gait_point_valid(leg, x2, z1, "touch-down") == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t march_gait_stride_reachable(uint8_t mode, float forward, float yaw)
{
    const uint8_t prev_mode = s_march.mode;
    const uint32_t prev_active_swing_ms = s_march.active_swing_ms;
    const float prev_forward_stride_x_mm = s_march.active_forward_stride_x_mm;
    const float prev_turn_stride_x_mm = s_march.active_turn_stride_x_mm;
    const float prev_swing_height_mm = s_march.active_swing_height_mm;
    float prev_deltas_mm[DOG_LEG_COUNT] = {};
    memcpy(prev_deltas_mm, s_march.active_stride_delta_mm, sizeof(prev_deltas_mm));
    s_march.mode = mode;
    s_march.active_swing_ms = dog_mit_gait_trot_swing_ms();
    s_march.active_forward_stride_x_mm = dog_mit_gait_forward_stride_x_mm();
    s_march.active_turn_stride_x_mm = dog_mit_gait_turn_stride_x_mm();
    s_march.active_swing_height_mm = dog_mit_gait_swing_height_mm();
    march_compute_drive_stride_deltas(forward, yaw,
                                      s_march.active_forward_stride_x_mm,
                                      s_march.active_turn_stride_x_mm,
                                      s_march.active_stride_delta_mm);
    const uint8_t ok = march_gait_corners_reachable();
    s_march.mode = prev_mode;
    s_march.active_swing_ms = prev_active_swing_ms;
    s_march.active_forward_stride_x_mm = prev_forward_stride_x_mm;
    s_march.active_turn_stride_x_mm = prev_turn_stride_x_mm;
    s_march.active_swing_height_mm = prev_swing_height_mm;
    memcpy(s_march.active_stride_delta_mm, prev_deltas_mm, sizeof(prev_deltas_mm));
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
        *z1 = dog_leg_stand_foot_z_mm(leg) + DOG_TROT_FOOT_Z1_MM +
              s_leg_touchdown_z_offset[leg];
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
    const uint32_t swing_ms = (s_march.active_swing_ms != 0U) ?
        s_march.active_swing_ms : dog_mit_gait_trot_swing_ms();
    const float period_s = (float)swing_ms * 0.001f;
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

static uint32_t march_trot_leg_delay_ms(uint8_t leg)
{
    if ((s_march.active_diagonal_stagger_ms == 0U) ||
        (s_march.active_diagonal_stagger_ms >= s_march.active_swing_ms) ||
        (dog_leg_is_rear(leg) != 0U)) {
        return 0U;
    }
    return s_march.active_diagonal_stagger_ms;
}

static float march_trot_leg_elapsed_s(uint32_t now, uint8_t leg,
                                      float *period_s_out, float *progress_out)
{
    const uint32_t pair_ms = (s_march.active_swing_ms != 0U) ?
        s_march.active_swing_ms : dog_mit_gait_trot_swing_ms();
    const uint32_t stagger_ms = (s_march.active_diagonal_stagger_ms < pair_ms) ?
        s_march.active_diagonal_stagger_ms : 0U;
    const uint32_t leg_ms = (pair_ms > stagger_ms) ? (pair_ms - stagger_ms) : pair_ms;
    const uint32_t delay_ms = march_trot_leg_delay_ms(leg);
    const uint32_t pair_elapsed_ms = (uint32_t)(now - s_march.swing_t0_ms);
    uint32_t leg_elapsed_ms = 0U;
    if (pair_elapsed_ms > delay_ms) {
        leg_elapsed_ms = pair_elapsed_ms - delay_ms;
        if (leg_elapsed_ms > leg_ms) {
            leg_elapsed_ms = leg_ms;
        }
    }

    const float period_s = (float)leg_ms * 0.001f;
    if (period_s_out != nullptr) {
        *period_s_out = period_s;
    }
    if (progress_out != nullptr) {
        *progress_out = (leg_ms == 0U) ? 1.0f :
            clampf((float)leg_elapsed_ms / (float)leg_ms, 0.0f, 1.0f);
    }
    return (float)leg_elapsed_ms * 0.001f;
}

static void march_trot_update_swing_pid_mask(uint32_t now)
{
    uint8_t leg_a = 0U;
    uint8_t leg_b = DOG_LEG_COUNT;
    march_get_swing_legs(&leg_a, &leg_b);
    const uint8_t legs[2U] = {leg_a, leg_b};
    const uint32_t pair_ms = s_march.active_swing_ms;
    const uint32_t stagger_ms = (s_march.active_diagonal_stagger_ms < pair_ms) ?
        s_march.active_diagonal_stagger_ms : 0U;
    const uint32_t leg_ms = (pair_ms > stagger_ms) ? (pair_ms - stagger_ms) : pair_ms;
    const uint32_t elapsed_ms = (uint32_t)(now - s_march.swing_t0_ms);
    uint8_t leg_mask = 0U;

    for (uint8_t i = 0U; i < 2U; ++i) {
        const uint8_t leg = legs[i];
        if (leg >= DOG_LEG_COUNT) {
            continue;
        }
        const uint32_t delay_ms = march_trot_leg_delay_ms(leg);
        if ((elapsed_ms >= delay_ms) && (elapsed_ms < (delay_ms + leg_ms))) {
            leg_mask |= (uint8_t)(1U << leg);
        }
    }
    mit_set_mixed_swing_leg_mask(leg_mask);
}

static void march_trot_trajectory_smooth(float t_s, const March_Trot_Traj_Params *params,
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

    const float tau = clampf(t_s / params->period_s, 0.0f, 1.0f);
    *x_mm = params->start_x + params->step_length * smoothstep5_01(tau);
    if (tau <= DOG_GAIT_SWING_APEX_PROGRESS) {
        const float rise = tau / DOG_GAIT_SWING_APEX_PROGRESS;
        *z_mm = params->start_z + params->step_height * smoothstep5_01(rise);
    } else {
        const float descend = (tau - DOG_GAIT_SWING_APEX_PROGRESS) /
                              (1.0f - DOG_GAIT_SWING_APEX_PROGRESS);
        *z_mm = params->start_z + params->step_height *
                (1.0f - smoothstep5_01(descend));
    }
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
    march_trot_update_swing_pid_mask(now);

    for (uint8_t i = 0U; i < 2U; ++i) {
        const uint8_t leg = swing_legs[i];
        float period_s = 0.0f;
        float progress = 0.0f;
        const float t_s = march_trot_leg_elapsed_s(now, leg, &period_s, &progress);
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
        march_trot_trajectory_smooth(t_s, &params, &foot_x, &foot_z);
        s_march.swing_progress[leg] = progress;
        if ((s_march.contact_mask & (uint8_t)(1U << leg)) == 0U) {
            s_march.frozen_x_mm[leg] = foot_x;
            s_march.frozen_z_mm[leg] = foot_z;
            march_trot_set_swing_foot(leg, foot_x, foot_z);
        } else {
            march_trot_set_swing_foot(leg, s_march.frozen_x_mm[leg],
                                      s_march.frozen_z_mm[leg]);
        }
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
        March_Trot_Traj_Params params = {
            -retract,
            0.0f,
            period_s,
            x1,
            z1,
        };
        float foot_x = x1;
        float foot_z = z1;
        march_trot_trajectory_smooth(t_s, &params, &foot_x, &foot_z);
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
            if ((s_march.contact_mask & (uint8_t)(1U << leg)) == 0U) {
                s_leg_foot_x_offset[leg] += delta;
            }
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
    const float lift_z = dog_leg_stand_foot_z_mm(leg) - DOG_MARCH_LIFT_Z_MM;
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

static void march_contact_samples_begin(uint8_t leg_a, uint8_t leg_b)
{
    const uint8_t legs[2U] = {leg_a, leg_b};
    s_march.swing_mask = 0U;
    s_march.contact_mask = 0x0FU;
    s_march.contact_search_mask = 0U;
    s_march.contact_failure_mask = 0U;
    memset(s_march.contact_candidate_since_ms, 0,
           sizeof(s_march.contact_candidate_since_ms));
    memset(s_march.contact_stable_since_ms, 0,
           sizeof(s_march.contact_stable_since_ms));
    memset(s_march.search_limit_since_ms, 0,
           sizeof(s_march.search_limit_since_ms));
    memset(s_march.contact_search_mm, 0, sizeof(s_march.contact_search_mm));
    memset(s_march.swing_progress, 0, sizeof(s_march.swing_progress));
    for (uint8_t i = 0U; i < 2U; ++i) {
        const float iq_a = (legs[i] < DOG_LEG_COUNT) ? march_leg_abs_iq_a(legs[i]) : 0.0f;
        if (legs[i] >= DOG_LEG_COUNT) {
            continue;
        }
        const uint8_t bit = (uint8_t)(1U << legs[i]);
        s_march.swing_mask |= bit;
        s_march.contact_mask &= (uint8_t)~bit;
        s_march.contact_iq_baseline_a[legs[i]] = iq_a;
        s_march.contact_iq_filtered_a[legs[i]] = iq_a;
    }
    s_march.contact_iq_query_ms = 0U;
}

static void march_contact_samples_update(uint32_t now)
{
    if (march_mode_uses_cycloid(s_march.mode) == 0U) {
        return;
    }

    uint8_t leg_a = 0U;
    uint8_t leg_b = DOG_LEG_COUNT;
    march_get_swing_legs(&leg_a, &leg_b);
    const uint8_t legs[2U] = {leg_a, leg_b};

    if ((s_march.contact_iq_query_ms == 0U) ||
        ((uint32_t)(now - s_march.contact_iq_query_ms) >= 10U)) {
        s_march.contact_iq_query_ms = now;
        for (uint8_t i = 0U; i < 2U; ++i) {
            if (legs[i] >= DOG_LEG_COUNT) {
                continue;
            }
            const uint8_t hip = leg_joint_index(legs[i], DOG_JOINT_HIP);
            const uint8_t knee = leg_joint_index(legs[i], DOG_JOINT_KNEE);
            if ((hip >= DOG_MOTOR_COUNT) || (knee >= DOG_MOTOR_COUNT)) {
                continue;
            }
            FDCAN_HandleTypeDef *h = bus_handle(g_dog_motor_config[hip].bus);
            if ((h != nullptr) && (fdcan_tx_free_level(h) >= 2U)) {
                MWGetIq(g_dog_motor_config[hip].bus, g_dog_motor_config[hip].node_id);
                MWGetIq(g_dog_motor_config[knee].bus, g_dog_motor_config[knee].node_id);
            }
        }
    }

    for (uint8_t i = 0U; i < 2U; ++i) {
        if (legs[i] >= DOG_LEG_COUNT) {
            continue;
        }
        float progress = 1.0f;
        (void)march_trot_leg_elapsed_s(now, legs[i], nullptr, &progress);
        const uint8_t leg = legs[i];
        const uint8_t bit = (uint8_t)(1U << leg);
        const float iq_a = march_leg_abs_iq_a(leg);
        if (progress < DOG_TROT_TOUCHDOWN_IQ_START) {
            if (iq_a < s_march.contact_iq_baseline_a[leg]) {
                s_march.contact_iq_baseline_a[leg] = iq_a;
            }
            s_march.contact_iq_filtered_a[leg] = iq_a;
            s_march.contact_candidate_since_ms[leg] = 0U;
        } else {
            s_march.contact_iq_filtered_a[leg] += DOG_TROT_TOUCHDOWN_IQ_ALPHA *
                (iq_a - s_march.contact_iq_filtered_a[leg]);
            if ((s_march.contact_mask & bit) != 0U) {
                continue;
            }
            const float iq_rise = s_march.contact_iq_filtered_a[leg] -
                                  s_march.contact_iq_baseline_a[leg];
            if (iq_rise >= DOG_TROT_TOUCHDOWN_IQ_RISE_A) {
                if (s_march.contact_candidate_since_ms[leg] == 0U) {
                    s_march.contact_candidate_since_ms[leg] = now;
                } else if ((uint32_t)(now - s_march.contact_candidate_since_ms[leg]) >=
                           DOG_TROT_TOUCHDOWN_CONFIRM_MS) {
                    s_march.contact_mask |= bit;
                    s_march.contact_search_mask &= (uint8_t)~bit;
                    s_leg_foot_x_offset[leg] = s_march.frozen_x_mm[leg];
                    s_leg_touchdown_z_offset[leg] = clampf(
                        s_march.frozen_z_mm[leg] - dog_leg_stand_foot_z_mm(leg) -
                            DOG_TROT_FOOT_Z1_MM,
                        -DOG_TROT_TOUCHDOWN_SEARCH_MM,
                        DOG_TROT_TOUCHDOWN_SEARCH_MM);
                }
            } else if (iq_rise <= DOG_TROT_TOUCHDOWN_IQ_EXIT_A) {
                s_march.contact_candidate_since_ms[leg] = 0U;
            }
        }
    }
}

static uint8_t march_swing_legs_contact_detected(void)
{
    return ((s_march.contact_mask & s_march.swing_mask) == s_march.swing_mask) ? 1U : 0U;
}

static void march_touchdown_search_update(uint32_t now)
{
    const uint32_t elapsed_ms = (uint32_t)(now - s_march.phase_t0_ms);
    const float requested_search_mm = fminf(
        (float)elapsed_ms * DOG_TROT_TOUCHDOWN_SEARCH_MM_S * 0.001f,
        DOG_TROT_TOUCHDOWN_SEARCH_MM);

    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        const uint8_t bit = (uint8_t)(1U << leg);
        if ((s_march.swing_mask & bit) == 0U) {
            continue;
        }
        s_march.swing_progress[leg] = 1.0f;
        if ((s_march.contact_mask & bit) != 0U) {
            s_march.contact_search_mask &= (uint8_t)~bit;
            continue;
        }

        s_march.contact_search_mask |= bit;
        s_march.contact_search_mm[leg] = requested_search_mm;
        float nominal_z = dog_leg_stand_foot_z_mm(leg) + DOG_TROT_FOOT_Z1_MM +
                          s_leg_touchdown_z_offset[leg];
        s_march.frozen_z_mm[leg] = nominal_z + requested_search_mm;
        march_trot_set_swing_foot(leg, s_march.frozen_x_mm[leg],
                                  s_march.frozen_z_mm[leg]);

        if ((requested_search_mm >= DOG_TROT_TOUCHDOWN_SEARCH_MM) &&
            (march_leg_joints_stable(leg, DOG_TROT_SETTLE_ERR_DEG,
                                     DOG_TROT_TOUCHDOWN_VEL_DPS) != 0U)) {
            if (s_march.contact_stable_since_ms[leg] == 0U) {
                s_march.contact_stable_since_ms[leg] = now;
            } else if ((uint32_t)(now - s_march.contact_stable_since_ms[leg]) >=
                       DOG_TROT_TOUCHDOWN_FALLBACK_MS) {
                s_march.contact_mask |= bit;
                s_march.contact_search_mask &= (uint8_t)~bit;
                s_leg_foot_x_offset[leg] = s_march.frozen_x_mm[leg];
                s_leg_touchdown_z_offset[leg] = clampf(
                    s_leg_touchdown_z_offset[leg] + requested_search_mm,
                    -DOG_TROT_TOUCHDOWN_SEARCH_MM,
                    DOG_TROT_TOUCHDOWN_SEARCH_MM);
                continue;
            }
        } else {
            s_march.contact_stable_since_ms[leg] = 0U;
        }

        if (requested_search_mm >= DOG_TROT_TOUCHDOWN_SEARCH_MM) {
            if (s_march.search_limit_since_ms[leg] == 0U) {
                s_march.search_limit_since_ms[leg] = now;
            } else if ((uint32_t)(now - s_march.search_limit_since_ms[leg]) >=
                       DOG_TROT_TOUCHDOWN_FALLBACK_MS) {
                s_march.contact_failure_mask |= bit;
            }
        }
    }
}

static void march_begin_stop_neutral(uint32_t now);

static void march_begin_swing_up(uint32_t now)
{
    uint8_t leg_a = 0U;
    uint8_t leg_b = DOG_LEG_COUNT;
    march_get_swing_legs(&leg_a, &leg_b);

    if (march_mode_uses_cycloid(s_march.mode) != 0U) {
        s_march.stop_progress = 0.0f;
        march_snapshot_drive_command();
        if (march_gait_corners_reachable() == 0U) {
            DebugUart_Printf("Drive target unreachable; stopping at neutral.\r\n");
            march_begin_stop_neutral(now);
            return;
        }
        march_refresh_support_legs(leg_a, leg_b);
        s_march.swing_t0_ms = now;
        s_march.trot_stride_applied = 0U;
        march_trot_update_swing_pid_mask(now);
        march_contact_samples_begin(leg_a, leg_b);
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
    if (s_march.active == 0U) {
        return;
    }

    memset(&s_march, 0, sizeof(s_march));
    memset(s_leg_foot_x_offset, 0, sizeof(s_leg_foot_x_offset));
    memset(s_leg_touchdown_z_offset, 0, sizeof(s_leg_touchdown_z_offset));
    memset(s_leg_hip_offset_deg, 0, sizeof(s_leg_hip_offset_deg));
    mit_set_all_stand_pid_mode();
    march_set_all_legs_stand_pose();
    dog_mit_reset_integrators();
}

void dog_mit_march_request_stop(void)
{
    if (s_march.active == 0U) {
        return;
    }
    if (s_march.phase == DOG_MARCH_PHASE_ENTRY_SETTLE) {
        dog_mit_march_in_place_stop();
        return;
    }
    s_march.stop_requested = 1U;
}

static void march_begin_stop_neutral(uint32_t now)
{
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        s_march.stop_start_x_mm[leg] = s_leg_command_x_mm[leg];
        s_march.stop_start_z_mm[leg] = s_leg_command_z_mm[leg];
    }
    mit_set_all_stand_pid_mode();
    s_march.phase = DOG_MARCH_PHASE_STOP_NEUTRAL;
    s_march.phase_t0_ms = now;
    s_march.stop_progress = 0.0f;
}

static void march_stop_neutral_tick(uint32_t now)
{
    const uint32_t elapsed_ms = (uint32_t)(now - s_march.phase_t0_ms);
    const float raw_progress = (elapsed_ms >= DOG_GAIT_STOP_NEUTRAL_MS) ? 1.0f :
        ((float)elapsed_ms / (float)DOG_GAIT_STOP_NEUTRAL_MS);
    const float progress = smoothstep5_01(raw_progress);
    s_march.stop_progress = progress;

    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        const float x_mm = s_march.stop_start_x_mm[leg] * (1.0f - progress);
        const float stand_z_mm = dog_leg_stand_foot_z_mm(leg);
        const float z_mm = s_march.stop_start_z_mm[leg] +
                           (stand_z_mm - s_march.stop_start_z_mm[leg]) * progress;
        march_set_leg_foot_xz(leg, x_mm, z_mm);
    }

    if (elapsed_ms >= DOG_GAIT_STOP_NEUTRAL_MS) {
        dog_mit_march_in_place_stop();
    }
}

static void march_advance_step(uint32_t now)
{
    if (march_mode_uses_cycloid(s_march.mode) != 0U) {
        s_march.leg = (uint8_t)((s_march.leg + 1U) % 2U);
    } else {
        s_march.leg = (uint8_t)((s_march.leg + 1U) % DOG_LEG_COUNT);
    }

    if (s_march.leg == 0U) {
        if (s_march.stop_requested != 0U) {
            if (march_mode_uses_cycloid(s_march.mode) != 0U) {
                march_begin_stop_neutral(now);
            } else {
                dog_mit_march_in_place_stop();
            }
            return;
        }
        if (s_march.cycles_remaining > 0U) {
            s_march.cycles_remaining--;
            if (s_march.cycles_remaining == 0U) {
                if (march_mode_uses_cycloid(s_march.mode) != 0U) {
                    march_begin_stop_neutral(now);
                } else {
                    dog_mit_march_in_place_stop();
                }
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

    if ((s_march.phase == DOG_MARCH_PHASE_ENTRY_SETTLE) ||
        (s_march.phase == DOG_MARCH_PHASE_TOUCHDOWN)) {
        march_stability_velocity_filter_update(now);
    }

    if ((cycloid_gait != 0U) && (s_march.phase == DOG_MARCH_PHASE_SWING_UP)) {
        march_trot_apply_swing_trajectory(now);
        march_trot_apply_support_stance(now);
        march_trot_finish_stride(now);
        march_contact_samples_update(now);
    } else if ((cycloid_gait != 0U) && (s_march.phase == DOG_MARCH_PHASE_TOUCHDOWN)) {
        march_contact_samples_update(now);
    }

    switch (s_march.phase) {
    case DOG_MARCH_PHASE_ENTRY_SETTLE: {
        const uint8_t low_profile = (dog_mit_get_gait_speed_profile() == DOG_GAIT_SPEED_LOW) ? 1U : 0U;
        const float entry_err_deg = (low_profile != 0U) ?
            DOG_GAIT_LOW_ENTRY_ERR_DEG : DOG_GAIT_ENTRY_ERR_DEG;
        const float entry_vel_dps = (low_profile != 0U) ?
            DOG_GAIT_LOW_ENTRY_VEL_DPS : DOG_GAIT_ENTRY_VEL_DPS;
        const uint32_t entry_stable_ms = (low_profile != 0U) ?
            DOG_GAIT_LOW_ENTRY_STABLE_MS : DOG_GAIT_ENTRY_STABLE_MS;
        const uint32_t entry_timeout_ms = (low_profile != 0U) ?
            DOG_GAIT_LOW_ENTRY_TIMEOUT_MS : DOG_GAIT_ENTRY_TIMEOUT_MS;
        if (march_all_legs_stable(entry_err_deg, entry_vel_dps) != 0U) {
            if (s_march.entry_stable_since_ms == 0U) {
                s_march.entry_stable_since_ms = now;
            } else if ((uint32_t)(now - s_march.entry_stable_since_ms) >=
                       entry_stable_ms) {
                march_begin_swing_up(now);
            }
        } else {
            s_march.entry_stable_since_ms = 0U;
        }
        if ((s_march.active != 0U) &&
            ((uint32_t)(now - s_march.phase_t0_ms) >= entry_timeout_ms)) {
            DebugUart_Printf("Gait start settle timeout; holding stand.\r\n");
            dog_mit_march_in_place_stop();
        }
        break;
    }

    case DOG_MARCH_PHASE_SWING_UP:
        if (cycloid_gait != 0U) {
            const uint32_t elapsed_ms = (uint32_t)(now - s_march.swing_t0_ms);
            const uint32_t swing_ms = s_march.active_swing_ms;
            if (elapsed_ms >= swing_ms) {
                mit_set_all_stand_pid_mode();
                s_march.phase = DOG_MARCH_PHASE_TOUCHDOWN;
                s_march.phase_t0_ms = now;
                s_march.touchdown_stable_since_ms = 0U;
                march_stability_velocity_filter_reset(now);
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

    case DOG_MARCH_PHASE_TOUCHDOWN: {
        const uint32_t elapsed_ms = (uint32_t)(now - s_march.phase_t0_ms);
        const uint32_t dwell_ms = s_march.active_touchdown_dwell_ms;
        march_touchdown_search_update(now);
        if (s_march.contact_failure_mask != 0U) {
            DebugUart_Printf("Trot no contact mask=0x%02X search=%ld/%ld/%ld/%ld um; stopping neutral.\r\n",
                             (unsigned)s_march.contact_failure_mask,
                             (long)(s_march.contact_search_mm[0U] * 1000.0f),
                             (long)(s_march.contact_search_mm[1U] * 1000.0f),
                             (long)(s_march.contact_search_mm[2U] * 1000.0f),
                             (long)(s_march.contact_search_mm[3U] * 1000.0f));
            march_begin_stop_neutral(now);
            break;
        }
        const uint8_t current_contact = march_swing_legs_contact_detected();
        if (s_march.active_speed_profile == DOG_GAIT_SPEED_LOW) {
            const uint8_t stable = march_all_legs_stable(DOG_GAIT_LOW_TOUCHDOWN_ERR_DEG,
                                                          DOG_GAIT_LOW_TOUCHDOWN_VEL_DPS);
            const uint8_t support_margin =
                march_current_pair_has_support_margin(DOG_GAIT_LOW_SUPPORT_CURRENT_A);
            if ((stable != 0U) && (support_margin != 0U)) {
                if (s_march.touchdown_stable_since_ms == 0U) {
                    s_march.touchdown_stable_since_ms = now;
                }
            } else {
                s_march.touchdown_stable_since_ms = 0U;
            }
            const uint8_t stable_window_ready =
                ((s_march.touchdown_stable_since_ms != 0U) &&
                 ((uint32_t)(now - s_march.touchdown_stable_since_ms) >=
                  DOG_GAIT_LOW_TOUCHDOWN_STABLE_MS)) ? 1U : 0U;
            if ((elapsed_ms >= dwell_ms) && (stable_window_ready != 0U) &&
                (current_contact != 0U)) {
                march_advance_step(now);
            } else if (elapsed_ms >= DOG_GAIT_LOW_TOUCHDOWN_TIMEOUT_MS) {
                DebugUart_Printf("LOW support settle timeout; stopping at neutral.\r\n");
                march_begin_stop_neutral(now);
            }
        } else {
            if ((elapsed_ms >= dwell_ms) && (current_contact != 0U)) {
                march_advance_step(now);
            }
        }
        break;
    }

    case DOG_MARCH_PHASE_STOP_NEUTRAL:
        march_stop_neutral_tick(now);
        break;

    case DOG_MARCH_PHASE_HOLD:
        if ((uint32_t)(now - s_march.phase_t0_ms) >=
            ((cycloid_gait != 0U) ? DOG_TROT_HOLD_MS : march_walk_hold_ms())) {
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
            ((cycloid_gait != 0U) ? DOG_MARCH_LEG_PAUSE_MS : march_walk_leg_pause_ms())) {
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

uint8_t dog_mit_march_in_place_is_stopping(void)
{
    return ((s_march.active != 0U) &&
            ((s_march.stop_requested != 0U) ||
             (s_march.phase == DOG_MARCH_PHASE_STOP_NEUTRAL))) ? 1U : 0U;
}

uint8_t dog_mit_trot_march_is_active(void)
{
    return ((s_march.active != 0U) && (s_march.mode == DOG_MARCH_MODE_TROT)) ? 1U : 0U;
}

uint8_t dog_mit_turn_march_is_active(void)
{
    return ((s_march.active != 0U) && (s_march.mode == DOG_MARCH_MODE_TROT) &&
            (fabsf(s_march.requested_forward) < 0.05f) &&
            (fabsf(s_march.requested_yaw) >= 0.05f)) ? 1U : 0U;
}

static uint8_t march_in_place_start_mode(uint8_t mode, uint8_t cycles,
                                         float forward, float yaw)
{
    if ((!isfinite(forward)) || (!isfinite(yaw))) {
        return 0U;
    }
    forward = clampf(forward, -1.0f, 1.0f);
    yaw = clampf(yaw, -1.0f, 1.0f);
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
    if ((march_mode_uses_cycloid(mode) != 0U) &&
        (fabsf(forward) < 1.0e-3f) && (fabsf(yaw) < 1.0e-3f)) {
        return 0U;
    }

    dog_mit_march_in_place_stop();
    dog_mit_diag_support_stop();
    mit_set_all_stand_pid_mode();
    march_set_all_legs_stand_pose();
    dog_mit_reset_integrators();

    memset(&s_march, 0, sizeof(s_march));
    memset(s_leg_foot_x_offset, 0, sizeof(s_leg_foot_x_offset));
    memset(s_leg_hip_offset_deg, 0, sizeof(s_leg_hip_offset_deg));
    if ((march_mode_uses_cycloid(mode) != 0U) &&
        (march_gait_stride_reachable(mode, forward, yaw) == 0U)) {
        return 0U;
    }
    s_march.active = 1U;
    s_march.mode = mode;
    s_march.cycles_remaining = cycles;
    s_march.active_speed_profile = dog_mit_get_gait_speed_profile();
    s_march.requested_forward = forward;
    s_march.requested_yaw = yaw;
    s_march.active_leg_contribution = 1.0f;
    s_march.phase = DOG_MARCH_PHASE_ENTRY_SETTLE;
    s_march.phase_t0_ms = HAL_GetTick();
    s_march.entry_stable_since_ms = 0U;
    march_stability_velocity_filter_reset(s_march.phase_t0_ms);

    if (march_mode_uses_cycloid(mode) != 0U) {
        DebugUart_Printf("Drive cycloid: f=%ld yaw=%ld speed=%s hz=%ld.%01ld step=%ldmm turn=%ldmm height=%ldmm swing=%lums dwell=%lums stagger=%lums\r\n",
                         (long)(forward * 1000.0f),
                         (long)(yaw * 1000.0f),
                         dog_mit_gait_speed_profile_name(),
                         (long)dog_mit_gait_trot_hz(),
                         (long)(dog_mit_gait_trot_hz() * 10.0f) % 10L,
                         (long)dog_mit_gait_forward_stride_x_mm(),
                         (long)dog_mit_gait_turn_stride_x_mm(),
                         (long)dog_mit_gait_swing_height_mm(),
                         (long)dog_mit_gait_trot_swing_ms(),
                         (unsigned long)dog_mit_gait_touchdown_dwell_ms(),
                         (unsigned long)dog_mit_gait_diagonal_stagger_ms());
    } else {
        DebugUart_Printf("March start: lift=%ldmm LF->RF->LB->RB, support STAND swing SWING, x=stop\r\n",
                         (long)DOG_MARCH_LIFT_Z_MM);
    }
    return 1U;
}

uint8_t dog_mit_march_in_place_start(uint8_t cycles)
{
    return march_in_place_start_mode(DOG_MARCH_MODE_WALK, cycles, 0.0f, 0.0f);
}

uint8_t dog_mit_drive_start(uint8_t cycles, float forward, float yaw)
{
    return march_in_place_start_mode(DOG_MARCH_MODE_TROT, cycles, forward, yaw);
}

uint8_t dog_mit_drive_update(float forward, float yaw)
{
    if ((s_march.active == 0U) || (s_march.mode != DOG_MARCH_MODE_TROT) ||
        (s_march.stop_requested != 0U) || (!isfinite(forward)) || (!isfinite(yaw))) {
        return 0U;
    }
    s_march.requested_forward = clampf(forward, -1.0f, 1.0f);
    s_march.requested_yaw = clampf(yaw, -1.0f, 1.0f);
    return 1U;
}

void dog_mit_drive_get_command(float *requested_forward, float *requested_yaw,
                               float *applied_forward, float *applied_yaw)
{
    if (requested_forward != nullptr) {
        *requested_forward = s_march.requested_forward;
    }
    if (requested_yaw != nullptr) {
        *requested_yaw = s_march.requested_yaw;
    }
    if (applied_forward != nullptr) {
        *applied_forward = s_march.applied_forward;
    }
    if (applied_yaw != nullptr) {
        *applied_yaw = s_march.applied_yaw;
    }
}

void dog_mit_set_gait_wheel_contribution(float contribution)
{
    if (!isfinite(contribution)) {
        contribution = 0.0f;
    }
    contribution = clampf(contribution, 0.0f, DOG_GAIT_WHEEL_MAX_CONTRIBUTION);
    taskENTER_CRITICAL();
    s_requested_gait_wheel_contribution = contribution;
    taskEXIT_CRITICAL();
}

void dog_mit_get_gait_sync_state(DogGaitSyncState *state)
{
    if (state == nullptr) {
        return;
    }
    memset(state, 0, sizeof(*state));
    taskENTER_CRITICAL();
    state->active = s_march.active;
    state->stopping = ((s_march.stop_requested != 0U) ||
                       (s_march.phase == DOG_MARCH_PHASE_STOP_NEUTRAL)) ? 1U : 0U;
    state->phase = s_march.phase;
    state->speed_profile = s_march.active_speed_profile;
    state->swing_mask = s_march.swing_mask;
    state->contact_mask = s_march.contact_mask;
    state->contact_search_mask = s_march.contact_search_mask;
    state->contact_failure_mask = s_march.contact_failure_mask;
    state->half_step_generation = s_march.half_step_generation;
    state->active_swing_ms = s_march.active_swing_ms;
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        state->swing_progress[leg] = s_march.swing_progress[leg];
        state->contact_search_mm[leg] = s_march.contact_search_mm[leg];
    }
    state->requested_forward = s_march.requested_forward;
    state->requested_yaw = s_march.requested_yaw;
    state->applied_forward = s_march.active_forward;
    state->applied_yaw = s_march.active_yaw;
    state->requested_wheel_contribution = s_requested_gait_wheel_contribution;
    state->active_wheel_contribution = s_march.active_wheel_contribution;
    state->active_leg_contribution = (s_march.active != 0U) ?
        s_march.active_leg_contribution : 1.0f;
    state->compatible_wheel_rpm = s_march.compatible_wheel_rpm;
    state->active_forward_stride_x_mm = s_march.active_forward_stride_x_mm;
    state->active_turn_stride_x_mm = s_march.active_turn_stride_x_mm;
    state->stop_progress = s_march.stop_progress;
    taskEXIT_CRITICAL();
}

uint8_t dog_mit_trot_in_place_start(uint8_t cycles)
{
    return dog_mit_drive_start(cycles, 1.0f, 0.0f);
}

uint8_t dog_mit_trot_reverse_in_place_start(uint8_t cycles)
{
    return dog_mit_drive_start(cycles, -1.0f, 0.0f);
}

uint8_t dog_mit_turn_left_in_place_start(uint8_t cycles)
{
    return dog_mit_drive_start(cycles, 0.0f, -1.0f);
}

uint8_t dog_mit_turn_right_in_place_start(uint8_t cycles)
{
    return dog_mit_drive_start(cycles, 0.0f, 1.0f);
}

static void diag_support_set_lift_leg(uint8_t leg)
{
    const float lift_z = dog_leg_stand_foot_z_mm(leg) - DOG_DIAG_SUPPORT_LIFT_Z_MM;
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
        dog_debug_rx_only();
    }

    uint8_t ok_count = dog_debug_mit_boot_sequence();
    if (ok_count == 0U) {
        return 0U;
    }

    s_mit_pid_profile = DOG_MIT_PID_STAND;
    dog_mit_reset_integrators();
    DebugUart_Printf("Stand STAND_PID foot IK front (%ld,%ld)->(%ld,%ld) rear (%ld,%ld)->(%ld,%ld)mm linear=%lums Ol=%ldmA\r\n",
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
        return 0U;
    }

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
        if (mit_pump_control() == 0U) {
            return 0U;
        }

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
    uint32_t safety_generation = 0U;
    if (motor_safety_token_acquire(&safety_generation) == 0U) {
        DebugUart_Printf("TqTest blocked: safety latch active.\r\n");
        return 0U;
    }

    uint8_t idx = motor_index(bus, node_id);
    if ((idx >= DOG_MOTOR_COUNT) || (!isfinite(torque_nm))) {
        return 0U;
    }

    dog_debug_mit_torque_stop();
    s_mit_debug_active = 0U;
    teach_hold_stop();
    s_position_tx_enabled = 0U;
    memset(s_motor_mit_probe_active, 0, sizeof(s_motor_mit_probe_active));

    Dog_Motor_Config *cfg = &g_dog_motor_config[idx];
    if (motor_heartbeat_fresh(idx, HAL_GetTick()) == 0U) {
        DebugUart_Printf("TqTest skip M%u(bus%u id%u): offline\r\n",
                         (unsigned)idx, (unsigned)bus, (unsigned)node_id);
        return 0U;
    }
    if (motor_has_fault(idx) != 0U) {
        DebugUart_Printf("TqTest skip M%u(bus%u id%u): fault\r\n",
                         (unsigned)idx, (unsigned)bus, (unsigned)node_id);
        return 0U;
    }

    if (motor_safety_token_valid(safety_generation) == 0U) {
        return 0U;
    }
    MWClearErrors(cfg->bus, cfg->node_id);
    if ((motor_safety_token_valid(safety_generation) == 0U) ||
        (motor_heartbeat_fresh(idx, HAL_GetTick()) == 0U)) {
        return 0U;
    }
    const uint8_t cl_result = request_mit_probe_for_angle(idx, DOG_ENCODER_WAIT_MS);
    if ((cl_result != 1U) || (motor_safety_token_valid(safety_generation) == 0U)) {
        DebugUart_Printf("TqTest FAIL M%u(bus%u id%u): cl=%u\r\n",
                         (unsigned)idx, (unsigned)bus, (unsigned)node_id,
                         (unsigned)cl_result);
        dog_mit_protect_hold();
        return 0U;
    }

    configure_motor_mit(idx);
    const uint32_t now = HAL_GetTick();
    if ((s_motor_configured[idx] == 0U) || (motor_ready(idx) == 0U) ||
        (motor_encoder_fresh(idx, now) == 0U) ||
        (motor_safety_token_valid(safety_generation) == 0U)) {
        dog_mit_protect_hold();
        return 0U;
    }
    s_motor_mit_probe_active[idx] = 0U;

    if (motor_safety_token_guard_take(safety_generation) == 0U) {
        return 0U;
    }
    s_mit_torque_test_index = idx;
    s_mit_torque_test_nm = torque_nm;
    s_mit_torque_test_active = 1U;
    motor_tx_guard_give();

    const uint8_t di = bus_to_diag_index(cfg->bus);
    const uint32_t drops_before = s_can_tx_drop_count[di];
    send_mit_fixed_torque(idx, torque_nm);
    if ((s_can_tx_drop_count[di] != drops_before) ||
        (motor_ready(idx) == 0U) ||
        (motor_encoder_fresh(idx, HAL_GetTick()) == 0U) ||
        (motor_safety_token_valid(safety_generation) == 0U)) {
        dog_mit_protect_hold();
        return 0U;
    }
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
    s_auto_stand_enabled = 0U;
    s_position_tx_arm_pending_mask = 0U;
    s_position_tx_arm_started_ms = 0U;
    memset(s_motor_configured, 0, sizeof(s_motor_configured));
    memset(s_motor_loop_requested, 0, sizeof(s_motor_loop_requested));
    memset(s_motor_final_mode_pending, 0, sizeof(s_motor_final_mode_pending));
    memset(s_position_idle_encoder_rx_baseline, 0, sizeof(s_position_idle_encoder_rx_baseline));
    memset(s_motor_mit_probe_active, 0, sizeof(s_motor_mit_probe_active));
    memset(s_encoder_turn_valid, 0, sizeof(s_encoder_turn_valid));
    if (s_stand_state != DOG_STAND_ESTOP) {
        s_stand_state = DOG_STAND_IDLE;
    }
}

void dog_debug_clear_errors(void)
{
    if (s_safety_latched != 0U) {
        return;
    }
    for (uint8_t i = 0U; i < selected_count(); ++i) {
        uint8_t idx = selected_index(i);
        if (idx < DOG_MOTOR_COUNT) {
            MWClearErrors(g_dog_motor_config[idx].bus, g_dog_motor_config[idx].node_id);
        }
    }
}

void dog_debug_position_setup(void)
{
    if (s_safety_latched != 0U) {
        return;
    }
    mit_debug_stop_tx();
    s_position_tx_arm_pending_mask = 0U;
    s_position_tx_arm_started_ms = 0U;
    s_control_loop_mode = DOG_CTRL_LOOP_POSITION;
    for (uint8_t i = 0U; i < selected_count(); ++i) {
        if (motor_blocking_service(nullptr) == 0U) {
            return;
        }
        const uint8_t index = selected_index(i);
        if ((index >= DOG_MOTOR_COUNT) ||
            (motor_heartbeat_fresh(index, HAL_GetTick()) == 0U) ||
            (motor_has_fault(index) != 0U)) {
            continue;
        }
        s_motor_loop_requested[index] = 0U;
        s_motor_final_mode_pending[index] = DOG_FINAL_MODE_NONE;
        s_motor_mit_probe_active[index] = 0U;
        s_motor_configured[index] = 0U;
        if (g_mw_motor_data[index].heartBeat.currentState == MW_AXIS_STATE_IDLE) {
            configure_motor_position(index);
        } else {
            Dog_Motor_Config *cfg = &g_dog_motor_config[index];
            MWSetAxisState(cfg->bus, cfg->node_id, MW_AXIS_STATE_IDLE);
        }
        HAL_Delay(5U);
    }
}

void dog_debug_mit_setup(void)
{
    if (s_safety_latched != 0U) {
        return;
    }
    teach_hold_stop();
    s_control_loop_mode = DOG_CTRL_LOOP_MIT_PID;
    dog_mit_reset_integrators();
    for (uint8_t i = 0U; i < selected_count(); ++i) {
        if (motor_blocking_service(nullptr) == 0U) {
            return;
        }
        configure_motor_mit(selected_index(i));
        HAL_Delay(2U);
    }
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
    if ((index >= DOG_MOTOR_COUNT) || (s_safety_latched != 0U)) {
        return 0U;
    }

    for (uint8_t n = 0U; n < 20U; ++n) {
        if (motor_blocking_service(nullptr) == 0U) {
            return 0U;
        }
        send_mit_zero_effort(index, 0.0f);
        mw_query_encoder_estimate(index);
        HAL_Delay(5U);
    }

    if (motor_blocking_service(nullptr) == 0U) {
        return 0U;
    }
    if ((motor_closed_loop(index) == 0U) || (s_encoder_est_fresh[index] == 0U)) {
        return 0U;
    }

    s_zero_offset_turn[index] = g_mw_motor_data[index].encoderEstimates.encoderPosEstimate;
    s_encoder_turn_filt[index] = s_zero_offset_turn[index];
    s_encoder_turn_valid[index] = 1U;
    return 1U;
}

static uint8_t next_booted_motor_on_bus(uint8_t bus, uint8_t *cursor)
{
    if (cursor == nullptr) {
        return DOG_MOTOR_COUNT;
    }

    const uint8_t start = (uint8_t)(*cursor % DOG_MOTOR_COUNT);
    for (uint8_t offset = 0U; offset < DOG_MOTOR_COUNT; ++offset) {
        const uint8_t index = (uint8_t)((start + offset) % DOG_MOTOR_COUNT);
        if ((g_dog_motor_config[index].bus == bus) && (s_mit_boot_ok[index] != 0U)) {
            *cursor = (uint8_t)((index + 1U) % DOG_MOTOR_COUNT);
            return index;
        }
    }
    return DOG_MOTOR_COUNT;
}

static uint8_t mit_debug_settle_and_arm(void)
{
    const uint32_t t0 = HAL_GetTick();
    uint32_t last_slot_ms = 0U;
    uint8_t cursor[2U] = {0U, 0U};
    const uint8_t buses[2U] = {DOG_CAN_FRONT_BUS, DOG_CAN_REAR_BUS};

    while ((uint32_t)(HAL_GetTick() - t0) < DOG_MIT_DEBUG_SETTLE_MS) {
        uint32_t now = 0U;
        if (motor_blocking_service(&now) == 0U) {
            return 0U;
        }

        if ((uint32_t)(now - last_slot_ms) >= DOG_ENCODER_FEEDBACK_PERIOD_MS) {
            last_slot_ms = now;
            for (uint8_t di = 0U; di < 2U; ++di) {
                const uint8_t index = next_booted_motor_on_bus(buses[di], &cursor[di]);
                if (index < DOG_MOTOR_COUNT) {
                    send_mit_zero_effort(index, 0.0f);
                    mw_query_encoder_estimate(index);
                }
            }
        }
        HAL_Delay(1U);
    }

    uint32_t now = 0U;
    if (motor_blocking_service(&now) == 0U) {
        return 0U;
    }

    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if (s_mit_boot_ok[i] == 0U) {
            continue;
        }
        const uint8_t heartbeat_fresh = ((s_last_heartbeat_tick_ms[i] != 0U) &&
            ((uint32_t)(now - s_last_heartbeat_tick_ms[i]) <= DOG_HEARTBEAT_TIMEOUT_MS)) ? 1U : 0U;
        const uint8_t encoder_fresh = ((s_encoder_est_fresh[i] != 0U) &&
            ((uint32_t)(now - s_last_encoder_tick_ms[i]) <= DOG_ENCODER_FEEDBACK_TIMEOUT_MS)) ? 1U : 0U;
        if ((s_motor_online[i] == 0U) || (heartbeat_fresh == 0U) ||
            (encoder_fresh == 0U) || (motor_closed_loop(i) == 0U) ||
            (motor_has_fault(i) != 0U)) {
            mit_debug_abort_control("MIT settle feedback/fault");
            return 0U;
        }
        s_target_deg[i] = user_deg(i);
        s_target_turn[i] = g_mw_motor_data[i].encoderEstimates.encoderPosEstimate;
        mit_reset_motor_integrator(i);
    }
    return 1U;
}

uint8_t dog_debug_mit_boot_sequence(void)
{
    uint32_t safety_generation = 0U;
    if (motor_safety_token_acquire(&safety_generation) == 0U) {
        DebugUart_Printf("MIT boot blocked: safety latch active.\r\n");
        return 0U;
    }

    uint8_t ok_count = 0U;

    s_position_tx_enabled = 0U;
    s_mit_debug_active = 0U;
    s_mit_fault_hold_active = 0U;
    teach_hold_stop();
    memset(s_mit_boot_ok, 0, sizeof(s_mit_boot_ok));

    for (uint8_t i = 0U; i < selected_count(); ++i) {
        if (s_safety_latched != 0U) {
            return 0U;
        }
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
        if (s_safety_latched != 0U) {
            return 0U;
        }
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
            if (s_safety_latched != 0U) {
                return 0U;
            }
            DebugUart_Printf("BOOT zero-fail M%u(bus%u id%u)\r\n",
                             (unsigned)idx, (unsigned)cfg->bus, (unsigned)cfg->node_id);
            continue;
        }

        s_target_turn[idx] = s_zero_offset_turn[idx];
        s_target_deg[idx] = 0.0f;
        if (configure_motor_mit_for_boot(idx) == 0U) {
            continue;
        }
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
        dog_mit_protect_hold();
        DebugUart_Printf("Stand boot deferred: no motor ready; holding and retrying.\r\n");
        return 0U;
    }
    if (ok_count != selected_count()) {
        DebugUart_Printf("Stand FAIL: boot OK %u/%u, wait all selected motors online/ready.\r\n",
                         (unsigned)ok_count,
                         (unsigned)selected_count());
        dog_mit_protect_hold();
        DebugUart_Printf("Stand boot deferred: transient readiness failure; holding and retrying.\r\n");
        return 0U;
    }

    if (mit_debug_settle_and_arm() == 0U) {
        if (s_safety_latched == 0U) {
            mit_debug_abort_control("MIT settle aborted");
        }
        return 0U;
    }

    if (motor_safety_token_guard_take(safety_generation) == 0U) {
        return 0U;
    }
    memset(s_motor_mit_probe_active, 0, sizeof(s_motor_mit_probe_active));
    s_control_loop_mode = DOG_CTRL_LOOP_MIT_PID;
    s_mit_debug_active = 1U;
    s_mit_fault_hold_active = 0U;
    s_position_tx_enabled = 1U;
    s_last_command_tick_ms = 0U;
    motor_tx_guard_give();
    return ok_count;
}

uint8_t dog_debug_teach_hold_start(void)
{
    uint32_t safety_generation = 0U;
    if (motor_safety_token_acquire(&safety_generation) == 0U) {
        DebugUart_Printf("Teach-hold blocked: safety latch active.\r\n");
        return 0U;
    }

    uint8_t ok_count = 0U;

    s_position_tx_enabled = 0U;
    s_auto_stand_enabled = 0U;
    s_mit_debug_active = 0U;
    s_mit_fault_hold_active = 0U;
    teach_hold_stop();
    memset(s_mit_boot_ok, 0, sizeof(s_mit_boot_ok));

    for (uint8_t i = 0U; i < selected_count(); ++i) {
        if (s_safety_latched != 0U) {
            return 0U;
        }
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
        if (s_safety_latched != 0U) {
            return 0U;
        }
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

        configure_motor_mit(idx);
        if ((s_motor_configured[idx] == 0U) ||
            (motor_safety_token_valid(safety_generation) == 0U)) {
            continue;
        }
        s_mit_boot_ok[idx] = 1U;
        ok_count++;

        long user_mdeg = (long)(user_deg(idx) * 1000.0f);
        long abs_user = (user_mdeg < 0L) ? -user_mdeg : user_mdeg;
        DebugUart_Printf("HOLD ok M%u(bus%u id%u) user=%c%ld.%03ld deg\r\n",
                         (unsigned)idx,
                         (unsigned)cfg->bus,
                         (unsigned)cfg->node_id,
                         (user_mdeg < 0L) ? '-' : '+',
                         abs_user / 1000L,
                         abs_user % 1000L);
    }

    if (ok_count != selected_count()) {
        dog_mit_protect_hold();
        return 0U;
    }

    if ((mit_debug_settle_and_arm() == 0U) ||
        (motor_safety_token_guard_take(safety_generation) == 0U)) {
        if (s_safety_latched == 0U) {
            dog_mit_protect_hold();
        }
        return 0U;
    }

    const uint32_t now = HAL_GetTick();
    for (uint8_t i = 0U; i < selected_count(); ++i) {
        const uint8_t index = selected_index(i);
        if (index < DOG_MOTOR_COUNT) {
            s_teach_hold_following[index] = 1U;
            s_teach_hold_last_motion_ms[index] = now;
        }
    }
    memset(s_motor_mit_probe_active, 0, sizeof(s_motor_mit_probe_active));
    s_control_loop_mode = DOG_CTRL_LOOP_MIT_PID;
    s_mit_debug_active = 1U;
    s_mit_fault_hold_active = 0U;
    s_teach_hold_active = 1U;
    s_position_tx_enabled = 1U;
    s_last_command_tick_ms = 0U;
    s_mit_last_pid_ms = 0U;
    motor_tx_guard_give();
    return ok_count;
}

void dog_debug_enter_closed_loop(void)
{
    if (s_safety_latched != 0U) {
        DebugUart_Printf("Closed-loop blocked: safety latch active.\r\n");
        return;
    }

    s_position_tx_enabled = 0U;
    s_mit_debug_active = 0U;
    s_mit_fault_hold_active = 0U;
    teach_hold_stop();

    for (uint8_t i = 0U; i < selected_count(); ++i) {
        if (s_safety_latched != 0U) {
            return;
        }
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
        if (s_safety_latched != 0U) {
            return;
        }
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

static void position_tx_arm_tick(uint32_t now)
{
    const uint8_t mask = s_position_tx_arm_pending_mask;
    if (mask == 0U) {
        return;
    }
    if ((uint32_t)(now - s_position_tx_arm_started_ms) >= DOG_POSITION_ARM_TIMEOUT_MS) {
        s_position_tx_arm_pending_mask = 0U;
        mit_debug_abort_control("position finalization timeout");
        return;
    }

    uint32_t safety_generation = 0U;
    if (motor_safety_token_acquire(&safety_generation) == 0U) {
        s_position_tx_arm_pending_mask = 0U;
        s_position_tx_arm_started_ms = 0U;
        return;
    }

    for (uint8_t index = 0U; index < DOG_MOTOR_COUNT; ++index) {
        if ((mask & (uint8_t)(1U << index)) == 0U) {
            continue;
        }
        if ((motor_heartbeat_fresh(index, now) == 0U) || (motor_has_fault(index) != 0U)) {
            s_position_tx_arm_pending_mask = 0U;
            s_position_tx_arm_started_ms = 0U;
            mit_debug_abort_control("position finalization feedback/fault");
            return;
        }
        if ((motor_ready(index) == 0U) || (motor_encoder_fresh(index, now) == 0U) ||
            (s_motor_configured[index] == 0U) ||
            (s_motor_mit_probe_active[index] != 0U) ||
            (s_motor_final_mode_pending[index] != DOG_FINAL_MODE_NONE)) {
            return;
        }
        if (!isfinite(s_target_turn[index])) {
            s_position_tx_arm_pending_mask = 0U;
            s_position_tx_arm_started_ms = 0U;
            mit_debug_abort_control("non-finite position target");
            return;
        }
    }

    if (motor_safety_token_guard_take(safety_generation) == 0U) {
        return;
    }
    if (s_position_tx_arm_pending_mask == mask) {
        s_position_tx_arm_pending_mask = 0U;
        s_position_tx_arm_started_ms = 0U;
        s_position_tx_enabled = 1U;
        s_last_command_tick_ms = 0U;
    }
    motor_tx_guard_give();
    if ((s_safety_latched == 0U) && (s_position_tx_enabled != 0U)) {
        DebugUart_Printf("Position mode finalized at current pose; target TX enabled.\r\n");
    }
}

void dog_debug_start_position_tx(void)
{
    uint32_t safety_generation = 0U;
    if (motor_safety_token_acquire(&safety_generation) == 0U) {
        DebugUart_Printf("Position TX blocked: safety latch active.\r\n");
        return;
    }

    const uint32_t now = HAL_GetTick();
    const uint8_t mask = selected_motor_mask();
    for (uint8_t i = 0U; i < selected_count(); ++i) {
        uint8_t idx = selected_index(i);
        if ((idx < DOG_MOTOR_COUNT) &&
            ((motor_heartbeat_fresh(idx, now) == 0U) || (motor_has_fault(idx) != 0U))) {
            DebugUart_Printf("Start ignored: target index=%u heartbeat/fault not ready.\r\n",
                             (unsigned)idx);
            return;
        }
    }

    if (s_control_loop_mode == DOG_CTRL_LOOP_POSITION) {
        if (motor_safety_token_guard_take(safety_generation) == 0U) {
            return;
        }
        s_position_tx_enabled = 0U;
        s_position_tx_arm_pending_mask = mask;
        s_position_tx_arm_started_ms = now;
        for (uint8_t index = 0U; index < DOG_MOTOR_COUNT; ++index) {
            if ((mask & (uint8_t)(1U << index)) != 0U) {
                s_motor_loop_requested[index] = 1U;
            }
        }
        motor_tx_guard_give();

        position_tx_arm_tick(HAL_GetTick());
        if (s_position_tx_arm_pending_mask != 0U) {
            DebugUart_Printf("Position TX waiting for safe IDLE/configure/hold/closed-loop finalization.\r\n");
        }
        return;
    }

    for (uint8_t i = 0U; i < selected_count(); ++i) {
        const uint8_t idx = selected_index(i);
        if ((idx >= DOG_MOTOR_COUNT) || (motor_ready(idx) == 0U) ||
            (motor_encoder_fresh(idx, now) == 0U)) {
            DebugUart_Printf("Start ignored: target index=%u not ready.\r\n", (unsigned)idx);
            return;
        }
    }
    s_position_tx_enabled = 1U;
    s_last_command_tick_ms = 0U;
    dog_mit_reset_integrators();
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
    uint32_t safety_generation = 0U;
    if (motor_safety_token_acquire(&safety_generation) == 0U) {
        DebugUart_Printf("Stand blocked: safety latch active.\r\n");
        return;
    }
    DogStand_ExitMechanicalLimitIdle();
    mit_debug_stop_tx();
    memset(s_motor_loop_requested, 0, sizeof(s_motor_loop_requested));
    memset(s_motor_final_mode_pending, 0, sizeof(s_motor_final_mode_pending));
    memset(s_motor_mit_probe_active, 0, sizeof(s_motor_mit_probe_active));
    if (motor_safety_token_guard_take(safety_generation) == 0U) {
        return;
    }
    s_auto_stand_enabled = 1U;
    s_stand_motor_cursor = 0U;
    s_state_start_ms = HAL_GetTick();
    s_stand_state = DOG_STAND_WAIT_HEARTBEAT;
    motor_tx_guard_give();
}

static uint8_t send_motor_estop_locked(uint8_t index)
{
    if (index >= DOG_MOTOR_COUNT) {
        return 0U;
    }

    Dog_Motor_Config *cfg = &g_dog_motor_config[index];
    FDCAN_HandleTypeDef *h = bus_handle(cfg->bus);
    if (h == nullptr) {
        return 0U;
    }

    uint8_t tx[8U] = {};
    const uint32_t can_id = ((uint32_t)cfg->node_id << 5) | MW_ESTOP_CMD;
    const uint8_t id_type = s_motor_can_id_type[index];
    uint8_t ok = 1U;
    if (id_type == DOG_CAN_ID_UNKNOWN) {
        const uint8_t ext_status = mw_send_can_frame(h, DOG_CAN_ID_EXTENDED, can_id, tx, 8U);
        const uint8_t std_status = mw_send_can_frame(h, DOG_CAN_ID_STANDARD, can_id, tx, 8U);
        mw_record_tx_result(cfg->bus, ext_status);
        mw_record_tx_result(cfg->bus, std_status);
        ok = ((ext_status == 0U) && (std_status == 0U)) ? 1U : 0U;
    } else {
        const uint8_t status = mw_send_can_frame(h, id_type, can_id, tx, 8U);
        mw_record_tx_result(cfg->bus, status);
        ok = (status == 0U) ? 1U : 0U;
    }
    return ok;
}

static void service_estop_pending_locked(void)
{
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        const uint8_t bit = (uint8_t)(1U << i);
        if (((s_estop_pending_mask & bit) != 0U) &&
            (send_motor_estop_locked(i) != 0U)) {
            s_estop_pending_mask &= (uint8_t)~bit;
        }
    }
}

static void queue_motor_estop(uint8_t index)
{
    if ((index >= DOG_MOTOR_COUNT) || (motor_tx_guard_take() == 0U)) {
        return;
    }
    if (s_safety_latched == 0U) {
        motor_tx_guard_give();
        return;
    }
    s_estop_pending_mask |= (uint8_t)(1U << index);
    service_estop_pending_locked();
    motor_tx_guard_give();
}

static void send_all_estop(void)
{
    if (motor_tx_guard_take() == 0U) {
        return;
    }
    s_estop_pending_mask = 0xFFU;
    service_estop_pending_locked();
    motor_tx_guard_give();
}

static uint8_t safety_clear_token_valid(uint32_t generation)
{
    if (motor_tx_guard_take() == 0U) {
        return 0U;
    }
    const uint8_t valid = ((s_safety_latched != 0U) &&
                           (s_safety_rearm_requested != 0U) &&
                           (s_safety_external_inhibit == 0U) &&
                           (s_safety_generation == generation) &&
                           (s_estop_pending_mask == 0U)) ? 1U : 0U;
    motor_tx_guard_give();
    return valid;
}

static uint8_t send_all_clear_errors(uint32_t generation)
{
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if (safety_clear_token_valid(generation) == 0U) {
            return 0U;
        }
        MWClearErrors(g_dog_motor_config[i].bus, g_dog_motor_config[i].node_id);
        if (safety_clear_token_valid(generation) == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t all_leg_motors_stopped_online(void)
{
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((s_motor_online[i] == 0U) || (motor_closed_loop(i) != 0U) ||
            ((int32_t)(s_last_heartbeat_tick_ms[i] - s_safety_latched_ms) <= 0)) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t all_leg_motors_clear_confirmed(void)
{
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((s_motor_online[i] == 0U) || (motor_closed_loop(i) != 0U) ||
            (motor_has_fault(i) != 0U) ||
            ((int32_t)(s_last_heartbeat_tick_ms[i] - s_safety_clear_started_ms) <= 0)) {
            return 0U;
        }
    }
    return 1U;
}

void DogStand_Disable(void)
{
    if (motor_tx_guard_take() == 0U) {
        ArmMotor_Disable();
        return;
    }

    if (s_safety_latched != 0U) {
        motor_tx_guard_give();
        ArmMotor_Disable();
        return;
    }

    /* Invalidate in-flight control work before removing queued motor commands. */
    s_safety_generation++;
    s_mechanical_idle_requested = 0U;
    s_mechanical_idle_ready = 0U;
    s_mechanical_idle_mask = 0U;
    s_mechanical_pose_requested = 0U;
    s_mechanical_pose_ready = 0U;
    s_mechanical_pose_mask = 0U;
    s_mechanical_idle_settle_since_ms = 0U;
    s_control_disabled = 1U;
    s_position_tx_enabled = 0U;
    s_position_tx_arm_pending_mask = 0U;
    s_position_tx_arm_started_ms = 0U;
    s_mit_debug_active = 0U;
    s_mit_fault_hold_active = 0U;
    s_mit_torque_test_active = 0U;
    s_mit_torque_test_index = DOG_MOTOR_COUNT;
    s_mit_torque_test_nm = 0.0f;
    s_jump_active = 0U;
    s_auto_stand_enabled = 0U;
    s_diag_support_active = 0U;
    s_last_command_tick_ms = 0U;
    s_mit_last_pid_ms = 0U;
    s_stand_state = DOG_STAND_IDLE;
    memset(&s_march, 0, sizeof(s_march));
    memset(s_leg_foot_x_offset, 0, sizeof(s_leg_foot_x_offset));
    memset(s_leg_hip_offset_deg, 0, sizeof(s_leg_hip_offset_deg));
    memset(s_motor_configured, 0, sizeof(s_motor_configured));
    memset(s_motor_loop_requested, 0, sizeof(s_motor_loop_requested));
    memset(s_motor_final_mode_pending, 0, sizeof(s_motor_final_mode_pending));
    memset(s_motor_mit_probe_active, 0, sizeof(s_motor_mit_probe_active));
    memset(s_position_idle_encoder_rx_baseline, 0, sizeof(s_position_idle_encoder_rx_baseline));
    memset(s_mit_boot_ok, 0, sizeof(s_mit_boot_ok));
    mit_clear_mixed_pid();
    teach_hold_stop();
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        s_target_deg[i] = user_deg(i);
        s_target_turn[i] = g_mw_motor_data[i].encoderEstimates.encoderPosEstimate;
        mit_reset_motor_integrator(i);
    }

    (void)fdcan_abort_all_tx(&hfdcan1);
    (void)fdcan_abort_all_tx(&hfdcan2);
    motor_tx_guard_give();

    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        MWSetAxisState(g_dog_motor_config[i].bus,
                       g_dog_motor_config[i].node_id,
                       MW_AXIS_STATE_IDLE);
    }
    ArmMotor_Disable();
}

uint8_t DogStand_ClearDisable(void)
{
    if (motor_tx_guard_take() == 0U) {
        return 0U;
    }
    if ((s_safety_latched != 0U) || (s_safety_external_inhibit != 0U)) {
        motor_tx_guard_give();
        return 0U;
    }
    if (s_control_disabled != 0U) {
        s_control_disabled = 0U;
        s_safety_generation++;
    }
    motor_tx_guard_give();
    return 1U;
}

uint8_t DogStand_IsDisabled(void)
{
    return s_control_disabled;
}

uint8_t DogStand_EnterMechanicalLimitPose(void)
{
    if ((s_safety_latched != 0U) || (s_safety_external_inhibit != 0U) ||
        (s_control_disabled != 0U)) {
        return 0U;
    }

    s_mechanical_pose_requested = 1U;
    s_mechanical_pose_ready = 0U;
    s_mechanical_pose_mask = 0U;
    dog_debug_set_target(DOG_DEBUG_TARGET_ALL);
    if (dog_debug_mit_boot_sequence() != DOG_MOTOR_COUNT) {
        s_mechanical_pose_requested = 0U;
        dog_mit_protect_hold();
        DebugUart_Printf("Mechanical wheel pose FAIL: MIT boot incomplete.\r\n");
        return 0U;
    }

    float hip_start_deg[DOG_LEG_COUNT] = {};
    float knee_hold_deg[DOG_LEG_COUNT] = {};
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        const uint8_t hip = leg_joint_index(leg, DOG_JOINT_HIP);
        const uint8_t knee = leg_joint_index(leg, DOG_JOINT_KNEE);
        if ((hip >= DOG_MOTOR_COUNT) || (knee >= DOG_MOTOR_COUNT) ||
            (!isfinite(user_deg(hip))) || (!isfinite(user_deg(knee)))) {
            s_mechanical_pose_requested = 0U;
            dog_mit_protect_hold();
            return 0U;
        }
        hip_start_deg[leg] = user_deg(hip);
        knee_hold_deg[leg] = user_deg(knee);
    }

    mit_set_all_stand_pid_mode();
    dog_mit_reset_integrators();
    const uint32_t t0 = HAL_GetTick();
    while (1) {
        const uint32_t now = HAL_GetTick();
        const float raw_progress = (DOG_LOW_WHEEL_HIP_LIFT_MS == 0U) ? 1.0f :
            clampf((float)(now - t0) / (float)DOG_LOW_WHEEL_HIP_LIFT_MS,
                   0.0f, 1.0f);
        const float progress = smoothstep5_01(raw_progress);
        for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
            dog_leg_set_motor_user_deg_for_leg(
                leg,
                hip_start_deg[leg] + DOG_LOW_WHEEL_HIP_LIFT_DEG * progress,
                knee_hold_deg[leg]);
        }
        if (mit_pump_control() == 0U) {
            s_mechanical_pose_requested = 0U;
            dog_mit_protect_hold();
            DebugUart_Printf("Mechanical wheel pose FAIL: control interrupted.\r\n");
            return 0U;
        }
        if (raw_progress >= 1.0f) {
            break;
        }
        HAL_Delay(1U);
    }

    if (mit_stand_wait_target_legs_settled(DOG_STAND_MOVE_MS) == 0U) {
        s_mechanical_pose_requested = 0U;
        dog_mit_protect_hold();
        DebugUart_Printf("Mechanical wheel pose FAIL: joint settle timeout.\r\n");
        return 0U;
    }
    s_mechanical_pose_mask = 0xFFU;
    s_mechanical_pose_ready = 1U;
    DebugUart_Printf("Mechanical wheel pose ready: hip=%+ldmdeg knee=hold.\r\n",
                     (long)(DOG_LOW_WHEEL_HIP_LIFT_DEG * 1000.0f));
    return 1U;
}

uint8_t DogStand_IsMechanicalLimitPose(void)
{
    return s_mechanical_pose_requested;
}

uint8_t DogStand_IsMechanicalLimitPoseReady(void)
{
    return ((s_mechanical_pose_requested != 0U) &&
            (s_mechanical_pose_ready != 0U)) ? 1U : 0U;
}

uint8_t DogStand_GetMechanicalLimitPoseMask(void)
{
    return s_mechanical_pose_mask;
}

void DogStand_ExitMechanicalLimitPose(void)
{
    s_mechanical_pose_requested = 0U;
    s_mechanical_pose_ready = 0U;
    s_mechanical_pose_mask = 0U;
}

uint8_t DogStand_EnterMechanicalLimitIdle(void)
{
    if (motor_tx_guard_take() == 0U) {
        ArmMotor_Disable();
        return 0U;
    }
    if ((s_safety_latched != 0U) || (s_safety_external_inhibit != 0U) ||
        (s_control_disabled != 0U)) {
        motor_tx_guard_give();
        ArmMotor_Disable();
        return 0U;
    }

    s_safety_generation++;
    s_mechanical_idle_requested = 1U;
    s_mechanical_idle_ready = 0U;
    s_mechanical_idle_mask = 0U;
    s_mechanical_idle_settle_since_ms = 0U;
    memcpy(s_mechanical_idle_heartbeat_baseline, s_heartbeat_rx_count,
           sizeof(s_mechanical_idle_heartbeat_baseline));
    s_position_tx_enabled = 0U;
    s_position_tx_arm_pending_mask = 0U;
    s_position_tx_arm_started_ms = 0U;
    s_mit_debug_active = 0U;
    s_mit_fault_hold_active = 0U;
    s_mit_torque_test_active = 0U;
    s_mit_torque_test_index = DOG_MOTOR_COUNT;
    s_mit_torque_test_nm = 0.0f;
    s_jump_active = 0U;
    s_auto_stand_enabled = 0U;
    s_diag_support_active = 0U;
    s_last_command_tick_ms = 0U;
    s_mit_last_pid_ms = 0U;
    s_stand_state = DOG_STAND_IDLE;
    memset(&s_march, 0, sizeof(s_march));
    memset(s_leg_foot_x_offset, 0, sizeof(s_leg_foot_x_offset));
    memset(s_leg_hip_offset_deg, 0, sizeof(s_leg_hip_offset_deg));
    memset(s_motor_configured, 0, sizeof(s_motor_configured));
    memset(s_motor_loop_requested, 0, sizeof(s_motor_loop_requested));
    memset(s_motor_final_mode_pending, 0, sizeof(s_motor_final_mode_pending));
    memset(s_motor_mit_probe_active, 0, sizeof(s_motor_mit_probe_active));
    memset(s_position_idle_encoder_rx_baseline, 0, sizeof(s_position_idle_encoder_rx_baseline));
    memset(s_mit_boot_ok, 0, sizeof(s_mit_boot_ok));
    mit_clear_mixed_pid();
    teach_hold_stop();
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        mit_reset_motor_integrator(i);
    }
    (void)fdcan_abort_all_tx(&hfdcan1);
    (void)fdcan_abort_all_tx(&hfdcan2);
    motor_tx_guard_give();

    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        MWSetAxisState(g_dog_motor_config[i].bus,
                       g_dog_motor_config[i].node_id,
                       MW_AXIS_STATE_IDLE);
    }
    ArmMotor_Disable();
    return 1U;
}

uint8_t DogStand_IsMechanicalLimitIdle(void)
{
    return s_mechanical_idle_requested;
}

uint8_t DogStand_IsMechanicalLimitIdleReady(void)
{
    return ((s_mechanical_idle_requested != 0U) &&
            (s_mechanical_idle_ready != 0U)) ? 1U : 0U;
}

uint8_t DogStand_GetMechanicalLimitIdleMask(void)
{
    return s_mechanical_idle_mask;
}

void DogStand_ExitMechanicalLimitIdle(void)
{
    if (motor_tx_guard_take() == 0U) {
        return;
    }
    if (s_mechanical_idle_requested != 0U) {
        s_safety_generation++;
    }
    s_mechanical_idle_requested = 0U;
    s_mechanical_idle_ready = 0U;
    s_mechanical_idle_mask = 0U;
    s_mechanical_idle_settle_since_ms = 0U;
    motor_tx_guard_give();
}

static void mechanical_limit_idle_tick(uint32_t now)
{
    if (s_mechanical_idle_requested == 0U) {
        return;
    }

    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((s_heartbeat_rx_count[i] != s_mechanical_idle_heartbeat_baseline[i]) &&
            (motor_heartbeat_fresh(i, now) != 0U) &&
            (motor_has_fault(i) == 0U) &&
            (g_mw_motor_data[i].heartBeat.currentState == MW_AXIS_STATE_IDLE)) {
            mask |= (uint8_t)(1U << i);
        }
    }
    s_mechanical_idle_mask = mask;
    if (mask != 0xFFU) {
        s_mechanical_idle_ready = 0U;
        s_mechanical_idle_settle_since_ms = 0U;
        return;
    }
    if (s_mechanical_idle_settle_since_ms == 0U) {
        s_mechanical_idle_settle_since_ms = now;
        return;
    }
    if ((uint32_t)(now - s_mechanical_idle_settle_since_ms) >=
        DOG_MECHANICAL_IDLE_SETTLE_MS) {
        s_mechanical_idle_ready = 1U;
    }
}

static void DogStand_Estop(void)
{
    const uint32_t now = HAL_GetTick();
    if (motor_tx_guard_take() != 0U) {
        s_safety_generation++;
        s_mechanical_idle_requested = 0U;
        s_mechanical_idle_ready = 0U;
        s_mechanical_idle_mask = 0U;
        s_mechanical_idle_settle_since_ms = 0U;
        s_safety_latched = 1U;
        s_control_disabled = 1U;
        s_safety_rearm_requested = 0U;
        s_safety_latched_ms = now;
        s_safety_clear_started_ms = 0U;
        s_safety_clear_generation = 0U;
        s_safety_last_action_ms = now;
        s_estop_pending_mask = 0xFFU;
        (void)fdcan_abort_all_tx(&hfdcan1);
        (void)fdcan_abort_all_tx(&hfdcan2);
        service_estop_pending_locked();
        motor_tx_guard_give();
    } else {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_safety_generation++;
        s_mechanical_idle_requested = 0U;
        s_mechanical_idle_ready = 0U;
        s_mechanical_idle_mask = 0U;
        s_mechanical_idle_settle_since_ms = 0U;
        s_safety_latched = 1U;
        s_control_disabled = 1U;
        s_safety_rearm_requested = 0U;
        s_safety_latched_ms = now;
        s_safety_clear_started_ms = 0U;
        s_safety_clear_generation = 0U;
        s_safety_last_action_ms = now;
        s_estop_pending_mask = 0xFFU;
        __set_PRIMASK(primask);
    }

    dog_debug_mit_torque_stop();
    mit_debug_stop_tx();
    s_position_tx_arm_pending_mask = 0U;
    s_position_tx_arm_started_ms = 0U;
    memset(s_motor_configured, 0, sizeof(s_motor_configured));
    memset(s_motor_loop_requested, 0, sizeof(s_motor_loop_requested));
    memset(s_motor_final_mode_pending, 0, sizeof(s_motor_final_mode_pending));
    memset(s_motor_mit_probe_active, 0, sizeof(s_motor_mit_probe_active));
    ArmMotor_Disable();
    s_auto_stand_enabled = 0U;
    s_stand_state = DOG_STAND_ESTOP;
}

uint8_t DogSafety_IsLatched(void)
{
    return s_safety_latched;
}

void DogSafety_SetSdEstop(uint8_t active)
{
    const uint8_t inhibit = (active != 0U) ? 1U : 0U;
    uint8_t trigger_estop = 0U;
    if (motor_tx_guard_take() == 0U) {
        return;
    }
    if (inhibit != 0U) {
        if ((s_safety_external_inhibit == 0U) || (s_safety_latched == 0U)) {
            s_safety_external_inhibit = 1U;
            trigger_estop = 1U;
        }
    } else {
        s_safety_external_inhibit = 0U;
    }
    motor_tx_guard_give();
    if (trigger_estop != 0U) {
        DogStand_Estop();
    }
}

uint8_t DogSafety_RequestRearm(void)
{
    if (motor_tx_guard_take() == 0U) {
        return 0U;
    }
    if (s_safety_external_inhibit != 0U) {
        motor_tx_guard_give();
        return 0U;
    }
    if (s_safety_latched == 0U) {
        motor_tx_guard_give();
        return 1U;
    }

    if (s_safety_rearm_requested == 0U) {
        s_safety_rearm_requested = 1U;
        s_safety_clear_started_ms = 0U;
        s_safety_clear_generation = 0U;
    }
    motor_tx_guard_give();
    return 1U;
}

static void motor_safety_tick(uint32_t now)
{
    uint32_t generation = 0U;
    uint32_t clear_started_ms = 0U;
    uint8_t rearm_requested = 0U;
    uint8_t external_inhibit = 0U;

    if (motor_tx_guard_take() == 0U) {
        return;
    }
    if (s_safety_latched == 0U) {
        motor_tx_guard_give();
        return;
    }
    service_estop_pending_locked();
    generation = s_safety_generation;
    clear_started_ms = s_safety_clear_started_ms;
    rearm_requested = s_safety_rearm_requested;
    external_inhibit = s_safety_external_inhibit;
    motor_tx_guard_give();

    if ((rearm_requested == 0U) || (external_inhibit != 0U) ||
        (all_leg_motors_stopped_online() == 0U)) {
        uint8_t repeat_estop = 0U;
        if (motor_tx_guard_take() != 0U) {
            if ((s_safety_latched != 0U) && (s_safety_generation == generation)) {
                s_safety_clear_started_ms = 0U;
                s_safety_clear_generation = 0U;
                if ((uint32_t)(now - s_safety_last_action_ms) >= DOG_SAFETY_RETRY_PERIOD_MS) {
                    s_safety_last_action_ms = now;
                    s_estop_pending_mask = 0xFFU;
                    service_estop_pending_locked();
                    repeat_estop = 1U;
                }
            }
            motor_tx_guard_give();
        }
        if (repeat_estop != 0U) {
            ArmMotor_Disable();
        }
        return;
    }

    if (clear_started_ms == 0U) {
        if (motor_tx_guard_take() == 0U) {
            return;
        }
        if ((s_safety_latched == 0U) || (s_safety_generation != generation) ||
            (s_safety_rearm_requested == 0U) || (s_safety_external_inhibit != 0U)) {
            motor_tx_guard_give();
            send_all_estop();
            return;
        }
        if (all_leg_motors_stopped_online() == 0U) {
            motor_tx_guard_give();
            return;
        }
        s_safety_clear_started_ms = now;
        s_safety_clear_generation = generation;
        s_safety_last_action_ms = now;
        motor_tx_guard_give();
        if (send_all_clear_errors(generation) == 0U) {
            send_all_estop();
        }
        return;
    }

    uint8_t retry_clear = 0U;
    if (motor_tx_guard_take() != 0U) {
        if ((s_safety_latched != 0U) && (s_safety_generation == generation) &&
            (s_safety_clear_generation == generation) &&
            (s_safety_rearm_requested != 0U) && (s_safety_external_inhibit == 0U) &&
            ((uint32_t)(now - s_safety_last_action_ms) >= DOG_SAFETY_RETRY_PERIOD_MS)) {
            s_safety_last_action_ms = now;
            retry_clear = 1U;
        }
        motor_tx_guard_give();
    }
    if ((retry_clear != 0U) && (send_all_clear_errors(generation) == 0U)) {
        send_all_estop();
        return;
    }

    if (((uint32_t)(now - clear_started_ms) >= DOG_SAFETY_CLEAR_CONFIRM_MS) &&
        (all_leg_motors_clear_confirmed() != 0U)) {
        uint8_t cleared = 0U;
        if (motor_tx_guard_take() != 0U) {
            if ((s_safety_latched != 0U) &&
                (s_safety_generation == generation) &&
                (s_safety_clear_generation == generation) &&
                (s_safety_rearm_requested != 0U) &&
                (s_safety_external_inhibit == 0U) &&
                (s_estop_pending_mask == 0U) &&
                (all_leg_motors_clear_confirmed() != 0U)) {
                s_safety_latched = 0U;
                s_safety_rearm_requested = 0U;
                s_safety_latched_ms = 0U;
                s_safety_clear_started_ms = 0U;
                s_safety_clear_generation = 0U;
                s_stand_state = DOG_STAND_IDLE;
                cleared = 1U;
            }
            motor_tx_guard_give();
        }
        if (cleared != 0U) {
            DebugUart_Printf("Safety latch cleared: all 8 leg motors idle and fault-free.\r\n");
        } else {
            send_all_estop();
        }
    }
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

uint8_t DogStand_GetFaultMask(void)
{
    uint8_t mask = 0U;
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if (motor_has_fault(i) != 0U) mask |= (uint8_t)(1U << i);
    }
    return mask;
}

static uint8_t next_online_motor_on_bus(uint8_t bus, uint8_t *cursor)
{
    if (cursor == nullptr) {
        return DOG_MOTOR_COUNT;
    }

    const uint8_t start = (uint8_t)(*cursor % DOG_MOTOR_COUNT);
    for (uint8_t offset = 0U; offset < DOG_MOTOR_COUNT; ++offset) {
        const uint8_t index = (uint8_t)((start + offset) % DOG_MOTOR_COUNT);
        if ((g_dog_motor_config[index].bus == bus) && (s_motor_online[index] != 0U)) {
            *cursor = (uint8_t)((index + 1U) % DOG_MOTOR_COUNT);
            return index;
        }
    }
    return DOG_MOTOR_COUNT;
}

static uint8_t motor_feedback_health_tick(uint32_t now)
{
    for (uint8_t i = 0U; i < DOG_MOTOR_COUNT; ++i) {
        if ((s_motor_online[i] != 0U) &&
            ((uint32_t)(now - s_last_rx_tick_ms[i]) > DOG_RX_TIMEOUT_MS)) {
            s_motor_online[i] = 0U;
            s_encoder_est_fresh[i] = 0U;
        }
        if ((s_encoder_est_fresh[i] != 0U) &&
            ((uint32_t)(now - s_last_encoder_tick_ms[i]) > DOG_ENCODER_FEEDBACK_TIMEOUT_MS)) {
            s_encoder_est_fresh[i] = 0U;
        }
        if (motor_closed_loop(i) == 0U) {
            s_encoder_est_fresh[i] = 0U;
        }
        if ((s_motor_loop_requested[i] != 0U) && (motor_has_fault(i) != 0U)) {
            s_motor_loop_requested[i] = 0U;
        }

        const uint8_t heartbeat_fresh = ((s_last_heartbeat_tick_ms[i] != 0U) &&
            ((uint32_t)(now - s_last_heartbeat_tick_ms[i]) <= DOG_HEARTBEAT_TIMEOUT_MS)) ? 1U : 0U;
        uint8_t controlled = 0U;
        if ((((s_mit_debug_active != 0U) || (s_mit_fault_hold_active != 0U)) &&
             (s_mit_boot_ok[i] != 0U)) ||
            ((s_position_tx_enabled != 0U) &&
             ((s_auto_stand_enabled != 0U) || (motor_is_selected(i) != 0U))) ||
            ((s_mit_torque_test_active != 0U) && (s_mit_torque_test_index == i))) {
            controlled = 1U;
        }
        if ((controlled != 0U) &&
            ((s_motor_online[i] == 0U) || (heartbeat_fresh == 0U) ||
             (s_encoder_est_fresh[i] == 0U) || (motor_closed_loop(i) == 0U) ||
             (motor_has_fault(i) != 0U))) {
            mit_debug_abort_control("leg motor feedback/fault");
            return 0U;
        }
    }
    return 1U;
}

static void encoder_feedback_query_tick(uint32_t now)
{
    if ((uint32_t)(now - s_encoder_query_last_ms) < DOG_ENCODER_FEEDBACK_PERIOD_MS) {
        return;
    }

    s_encoder_query_last_ms = now;
    const uint8_t buses[2U] = {DOG_CAN_FRONT_BUS, DOG_CAN_REAR_BUS};
    for (uint8_t di = 0U; di < 2U; ++di) {
        if (mit_probe_bus_tx_busy(buses[di]) != 0U) {
            continue;
        }
        const uint8_t index = next_online_motor_on_bus(buses[di], &s_encoder_query_cursor[di]);
        if (index < DOG_MOTOR_COUNT) {
            mw_query_encoder_estimate(index);
        }
    }
}

void motor_task_init(void)
{
    if (s_motor_tx_guard == nullptr) {
        s_motor_tx_guard = xSemaphoreCreateMutexStatic(&s_motor_tx_guard_storage);
    }
    if (s_motor_tx_guard == nullptr) {
        s_safety_latched = 1U;
        DebugUart_Printf("Motor TX guard init failed: control remains safety-latched.\r\n");
        return;
    }

    bsp_can_init(&hfdcan1, CAN1_Callback);
    bsp_can_init(&hfdcan2, CAN2_Callback);
    bsp_can_init(&hfdcan3, CAN3_Callback);
    WheelDrive_Init(&hfdcan3);
    ArmMotor_Init(&hfdcan1,
                  ARM_J0_DM_CAN_ID,
                  ARM_J0_DM_FEEDBACK_ID,
                  &hfdcan2,
                  ARM_J1_EL05_CAN_ID,
                  ARM_J1_EL05_INIT_MODEL_IGNORED);

    memset(g_mw_motor_data, 0, sizeof(g_mw_motor_data));
    memset(s_last_rx_tick_ms, 0, sizeof(s_last_rx_tick_ms));
    memset(s_motor_online, 0, sizeof(s_motor_online));
    memset(s_motor_configured, 0, sizeof(s_motor_configured));
    memset(s_motor_loop_requested, 0, sizeof(s_motor_loop_requested));
    memset(s_motor_final_mode_pending, 0, sizeof(s_motor_final_mode_pending));
    memset(s_position_idle_encoder_rx_baseline, 0, sizeof(s_position_idle_encoder_rx_baseline));
    memset(s_last_encoder_tick_ms, 0, sizeof(s_last_encoder_tick_ms));
    memset(s_last_heartbeat_tick_ms, 0, sizeof(s_last_heartbeat_tick_ms));
    memset(s_heartbeat_rx_count, 0, sizeof(s_heartbeat_rx_count));
    memset(s_motor_can_id_type, 0, sizeof(s_motor_can_id_type));
    memset(s_encoder_query_cursor, 0, sizeof(s_encoder_query_cursor));
    memset(s_slow_query_cursor, 0, sizeof(s_slow_query_cursor));
    memset(s_slow_query_kind, 0, sizeof(s_slow_query_kind));
    memset(s_mit_probe_keepalive_cursor, 0, sizeof(s_mit_probe_keepalive_cursor));
    s_encoder_query_last_ms = 0U;
    s_mit_probe_tx_busy_mask = 0U;
    s_position_tx_arm_pending_mask = 0U;
    s_position_tx_arm_started_ms = 0U;
    s_can_tx_drop_count[0] = 0U;
    s_can_tx_drop_count[1] = 0U;
    memset(s_encoder_est_fresh, 0, sizeof(s_encoder_est_fresh));
    memset(s_encoder_rx_count, 0, sizeof(s_encoder_rx_count));
    memset(s_motor_mit_probe_active, 0, sizeof(s_motor_mit_probe_active));
    memset(s_mit_boot_ok, 0, sizeof(s_mit_boot_ok));
    memset(s_encoder_turn_valid, 0, sizeof(s_encoder_turn_valid));
    s_safety_latched = 0U;
    s_safety_external_inhibit = 0U;
    s_control_disabled = 0U;
    s_mechanical_idle_requested = 0U;
    s_mechanical_idle_ready = 0U;
    s_mechanical_idle_mask = 0U;
    s_mechanical_idle_settle_since_ms = 0U;
    s_mechanical_pose_requested = 0U;
    s_mechanical_pose_ready = 0U;
    s_mechanical_pose_mask = 0U;
    memset(s_mechanical_idle_heartbeat_baseline, 0,
           sizeof(s_mechanical_idle_heartbeat_baseline));
    s_safety_rearm_requested = 0U;
    s_safety_generation = 0U;
    s_safety_latched_ms = 0U;
    s_safety_clear_started_ms = 0U;
    s_safety_clear_generation = 0U;
    s_safety_last_action_ms = 0U;
    s_estop_pending_mask = 0U;
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
    if ((system_can[0] == false) || (system_can[1] == false)) {
        DebugUart_Printf("FDCAN init failed: all motors disabled.\r\n");
        DogStand_Disable();
        return;
    }
    if (system_can[2] == false) {
        DebugUart_Printf("FDCAN3 init failed: wheel drive locked; leg/arm diagnostics remain available.\r\n");
    }
    DebugUart_Printf("quadruped SDK debug: CAN1 front+J0_DM=0x01/0x10 CAN2 rear+J1_EL05=0x7F disabled\r\n");
}

void motor_task(void)
{
    fdcan_poll_rx(&hfdcan1);
    fdcan_poll_rx(&hfdcan2);
    fdcan_poll_rx(&hfdcan3);
    const uint32_t now = HAL_GetTick();

    static uint32_t s_can_recovery_last_ms = 0U;
    if ((uint32_t)(now - s_can_recovery_last_ms) >= DOG_CAN_RECOVERY_PERIOD_MS) {
        s_can_recovery_last_ms = now;
        restart_bus_off(&hfdcan1);
        restart_bus_off(&hfdcan2);
    }

    (void)motor_feedback_health_tick(now);

    mechanical_limit_idle_tick(now);

    motor_safety_tick(now);

    if (s_mechanical_idle_requested == 0U) {
        stand_state_tick(now);
        dog_mit_lower_to_start_pose_tick(now);
        if (s_lower_state == DOG_LOWER_IDLE) {
            march_in_place_tick(now);
        }
    }

    static uint32_t s_closed_loop_retry_last_ms = 0U;
    static uint8_t s_closed_loop_retry_cursor = 0U;
    if ((s_safety_latched == 0U) && (s_mechanical_idle_requested == 0U) &&
        ((uint32_t)(now - s_closed_loop_retry_last_ms) >= DOG_CLOSED_LOOP_RETRY_SLOT_MS)) {
        s_closed_loop_retry_last_ms = now;
        const uint8_t index = s_closed_loop_retry_cursor;
        s_closed_loop_retry_cursor = (uint8_t)((s_closed_loop_retry_cursor + 1U) % DOG_MOTOR_COUNT);
        if ((s_motor_loop_requested[index] != 0U) &&
            (mit_probe_bus_tx_busy(g_dog_motor_config[index].bus) == 0U) &&
            (motor_heartbeat_fresh(index, now) != 0U) &&
            (motor_has_fault(index) == 0U) &&
            ((s_motor_final_mode_pending[index] != DOG_FINAL_MODE_NONE) ||
             (s_motor_configured[index] == 0U) ||
             (motor_closed_loop(index) == 0U) ||
             ((s_motor_mit_probe_active[index] != 0U) &&
              (motor_encoder_fresh(index, now) != 0U)))) {
            (void)request_closed_loop(index);
        }
    }

    if (s_mechanical_idle_requested == 0U) {
        position_finalization_fast_tick();
        position_tx_arm_tick(now);
    }

    if (s_mechanical_idle_requested == 0U) {
        send_mit_probe_keepalive(now);
        send_mit_torque_test_keepalive(now);
    }

    if (s_position_tx_enabled != 0U) {
        send_enabled_targets(now);
    }

    if (s_mechanical_idle_requested == 0U) {
        encoder_feedback_query_tick(now);
    }

    static uint32_t s_arm_cmd_last_ms = 0U;
    if ((s_safety_latched == 0U) && (s_mechanical_idle_requested == 0U) &&
        (mit_probe_bus_tx_busy(DOG_CAN_FRONT_BUS) == 0U) &&
        (mit_probe_bus_tx_busy(DOG_CAN_REAR_BUS) == 0U) &&
        ((uint32_t)(now - s_arm_cmd_last_ms) >= ARM_CMD_PERIOD_MS) &&
        (fdcan_tx_free_level(&hfdcan1) >= 1U) &&
        (fdcan_tx_free_level(&hfdcan2) >= 1U)) {
        s_arm_cmd_last_ms = now;
        ArmMotor_Send();
    }

    uint8_t arm_diagnostic_tx_mask = 0U;
    if ((system_can[0] != false) &&
        (mit_probe_bus_tx_busy(DOG_CAN_FRONT_BUS) == 0U) &&
        (fdcan_tx_free_level(&hfdcan1) >= 1U)) {
        arm_diagnostic_tx_mask |= ARM_J0_DM4310_MASK;
    }
    if ((system_can[1] != false) &&
        (mit_probe_bus_tx_busy(DOG_CAN_REAR_BUS) == 0U) &&
        (fdcan_tx_free_level(&hfdcan2) >= 1U)) {
        arm_diagnostic_tx_mask |= ARM_J1_LZ_MASK;
    }
    ArmMotor_DiagnosticPoll(now, arm_diagnostic_tx_mask);

    static uint32_t s_slow_feedback_last_ms = 0U;
    if ((s_mechanical_idle_requested == 0U) &&
        ((uint32_t)(now - s_slow_feedback_last_ms) >= DOG_SLOW_FEEDBACK_SLOT_MS)) {
        s_slow_feedback_last_ms = now;
        const uint8_t buses[2U] = {DOG_CAN_FRONT_BUS, DOG_CAN_REAR_BUS};
        for (uint8_t di = 0U; di < 2U; ++di) {
            if (mit_probe_bus_tx_busy(buses[di]) != 0U) {
                continue;
            }
            const uint8_t index = next_online_motor_on_bus(buses[di], &s_slow_query_cursor[di]);
            if (index >= DOG_MOTOR_COUNT) {
                continue;
            }
            FDCAN_HandleTypeDef *bus = bus_handle(buses[di]);
            if ((bus == nullptr) || (fdcan_tx_free_level(bus) == 0U)) {
                continue;
            }
            if (s_slow_query_kind[index] == 0U) {
                MWGetIq(g_dog_motor_config[index].bus, g_dog_motor_config[index].node_id);
            } else {
                MWGetBusVoltageCurrent(g_dog_motor_config[index].bus, g_dog_motor_config[index].node_id);
            }
            s_slow_query_kind[index] ^= 1U;
        }
    }

    DebugUart_Process();
}

extern "C" void motor_task_tick(void)
{
    motor_task();
}

extern "C" void motor_can_rx_tick(void)
{
    fdcan_poll_rx(&hfdcan1);
    fdcan_poll_rx(&hfdcan2);
    fdcan_poll_rx(&hfdcan3);
}
