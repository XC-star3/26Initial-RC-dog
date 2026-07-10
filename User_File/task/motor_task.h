#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DOG_CAN_FRONT_BUS       1U
#define DOG_CAN_REAR_BUS        2U
#define DOG_MOTOR_COUNT         8U
#define DOG_LEG_COUNT           4U
#define DOG_MOTORS_PER_LEG      2U

#define DOG_GAIT_SPEED_LOW      0U
#define DOG_GAIT_SPEED_MID      1U
#define DOG_GAIT_SPEED_HIGH     2U
#define DOG_GAIT_SPEED_DEFAULT  DOG_GAIT_SPEED_MID

#define DOG_LEG_LF              0U
#define DOG_LEG_RF              1U
#define DOG_LEG_LB              2U
#define DOG_LEG_RB              3U

#define DOG_JOINT_HIP           0U
#define DOG_JOINT_KNEE          1U

#define DOG_MIT_PROBE_CURRENT_LIMIT_A 0.35f

#define DOG_MIT_DEBUG_MAX_ERR_DEG       720.0f
#define DOG_MIT_DEBUG_KP_A_PER_DEG      0.08f
#define DOG_MIT_DEBUG_SETTLE_MS         300U
#define DOG_MIT_DEBUG_SAFETY_ERR_DEG    720.0f

#define DOG_STAND_POSE_HIP_MOTOR_DEG    12.0f
#define DOG_STAND_POSE_KNEE_MOTOR_DEG   58.0f
#define DOG_STAND_HIP_MOVE_MS           1500U
#define DOG_STAND_KNEE_MOVE_MS          1500U
#define DOG_STAND_HIP_SETTLE_ERR_DEG    5.0f
#define DOG_STAND_KNEE_SETTLE_ERR_DEG   2.0f
#define DOG_STAND_JOINT_SETTLE_ERR_DEG  DOG_STAND_HIP_SETTLE_ERR_DEG

#define DOG_REAR_FOOT_EXTRA_Z_MM        15.0f    /* LB/RB touch-down Z above front legs */
#define DOG_STAND_FOOT_X_MM             0.0f
#define DOG_STAND_FOOT_Z_START_MM       150.0f   /* front legs stand rise IK start Z */
#define DOG_STAND_FOOT_Z_MM             300.0f   /* front legs stand rise IK end Z */
#define DOG_STAND_RISE_MS               1500U
#define DOG_JUMP_FOOT_X_MM              DOG_STAND_FOOT_X_MM
#define DOG_JUMP_APEX_Z_MM              400.0f
#define DOG_JUMP_LAND_Z_MM              DOG_STAND_FOOT_Z_MM
#define DOG_JUMP_APEX_HOLD_MS           80U
#define DOG_JUMP_LAND_MS                150U
#define DOG_JUMP_FF_MAX_NM              2.5f
#define DOG_JUMP_FF_ERR_DEADBAND_DEG    1.5f
#define DOG_FOOT_TARGET_X_MM            160.0f
#define DOG_FOOT_TARGET_Z_MM            220.0f
#define DOG_FOOT_GOTO_LEG               DOG_LEG_LF

/* Two-point foot IK calib: motor (hip,knee) deg <-> foot (X,Z) mm, L1/L2 in motor_task.cpp */
#define DOG_IK_CAL_A_MOTOR_HIP_DEG        (-33.0f)
#define DOG_IK_CAL_A_MOTOR_KNEE_DEG       33.0f
#define DOG_IK_CAL_A_FOOT_X_MM            0.0f
#define DOG_IK_CAL_A_FOOT_Z_MM            135.6f
#define DOG_IK_CAL_B_MOTOR_HIP_DEG        0.0f
#define DOG_IK_CAL_B_MOTOR_KNEE_DEG       0.0f
#define DOG_IK_CAL_B_FOOT_X_MM            80.0f
#define DOG_IK_CAL_B_FOOT_Z_MM            110.0f

#define DOG_MARCH_LIFT_Z_MM             60.0f    /* positive clearance, subtracted from +Z-down touch-down */
#define DOG_DIAG_SUPPORT_LIFT_Z_MM      DOG_MARCH_LIFT_Z_MM
#define DOG_TROT_FOOT_X1_MM             0.0f
#define DOG_TROT_FOOT_Z1_MM             0.0f     /* relative to per-leg stand touch-down Z */
#define DOG_TROT_FORWARD_X_SIGN          1.0f    /* physical forward calibration for trot */
#define DOG_TURN_X_SIGN                 (-1.0f)  /* independent turn calibration; preserves left/right */
#define DOG_TROT_YAW_TRIM_X_MM           0.0f
#define DOG_FORWARD_HIP_STEP_DEG        5.0f
#define DOG_FORWARD_SWING_HIP_DEG       8.0f
#define DOG_FORWARD_SWING_KNEE_DEG      (-20.0f)
#define DOG_MARCH_HOLD_MS               250U
#define DOG_MARCH_LEG_PAUSE_MS          150U
#define DOG_MARCH_SETTLE_TIMEOUT_MS     2500U
#define DOG_MARCH_SETTLE_ERR_DEG        DOG_STAND_KNEE_SETTLE_ERR_DEG

#define DOG_TROT_HZ                     2.0f
#define DOG_TROT_CYCLE_MS               (1000U / DOG_TROT_HZ)
#define DOG_TROT_PAIR_MS                (DOG_TROT_CYCLE_MS / 2U)
#define DOG_TROT_SWING_MS               DOG_TROT_PAIR_MS   /* cycloid swing = half cycle @ 2Hz */
#define DOG_TROT_SWING_UP_MS            35U                /* walk mode only */
#define DOG_TROT_HOLD_MS                15U
#define DOG_TROT_SWING_DOWN_MS          35U
#define DOG_TROT_SETTLE_ERR_DEG         4.0f
#define DOG_GAIT_ENTRY_ERR_DEG           5.0f
#define DOG_GAIT_ENTRY_VEL_DPS          10.0f
#define DOG_GAIT_ENTRY_STABLE_MS        150U
#define DOG_GAIT_ENTRY_TIMEOUT_MS      1500U
#define DOG_GAIT_LOW_ENTRY_ERR_DEG       4.0f
#define DOG_GAIT_LOW_ENTRY_VEL_DPS       8.0f
#define DOG_GAIT_LOW_ENTRY_STABLE_MS    200U
#define DOG_GAIT_LOW_ENTRY_TIMEOUT_MS  2000U
#define DOG_TROT_TOUCHDOWN_VEL_DPS      10.0f
#define DOG_TROT_TOUCHDOWN_IQ_RISE_A     1.5f
#define DOG_TROT_TOUCHDOWN_IQ_ALPHA      0.10f
#define DOG_TROT_TOUCHDOWN_IQ_START      0.80f
#define DOG_TROT_TOUCHDOWN_FALLBACK_MS   40U
#define DOG_TROT_TOUCHDOWN_TIMEOUT_MS   200U
#define DOG_GAIT_LOW_TOUCHDOWN_ERR_DEG    4.0f
#define DOG_GAIT_LOW_TOUCHDOWN_VEL_DPS    8.0f
#define DOG_GAIT_LOW_TOUCHDOWN_STABLE_MS 80U
#define DOG_GAIT_LOW_TOUCHDOWN_TIMEOUT_MS 500U
#define DOG_GAIT_LOW_SUPPORT_CURRENT_A   16.0f

#define DOG_TURN_STRIDE_X_MM             30.0f    /* in-place turn step per leg side */
#define DOG_TURN_HZ                       DOG_TROT_HZ
#define DOG_TURN_SWING_MS                 DOG_TROT_SWING_MS

enum Dog_March_Mode {
    DOG_MARCH_MODE_WALK = 0U,
    DOG_MARCH_MODE_TROT = 1U,
    DOG_MARCH_MODE_TURN_LEFT = 2U,
    DOG_MARCH_MODE_TURN_RIGHT = 3U,
};

enum Dog_Leg_Kin_Mode {
    DOG_LEG_KIN_LINEAR = 0U,
    DOG_LEG_KIN_CLOSED_CHAIN = 1U,
};

struct Dog_Leg_Kin_Params {
    float crank_mm;
    float thigh_mm;
    float shank_mm;
    float para_mm;
    float knee_offset_h_mm;
    float knee_offset_d_mm;
    float thigh_scale;
    float thigh_offset_deg;
    float shank_scale;
    float shank_offset_deg;
    uint8_t mode;
};

enum Dog_Debug_Target {
    DOG_DEBUG_TARGET_SINGLE = 0U,
    DOG_DEBUG_TARGET_LEG = 1U,
    DOG_DEBUG_TARGET_FRONT_PAIR = 2U,
    DOG_DEBUG_TARGET_REAR_PAIR = 3U,
    DOG_DEBUG_TARGET_ALL = 4U,
    DOG_DEBUG_TARGET_SINGLE_KNEE = 5U,
};

enum Dog_Stand_State {
    DOG_STAND_IDLE = 0U,
    DOG_STAND_WAIT_HEARTBEAT,
    DOG_STAND_CONFIGURE,
    DOG_STAND_CLOSED_LOOP,
    DOG_STAND_MOVING,
    DOG_STAND_STANDING,
    DOG_STAND_ESTOP,
    DOG_STAND_FAULT,
};

enum Dog_Control_Loop_Mode {
    DOG_CTRL_LOOP_POSITION = 0U,
    DOG_CTRL_LOOP_MIT_PID = 1U,
};

enum Dog_Mit_Pid_Profile {
    DOG_MIT_PID_STAND = 0U,
    DOG_MIT_PID_SWING = 1U,
};

struct Dog_Mit_Ang_Pid {
    float kp_a_per_deg;
    float ki_a_per_deg_s;
    float kd_a_per_dps;
    float output_limit_a;
};

struct Dog_Mit_Motor_Limits {
    float current_limit_a;
    float vel_limit_turn_s;
    float torque_nm_per_a;
};

struct Dog_Mit_Pid_Telemetry {
    float target_deg;
    float user_deg;
    float err_deg;
    float cmd_a;
    float p_a;
    float i_a;
    float d_a;
};

struct Dog_Motor_Seen {
    uint8_t bus;
    uint16_t motor_id;
};

struct Dog_Can_Diag {
    uint8_t bus_off;
    uint8_t error_passive;
    uint8_t warning;
    uint8_t activity;
    uint8_t last_error_code;
    uint8_t data_last_error_code;
    uint8_t tx_error_count;
    uint8_t rx_error_count;
    uint8_t rx_error_passive;
    uint8_t error_logging;
    uint8_t rx_fifo_fill;
    uint8_t parsed_node_mask;
    uint16_t last_parsed_id;
    uint8_t last_parsed_node;
    uint8_t last_parsed_cmd;
    uint32_t rx_frame_count;
    uint32_t parsed_frame_count;
    uint16_t last_rx_id;
    uint8_t last_rx_len;
    uint8_t last_rx_ext;
    uint32_t rx_reject_format;
    uint32_t rx_reject_node;
    uint32_t rx_reject_nodata;
    uint32_t tx_drop_count;
};

struct Dog_Motor_Config {
    uint8_t leg;
    uint8_t joint;
    uint8_t bus;
    uint8_t node_id;
    float direction;
    float torque_direction;
    float zero_offset_deg;
    float encoder_gear_ratio;
    float min_deg;
    float max_deg;
};

struct Dog_Imu_Sample {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gyro_dps[3];
    float accel_mps2[3];
    uint32_t tick_ms;
};

struct Dog_Remote_Sample {
    uint8_t stand_request;
    uint8_t estop_request;
    uint8_t mode;
    uint32_t tick_ms;
};

#ifdef __cplusplus
}
#endif

#include "MWMotor.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t g_mw_node_ids[DOG_MOTOR_COUNT];
extern MW_MOTOR_DATA g_mw_motor_data[DOG_MOTOR_COUNT];
extern Dog_Motor_Config g_dog_motor_config[DOG_MOTOR_COUNT];
extern Dog_Leg_Kin_Params g_dog_leg_kin_params[DOG_LEG_COUNT];
extern Dog_Mit_Ang_Pid g_dog_mit_stand_pid;
extern Dog_Mit_Ang_Pid g_dog_mit_swing_pid;
extern Dog_Mit_Motor_Limits g_dog_mit_motor_limits;

const char *dog_leg_name(uint8_t leg);
const char *dog_control_loop_mode_name(void);
const char *dog_debug_target_name(void);
uint8_t dog_leg_target_leg(void);
uint8_t dog_debug_target_count(void);
uint8_t dog_leg_target_expected_mask(void);
uint8_t dog_debug_target(void);

void dog_debug_set_target(uint8_t target);
void dog_debug_set_target_leg(uint8_t leg);
void dog_debug_rx_only(void);
void dog_debug_clear_errors(void);
void dog_debug_position_setup(void);
void dog_debug_mit_setup(void);
void dog_debug_enter_closed_loop(void);
uint8_t dog_debug_mit_boot_sequence(void);
uint8_t dog_debug_teach_hold_start(void);
uint8_t dog_mit_debug_is_active(void);
uint8_t dog_mit_fault_hold_is_active(void);
uint8_t dog_debug_mit_torque_test(uint8_t bus, uint8_t node_id, float torque_nm);
void dog_debug_mit_torque_stop(void);
void dog_debug_start_position_tx(void);
void dog_debug_idle(void);

void dog_mit_reset_integrators(void);
void dog_mit_clamp_integrators(void);
void dog_mit_send_control_now(void);
float dog_mit_get_cmd_current_a(uint8_t motor_index);
uint8_t dog_mit_get_pid_profile(void);
const char *dog_mit_pid_profile_name(void);
void dog_mit_set_pid_profile(uint8_t profile);
const Dog_Mit_Ang_Pid *dog_mit_active_ang_pid(void);
uint8_t dog_mit_goto_motor_pose(float hip_motor_deg, float knee_motor_deg);
uint8_t dog_mit_goto_stand_pose(void);
uint8_t dog_mit_stand_sequence(void);
uint8_t dog_mit_return_to_stand_start_pose(void);
uint8_t dog_mit_jump_test_sequence(void);
uint8_t dog_mit_goto_foot_xz(float x_mm, float z_mm);
uint8_t dog_mit_march_in_place_start(uint8_t cycles);
uint8_t dog_mit_trot_in_place_start(uint8_t cycles);
uint8_t dog_mit_trot_reverse_in_place_start(uint8_t cycles);
uint8_t dog_mit_turn_left_in_place_start(uint8_t cycles);
uint8_t dog_mit_turn_right_in_place_start(uint8_t cycles);
uint8_t dog_mit_drive_start(uint8_t cycles, float forward, float yaw);
uint8_t dog_mit_drive_update(float forward, float yaw);
void dog_mit_drive_get_command(float *requested_forward, float *requested_yaw,
                               float *applied_forward, float *applied_yaw);
void dog_mit_march_in_place_stop(void);
void dog_mit_march_request_stop(void);
uint8_t dog_mit_march_in_place_is_active(void);
uint8_t dog_mit_march_in_place_is_stopping(void);
uint8_t dog_mit_trot_march_is_active(void);
uint8_t dog_mit_turn_march_is_active(void);
void dog_mit_set_gait_speed_profile(uint8_t profile);
uint8_t dog_mit_get_gait_speed_profile(void);
const char *dog_mit_gait_speed_profile_name(void);
float dog_mit_gait_trot_hz(void);
float dog_mit_gait_forward_stride_x_mm(void);
float dog_mit_gait_turn_stride_x_mm(void);
float dog_mit_gait_swing_height_mm(void);
uint32_t dog_mit_gait_touchdown_dwell_ms(void);
uint32_t dog_mit_gait_diagonal_stagger_ms(void);
uint32_t dog_mit_gait_trot_swing_ms(void);
uint8_t dog_mit_diag_support_lf_rb_start(void);
void dog_mit_diag_support_stop(void);
uint8_t dog_mit_diag_support_is_active(void);
void dog_diag_support_print_status(void);
void dog_mit_print_all_motor_current(const char *tag);
uint8_t dog_leg_foot_xz_is_reachable(float x_mm, float z_mm);
uint8_t dog_leg_foot_xz_to_motor_deg(uint8_t leg, float x_mm, float z_mm,
                                     float *hip_motor_deg, float *knee_motor_deg);
uint8_t dog_mit_get_pid_telemetry(uint8_t motor_index, Dog_Mit_Pid_Telemetry *telemetry);
uint8_t dog_mit_get_default_vofa_motor_index(void);
uint8_t dog_control_get_loop_mode(void);

void dog_leg_set_target_leg_deg(float hip_deg, float knee_deg);
void dog_leg_set_target_motor_user_deg(float hip_motor_deg, float knee_motor_deg);
void dog_leg_get_target_leg_angles(float *hip_deg, float *knee_deg);
void dog_leg_get_target_leg_cmd_deg(float *hip_deg, float *knee_deg);
void dog_leg_get_target_encoder_fresh(uint8_t *hip_fresh, uint8_t *knee_fresh);
void dog_leg_get_target_leg_raw_angles(float *hip_deg, float *knee_deg);
void dog_leg_get_target_zero_offsets(float *hip_deg, float *knee_deg);
/* hip/knee motor user_deg -> foot (x,z) mm in leg IK frame (+Z down) */
uint8_t dog_leg_forward_kinematics(float hip_motor_deg, float knee_motor_deg,
                                   float *x_mm, float *z_mm);
uint8_t dog_leg_motor_to_geom(uint8_t leg, float q_thigh_motor_deg, float q_shank_motor_deg,
                              float *thigh_geom_deg, float *knee_geom_deg);
uint8_t dog_leg_geom_to_motor(uint8_t leg, float thigh_geom_deg, float knee_geom_deg,
                              float *q_thigh_motor_deg, float *q_shank_motor_deg);
uint8_t dog_leg_read_geom_from_motors(uint8_t leg, float *thigh_geom_deg, float *knee_geom_deg);
void dog_leg_set_kin_mode(uint8_t leg, uint8_t mode);
void dog_leg_set_kin_linear_calib(uint8_t leg, float thigh_scale, float thigh_offset_deg,
                                  float shank_scale, float shank_offset_deg);
uint8_t dog_leg_set_target_zero_current(void);
uint8_t dog_leg_target_motor_mode_mask(void);
uint8_t dog_leg_target_online_mask(void);
uint8_t dog_leg_target_ready_mask(void);
uint8_t dog_leg_target_online_ids(uint16_t *ids, uint8_t max_count);
uint8_t dog_leg_seen_online_ids(Dog_Motor_Seen *seen, uint8_t max_count);
void dog_leg_dump_target_status(void);
void dog_can1_get_diag(Dog_Can_Diag *diag);
void dog_can2_get_diag(Dog_Can_Diag *diag);
void dog_leg_print_angle_status(const char *tag);
void dog_leg_print_angle_status_for_leg(const char *tag, uint8_t leg);
uint8_t dog_leg_foot_xz_from_motor_deg(uint8_t leg, float hip_motor_deg, float knee_motor_deg,
                                       float *x_mm, float *z_mm);
uint8_t dog_leg_foot_x_stand_ref_mm(uint8_t leg, float *stand_x_mm);
void dog_lf_print_periodic_status(void);
uint8_t dog_motor_encoder_fresh(uint8_t motor_index);
void dog_motor_poll_can(void);
void dog_motor_query_encoder(uint8_t motor_index);
void dog_motor_query_online_encoders(void);

void DogImu_Update(const Dog_Imu_Sample *sample);
void DogRemote_Update(const Dog_Remote_Sample *sample);
void DogStand_Request(void);
void DogStand_Disable(void);
uint8_t DogStand_ClearDisable(void);
uint8_t DogStand_IsDisabled(void);
Dog_Stand_State DogStand_GetState(void);
uint8_t DogStand_GetOnlineMask(void);
uint8_t DogStand_GetReadyMask(void);
uint8_t DogSafety_IsLatched(void);
void DogSafety_SetSdEstop(uint8_t active);
uint8_t DogSafety_RequestRearm(void);

void motor_task_init(void);
void motor_task(void);

#ifdef __cplusplus
}
#endif

#endif
