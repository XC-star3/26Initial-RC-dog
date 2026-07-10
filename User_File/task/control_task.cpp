#include "control_task.h"

#include "arm_motor_task.h"
#include "debug_uart.h"
#include "motor_task.h"
#include "sbus.h"
#include "tim.h"
#include "vofa_pid.h"

#include "cmsis_os2.h"

#include <math.h>

#define DEBUG_LF_STATUS_MS 1000U
#define STEP_DEG           30.0f

#define SBUS_REMOTE_TIMEOUT_MS 1000U
#define SBUS_SPEED_CH          2U
#define SBUS_MAIN_MODE_CH      4U
#define SBUS_SUB_MODE_CH       7U
#define SBUS_ARM_J0_CH         5U
#define SBUS_ARM_J1_CH         6U
#define SBUS_ARM_UPDATE_MS     20U
#define SBUS_ARM_RETRY_MS      500U
#define SBUS_ARM_RATE_DEG_S    30.0f
#define SBUS_ARM_J0_MIN_DEG    (-60.0f)
#define SBUS_ARM_J0_MAX_DEG    90.0f
#define SBUS_ARM_J1_MIN_DEG    (-90.0f)
#define SBUS_ARM_J1_MAX_DEG    90.0f
#define SBUS_MOVE_ENTER_DEADBAND 25
#define SBUS_MOVE_EXIT_DEADBAND  12
#define SBUS_ARM_JOG_DEADBAND    8
#define SBUS_SPEED_LOW_NORM    (-60)
#define SBUS_SPEED_HIGH_NORM   60
#define SBUS_SAFETY_CH         8U
#define SBUS_SAFETY_HIGH_NORM  50
#define SBUS_SAFETY_RELEASE_NORM 20
#define SBUS_SAFETY_RECOVERY_MS 150U
#define SBUS_GAIT_RETRY_MS     500U

#define SBUS_SAFETY_RELEASED   0U
#define SBUS_SAFETY_INHIBIT    1U
#define SBUS_SAFETY_TRIGGER    2U

enum ControlMode {
    MODE_RX_ONLY = 0U,
    MODE_HOLD = 1U,
    MODE_IDLE = 3U,
    MODE_MIT_DEBUG = 4U,
};

enum SbusQuadCmd {
    SBUS_QUAD_STOP = 0U,
    SBUS_QUAD_DRIVE,
};

struct SbusDriveInput {
    float forward;
    float yaw;
    uint8_t active;
};

static ControlMode s_mode = MODE_RX_ONLY;
static uint8_t s_can_rx_log_enabled = 0U;
static uint32_t s_last_lf_status_ms = 0U;
static uint8_t s_sbus_seen_fresh = 0U;
static uint8_t s_sbus_switch_valid = 0U;
static uint8_t s_sbus_main_prev = SBUS_SWITCH_LOW;
static uint8_t s_sbus_sub_prev = SBUS_SWITCH_LOW;
static uint8_t s_sbus_arm_active = 0U;
static uint8_t s_sbus_arm_enable_pending = 0U;
static fp32 s_sbus_arm_j0_target_deg = 0.0f;
static fp32 s_sbus_arm_j1_target_deg = 0.0f;
static uint32_t s_sbus_arm_last_ms = 0U;
static uint8_t s_sbus_safety_active = 0U;
static uint8_t s_sbus_safety_armed = 0U;
static uint8_t s_sbus_safety_wait_release_logged = 0U;
static uint8_t s_sbus_safety_needs_clear = 0U;
static uint32_t s_sbus_safety_recovery_since_ms = 0U;
static uint32_t s_sbus_lost_since_ms = 0U;
static uint32_t s_sbus_start_ms = 0U;
static uint8_t s_sbus_failsafe_stop_sent = 0U;
static uint8_t s_sbus_remote_lockout = 0U;
static uint8_t s_sbus_remote_lockout_logged = 0U;
static uint8_t s_sbus_quad_standing = 0U;
static uint8_t s_sbus_quad_cmd = SBUS_QUAD_STOP;
static uint8_t s_sbus_gait_retry_cmd = SBUS_QUAD_STOP;
static uint32_t s_sbus_gait_retry_ms = 0U;
static uint8_t s_sbus_gait_rearm_required = 0U;
static uint8_t s_sbus_drive_reverse_pending = 0U;
static uint8_t s_sbus_speed_profile = DOG_GAIT_SPEED_DEFAULT;

static const char *sbus_switch_name(uint8_t sw);
static uint8_t sbus_speed_profile_from_ch3(const SbusState *rc);
static void sbus_drive_input_from_sticks(const SbusState *rc, SbusDriveInput *input);
static uint8_t sbus_safety_raw_is_high(const SbusState *rc);
static uint8_t sbus_safety_raw_is_released(const SbusState *rc);
static uint8_t sbus_safety_update(const SbusState *rc);
static void sbus_update_switch_state(uint8_t main_sw, uint8_t sub_sw);

static void print_help(void)
{
    DebugUart_Printf("\r\nMWSDK CANSimple debug commands:\r\n");
    DebugUart_Printf("  L/R/B/N : select leg LF/RF/LB/RB (N = RB)\r\n");
    DebugUart_Printf("  1 : target selected leg hip motor only\r\n");
    DebugUart_Printf("  k : target selected leg knee motor only\r\n");
    DebugUart_Printf("  2 : target selected leg hip+knee\r\n");
    DebugUart_Printf("  3 : target front pair LF+RF\r\n");
    DebugUart_Printf("  4 : target rear pair LB+RB\r\n");
    DebugUart_Printf("  8 : target all 8 motors\r\n");
    DebugUart_Printf("  r : RX-only, no control TX\r\n");
    DebugUart_Printf("  e : clear errors on active target\r\n");
    DebugUart_Printf("  m : set trapezoidal position mode (Control=3 Input=5)\r\n");
    DebugUart_Printf("  o : MIT probe only, read real_angle, NO position TX\r\n");
    DebugUart_Printf("  b : swing PID boot probe+zero SWING_PID a/d/j/l Ilim=%ldmA\r\n",
                     (long)(g_dog_mit_motor_limits.current_limit_a * 1000.0f));
    DebugUart_Printf("  a/d : hip -/+ 30 deg  j/l : knee -/+ 30 deg  u/i : both -/+ 30 deg (SWING)\r\n");
    DebugUart_Printf("  s : stand STAND_PID foot IK front z=%ld rear z=%ld mm Ol=%ldmA\r\n",
                     (long)DOG_STAND_FOOT_Z_MM,
                     (long)(DOG_STAND_FOOT_Z_MM + DOG_REAR_FOOT_EXTRA_Z_MM),
                     (long)(g_dog_mit_stand_pid.output_limit_a * 1000.0f));
    DebugUart_Printf("  f : LF only, goto foot x=%ld z=%ld mm (SWING_PID, +x fwd branch)\r\n",
                     (long)DOG_FOOT_TARGET_X_MM,
                     (long)DOG_FOOT_TARGET_Z_MM);
    DebugUart_Printf("  g : walk march in place (3 STAND + 1 SWING), LF->RF->LB->RB, x=stop\r\n");
    DebugUart_Printf("  T : drive cycloid speed=%s hz=%ld.%01ld step=%ldmm height=%ldmm dwell=%lums stagger=%lums, x=stop\r\n",
                     dog_mit_gait_speed_profile_name(),
                     (long)dog_mit_gait_trot_hz(),
                      (long)(dog_mit_gait_trot_hz() * 10.0f) % 10L,
                      (long)dog_mit_gait_forward_stride_x_mm(),
                      (long)dog_mit_gait_swing_height_mm(),
                      (unsigned long)dog_mit_gait_touchdown_dwell_ms(),
                      (unsigned long)dog_mit_gait_diagonal_stagger_ms());
    DebugUart_Printf("  [ : turn left in place speed=%s stride=%ldmm, diag opposite LF+RB<->RF+LB, x=stop\r\n",
                     dog_mit_gait_speed_profile_name(),
                     (long)dog_mit_gait_turn_stride_x_mm());
    DebugUart_Printf("  ] : turn right in place speed=%s stride=%ldmm, diag opposite LF+RB<->RF+LB, x=stop\r\n",
                     dog_mit_gait_speed_profile_name(),
                     (long)dog_mit_gait_turn_stride_x_mm());
    DebugUart_Printf("  J : jump test STAND_PID snap land front=%ld rear=%ld apex=%ld mm, need 8+s first\r\n",
                     (long)DOG_JUMP_LAND_Z_MM,
                     (long)(DOG_STAND_FOOT_Z_MM + DOG_REAR_FOOT_EXTRA_Z_MM),
                     (long)DOG_JUMP_APEX_Z_MM);
    DebugUart_Printf("  D : LF+RB diagonal support test, RF+LB lifted, log cmdI/iq, D/x=stop\r\n");
    DebugUart_Printf("  h : MIT teach-hold, hand move then release to hold posture\r\n");
    DebugUart_Printf("  ! : disable all motors (SD/CH9 is the only ESTOP input)\r\n");
    DebugUart_Printf("  z : set user zero from current encoder feedback\r\n");
    DebugUart_Printf("  x : send idle (Axis_State=1) and stop TX\r\n");
    DebugUart_Printf("  c : toggle raw CAN RX log\r\n");
    DebugUart_Printf("  q : query encoder now (est+count, std+ext)\r\n");
    DebugUart_Printf("  p : print status\r\n");
    DebugUart_Printf("  Y : print SBUS remote raw/norm/mode diagnostics\r\n");
    DebugUart_Printf("  v : toggle VOFA JustFloat PID plot (500Hz, auto mutes text log)\r\n");
    DebugUart_Printf("  W : cycle VOFA watch motor M0..M7\r\n");
    DebugUart_Printf("  ? : print this help\r\n\r\n");
    DebugUart_Printf("SBUS remote on UART5:\r\n");
    DebugUart_Printf("  CH1 yaw + CH2 forward/reverse blend, CH3 speed profile, CH9 safety\r\n");
    DebugUart_Printf("  CH5 main: LOW=RX-only, MID=MIT stand, HIGH=gait\r\n");
    DebugUart_Printf("  CH8 sub: HIGH with CH5 MID enables arm jog mode\r\n");
    DebugUart_Printf("  CH6/CH7 arm jog: J0 DM4310 / J1 EL05, %.0f deg/s max\r\n\r\n",
                     (double)SBUS_ARM_RATE_DEG_S);
}

static void enter_rx_only(void)
{
    dog_debug_rx_only();
    s_mode = MODE_RX_ONLY;
    DebugUart_SetLogVerbose(0U);
    DebugUart_SetCanRxVerbose(0U);
    s_can_rx_log_enabled = 0U;
    DebugUart_Printf("RX-only leg=%s target=%s\r\n",
                     dog_leg_name(dog_leg_target_leg()),
                     dog_debug_target_name());
}

static void print_pid_gains(void)
{
    const Dog_Mit_Ang_Pid *active = dog_mit_active_ang_pid();
    DebugUart_Printf("Profile=%s act Kp=%ld Ki=%ld Kd=%ld Ol=%ldmA\r\n",
                     dog_mit_pid_profile_name(),
                     (long)(active->kp_a_per_deg * 1000000.0f),
                     (long)(active->ki_a_per_deg_s * 1000000.0f),
                     (long)(active->kd_a_per_dps * 1000000.0f),
                     (long)(active->output_limit_a * 1000.0f));
    DebugUart_Printf("STAND Kp=%ld Ki=%ld Kd=%ld Ol=%ld  SWING Kp=%ld Ki=%ld Kd=%ld Wl=%ld Ilim=%ld watch=M%u\r\n",
                     (long)(g_dog_mit_stand_pid.kp_a_per_deg * 1000000.0f),
                     (long)(g_dog_mit_stand_pid.ki_a_per_deg_s * 1000000.0f),
                     (long)(g_dog_mit_stand_pid.kd_a_per_dps * 1000000.0f),
                     (long)(g_dog_mit_stand_pid.output_limit_a * 1000.0f),
                     (long)(g_dog_mit_swing_pid.kp_a_per_deg * 1000000.0f),
                     (long)(g_dog_mit_swing_pid.ki_a_per_deg_s * 1000000.0f),
                     (long)(g_dog_mit_swing_pid.kd_a_per_dps * 1000000.0f),
                     (long)(g_dog_mit_swing_pid.output_limit_a * 1000.0f),
                     (long)(g_dog_mit_motor_limits.current_limit_a * 1000.0f),
                     (unsigned)VofaPid_GetMotorIndex());
}

static void toggle_vofa(void)
{
    if (VofaPid_IsEnabled() == 0U) {
        VofaPid_SetMotorIndex(dog_mit_get_default_vofa_motor_index());
        VofaPid_SetEnabled(1U);
        DebugUart_SetLogVerbose(0U);
        DebugUart_SetCanRxVerbose(0U);
        s_can_rx_log_enabled = 0U;
        DebugUart_Printf("VOFA ON 5ch: tgt/user/err/cmdI,P. Down: Kp/Ki/Kd/Ol Skp..Sl Wkp..Wl Il Motor:\r\n");
        print_pid_gains();
        return;
    }

    VofaPid_SetEnabled(0U);
    DebugUart_Printf("VOFA OFF. Text log still muted; send 'c' to re-enable CAN log.\r\n");
    print_pid_gains();
}

static void print_status(void)
{
    dog_leg_print_angle_status("STAT ");
    dog_leg_dump_target_status();
}

static void print_sbus_status(void)
{
    SbusState rc = {};
    const uint32_t now = HAL_GetTick();
    Sbus_Process();
    const uint8_t online = Sbus_GetState(&rc);
    const uint8_t fresh = Sbus_IsFresh(SBUS_REMOTE_TIMEOUT_MS);
    const uint32_t age = (rc.last_update_ms == 0U) ? 0xFFFFFFFFU :
                         (uint32_t)(now - rc.last_update_ms);
    const uint8_t main_sw = Sbus_Switch3(SBUS_MAIN_MODE_CH);
    const uint8_t sub_sw = Sbus_Switch3(SBUS_SUB_MODE_CH);
    const uint8_t speed = sbus_speed_profile_from_ch3(&rc);
    const uint8_t safety_active = sbus_safety_raw_is_high(&rc);
    SbusDriveInput drive = {};
    sbus_drive_input_from_sticks(&rc, &drive);
    float requested_forward = 0.0f;
    float requested_yaw = 0.0f;
    float applied_forward = 0.0f;
    float applied_yaw = 0.0f;
    dog_mit_drive_get_command(&requested_forward, &requested_yaw,
                              &applied_forward, &applied_yaw);

    DebugUart_Printf("SBUS stat: frames=%lu parse_err=%lu rx_evt=%lu rx_bytes=%lu last_size=%u flag=0x%02X online=%u get=%u fresh=%u age=%lums lost=%u failsafe=%u\r\n",
                     (unsigned long)rc.frame_count,
                     (unsigned long)rc.parse_error_count,
                     (unsigned long)rc.rx_event_count,
                     (unsigned long)rc.rx_byte_count,
                     (unsigned)rc.last_rx_size,
                     (unsigned)rc.flag,
                     (unsigned)rc.online,
                     (unsigned)online,
                     (unsigned)fresh,
                     (unsigned long)age,
                     (unsigned)rc.signal_lost,
                     (unsigned)rc.failsafe);
    DebugUart_Printf("  CH1 turn raw=%u norm=%d  CH2 move raw=%u norm=%d  CH3 speed raw=%u norm=%d -> %s\r\n",
                     (unsigned)rc.ch[0],
                     (int)rc.norm[0],
                     (unsigned)rc.ch[1],
                     (int)rc.norm[1],
                     (unsigned)rc.ch[SBUS_SPEED_CH],
                     (int)rc.norm[SBUS_SPEED_CH],
                     (speed == DOG_GAIT_SPEED_LOW) ? "LOW" :
                     ((speed == DOG_GAIT_SPEED_HIGH) ? "HIGH" : "MID"));
    DebugUart_Printf("  CH5 main raw=%u -> %s  CH8 sub raw=%u -> %s  CH9 safety raw=%u norm=%d high=%u armed=%u\r\n",
                     (unsigned)rc.ch[SBUS_MAIN_MODE_CH],
                     sbus_switch_name(main_sw),
                     (unsigned)rc.ch[SBUS_SUB_MODE_CH],
                     sbus_switch_name(sub_sw),
                     (unsigned)rc.ch[SBUS_SAFETY_CH],
                     (int)rc.norm[SBUS_SAFETY_CH],
                     (unsigned)safety_active,
                     (unsigned)s_sbus_safety_armed);
    DebugUart_Printf("  decoded: drive=%u stick=(%ld,%ld) req=(%ld,%ld) applied=(%ld,%ld) arm=%u standing=%u active_gait=%u estop=%u disabled=%u current_speed=%s\r\n",
                     (unsigned)drive.active,
                     (long)(drive.forward * 1000.0f),
                     (long)(drive.yaw * 1000.0f),
                     (long)(requested_forward * 1000.0f),
                     (long)(requested_yaw * 1000.0f),
                     (long)(applied_forward * 1000.0f),
                     (long)(applied_yaw * 1000.0f),
                     (unsigned)(((main_sw == SBUS_SWITCH_MID) &&
                                 (sub_sw == SBUS_SWITCH_HIGH)) ? 1U : 0U),
                     (unsigned)s_sbus_quad_standing,
                     (unsigned)dog_mit_march_in_place_is_active(),
                     (unsigned)DogSafety_IsLatched(),
                     (unsigned)DogStand_IsDisabled(),
                     dog_mit_gait_speed_profile_name());
}

static void set_target(uint8_t target)
{
    dog_debug_set_target(target);
    s_mode = MODE_RX_ONLY;
    DebugUart_Printf("Target=%s leg=%s\r\n",
                     dog_debug_target_name(),
                     dog_leg_name(dog_leg_target_leg()));
}

static void set_leg(uint8_t leg)
{
    dog_debug_set_target_leg(leg);
    s_mode = MODE_RX_ONLY;
    DebugUart_Printf("Leg=%s target=%s\r\n",
                     dog_leg_name(dog_leg_target_leg()),
                     dog_debug_target_name());
}

static void move_mit_debug_deg(float delta_hip, float delta_knee)
{
    if (dog_mit_debug_is_active() == 0U) {
        DebugUart_Printf("Send 'b' first (MIT boot: probe+zero+loop).\r\n");
        return;
    }

    float hip_cmd = 0.0f;
    float knee_cmd = 0.0f;
    dog_leg_get_target_leg_cmd_deg(&hip_cmd, &knee_cmd);
    dog_leg_set_target_motor_user_deg(hip_cmd + delta_hip, knee_cmd + delta_knee);
    dog_mit_send_control_now();
    s_mode = MODE_MIT_DEBUG;
}

static void mit_debug_boot(void)
{
    uint8_t ok = dog_debug_mit_boot_sequence();
    if (ok == 0U) {
        DebugUart_Printf("MIT boot FAIL.\r\n");
        return;
    }
    dog_mit_set_pid_profile(DOG_MIT_PID_SWING);
    s_mode = MODE_MIT_DEBUG;
    DebugUart_Printf("Swing PID boot OK %u/%u: a/d/j/l angle step, x=stop\r\n",
                     (unsigned)ok,
                     (unsigned)dog_debug_target_count());
    print_status();
}

static void hold_current(void)
{
    uint8_t ok = dog_debug_teach_hold_start();
    if (ok == 0U) {
        DebugUart_Printf("Teach-hold FAIL. Check online/fault/encoder, try e/o/p first.\r\n");
        return;
    }
    s_mode = MODE_HOLD;
    DebugUart_Printf("Teach-hold OK %u/%u loop=%s leg=%s target=%s. Move by hand, release to hold.\r\n",
                     (unsigned)ok,
                     (unsigned)dog_debug_target_count(),
                     dog_control_loop_mode_name(),
                     dog_leg_name(dog_leg_target_leg()),
                     dog_debug_target_name());
}

static void move_target(float delta_inner_deg, float delta_outer_deg)
{
    if (dog_mit_fault_hold_is_active() != 0U) {
        DebugUart_Printf("MIT safety fault-hold active. Send 'b' or 'h' to re-arm, 'x' or 'r' to stop.\r\n");
        return;
    }
    if (dog_mit_debug_is_active() != 0U) {
        move_mit_debug_deg(delta_inner_deg, delta_outer_deg);
        return;
    }
    if (dog_control_get_loop_mode() == DOG_CTRL_LOOP_MIT_PID) {
        DebugUart_Printf("MIT TX is stopped. Send 'b' or 'h' before a/d/j/l/u/i.\r\n");
        return;
    }

    float in = 0.0f;
    float out = 0.0f;
    dog_leg_get_target_leg_angles(&in, &out);
    if (s_mode != MODE_HOLD) {
        dog_debug_start_position_tx();
        s_mode = MODE_HOLD;
    }
    dog_leg_set_target_leg_deg(in + delta_inner_deg, out + delta_outer_deg);
}

static fp32 clamp_fp32_local(fp32 value, fp32 min_value, fp32 max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int16_t sbus_apply_deadband(int16_t value, int16_t deadband)
{
    if ((value > -deadband) && (value < deadband)) {
        return 0;
    }
    return value;
}

static const char *sbus_switch_name(uint8_t sw)
{
    switch (sw) {
    case SBUS_SWITCH_LOW:
        return "LOW";
    case SBUS_SWITCH_MID:
        return "MID";
    case SBUS_SWITCH_HIGH:
        return "HIGH";
    default:
        return "?";
    }
}

static void sbus_arm_leave(void)
{
    if ((s_sbus_arm_active == 0U) && (s_sbus_arm_enable_pending == 0U)) {
        return;
    }
    s_sbus_arm_active = 0U;
    s_sbus_arm_enable_pending = 0U;
    s_sbus_arm_last_ms = 0U;
    ArmMotor_Disable();
    DebugUart_Printf("SBUS arm mode OFF.\r\n");
}

static void sbus_arm_enter(uint32_t now)
{
    ArmMotorFeedback j0_feedback = {};
    ArmMotorFeedback j1_feedback = {};

    if ((DogSafety_IsLatched() != 0U) || (s_sbus_safety_active != 0U) ||
        (s_sbus_remote_lockout != 0U)) {
        return;
    }
    if (s_sbus_arm_active != 0U) {
        return;
    }
    if ((s_sbus_arm_last_ms != 0U) &&
        ((uint32_t)(now - s_sbus_arm_last_ms) < SBUS_ARM_RETRY_MS)) {
        return;
    }

    const uint8_t j0_online = ArmMotor_GetFeedback(ARM_J0_DM4310, &j0_feedback);
    const uint8_t j1_online = ArmMotor_GetFeedback(ARM_J1_LZ, &j1_feedback);
    if ((j0_online == 0U) && (j1_online == 0U)) {
        s_sbus_arm_last_ms = now;
        s_sbus_arm_enable_pending = 1U;
        ArmMotor_Enable();
        DebugUart_Printf("SBUS arm enable request sent; waiting for J0/J1 feedback.\r\n");
        return;
    }

    s_sbus_arm_active = 1U;
    s_sbus_arm_enable_pending = 0U;
    s_sbus_arm_last_ms = now;
    if (j0_online != 0U) {
        s_sbus_arm_j0_target_deg = clamp_fp32_local(j0_feedback.angle_deg,
                                                    SBUS_ARM_J0_MIN_DEG,
                                                    SBUS_ARM_J0_MAX_DEG);
    }
    if (j1_online != 0U) {
        s_sbus_arm_j1_target_deg = clamp_fp32_local(j1_feedback.angle_deg,
                                                    SBUS_ARM_J1_MIN_DEG,
                                                    SBUS_ARM_J1_MAX_DEG);
    }

    ArmMotor_Enable();
    if (j0_online != 0U) {
        ArmMotor_SetTargetDeg(ARM_J0_DM4310, s_sbus_arm_j0_target_deg, 0.0f, 0.0f);
    }
    if (j1_online != 0U) {
        ArmMotor_SetTargetDeg(ARM_J1_LZ, s_sbus_arm_j1_target_deg, 0.0f, 0.0f);
    }
    DebugUart_Printf("SBUS arm mode ON J0=%u J1=%u.\r\n",
                     (unsigned)j0_online, (unsigned)j1_online);
}

static void sbus_arm_update(const SbusState *rc, uint32_t now)
{
    if ((rc == nullptr) || (s_sbus_arm_active == 0U)) {
        return;
    }

    if ((uint32_t)(now - s_sbus_arm_last_ms) < SBUS_ARM_UPDATE_MS) {
        return;
    }

    ArmMotorFeedback j0_feedback = {};
    ArmMotorFeedback j1_feedback = {};
    const uint8_t j0_online = ArmMotor_GetFeedback(ARM_J0_DM4310, &j0_feedback);
    const uint8_t j1_online = ArmMotor_GetFeedback(ARM_J1_LZ, &j1_feedback);
    if ((j0_online == 0U) && (j1_online == 0U)) {
        DebugUart_Printf("SBUS arm feedback timeout: both joints offline; move main LOW to recover.\r\n");
        sbus_arm_leave();
        s_sbus_remote_lockout = 1U;
        s_sbus_remote_lockout_logged = 0U;
        return;
    }

    fp32 dt_s = (fp32)(now - s_sbus_arm_last_ms) * 0.001f;
    if (dt_s > 0.1f) {
        dt_s = 0.1f;
    }
    s_sbus_arm_last_ms = now;

    const int16_t j0_norm = sbus_apply_deadband(rc->norm[SBUS_ARM_J0_CH],
                                                SBUS_ARM_JOG_DEADBAND);
    const int16_t j1_norm = sbus_apply_deadband(rc->norm[SBUS_ARM_J1_CH],
                                                SBUS_ARM_JOG_DEADBAND);
    const fp32 j0_delta_deg = ((fp32)j0_norm / 100.0f) *
                              SBUS_ARM_RATE_DEG_S * dt_s;
    const fp32 j1_delta_deg = ((fp32)j1_norm / 100.0f) *
                              SBUS_ARM_RATE_DEG_S * dt_s;

    if (j0_online != 0U) {
        s_sbus_arm_j0_target_deg = clamp_fp32_local(s_sbus_arm_j0_target_deg + j0_delta_deg,
                                                    SBUS_ARM_J0_MIN_DEG,
                                                    SBUS_ARM_J0_MAX_DEG);
    }
    if (j1_online != 0U) {
        s_sbus_arm_j1_target_deg = clamp_fp32_local(s_sbus_arm_j1_target_deg + j1_delta_deg,
                                                    SBUS_ARM_J1_MIN_DEG,
                                                    SBUS_ARM_J1_MAX_DEG);
    }

    if (j0_online != 0U) {
        ArmMotor_SetTargetDeg(ARM_J0_DM4310, s_sbus_arm_j0_target_deg, 0.0f, 0.0f);
    }
    if (j1_online != 0U) {
        ArmMotor_SetTargetDeg(ARM_J1_LZ, s_sbus_arm_j1_target_deg, 0.0f, 0.0f);
    }
}

static uint8_t sbus_safety_raw_is_high(const SbusState *rc)
{
    if (rc == nullptr) {
        return 0U;
    }
    return (rc->norm[SBUS_SAFETY_CH] >= SBUS_SAFETY_HIGH_NORM) ? 1U : 0U;
}

static uint8_t sbus_safety_raw_is_released(const SbusState *rc)
{
    if (rc == nullptr) {
        return 0U;
    }
    return (rc->norm[SBUS_SAFETY_CH] <= SBUS_SAFETY_RELEASE_NORM) ? 1U : 0U;
}

static uint8_t sbus_safety_update(const SbusState *rc)
{
    if (sbus_safety_raw_is_released(rc) != 0U) {
        s_sbus_safety_armed = 1U;
        s_sbus_safety_wait_release_logged = 0U;
        return SBUS_SAFETY_RELEASED;
    }

    if (sbus_safety_raw_is_high(rc) != 0U) {
        s_sbus_safety_armed = 0U;
        if (s_sbus_safety_wait_release_logged == 0U) {
            DebugUart_Printf("SBUS CH9 safety inhibit active; release it and move main LOW.\r\n");
            s_sbus_safety_wait_release_logged = 1U;
        }
        return SBUS_SAFETY_TRIGGER;
    }

    return SBUS_SAFETY_INHIBIT;
}

static void sbus_quad_reset_state(void)
{
    s_sbus_quad_standing = 0U;
    s_sbus_quad_cmd = SBUS_QUAD_STOP;
    s_sbus_gait_retry_cmd = SBUS_QUAD_STOP;
    s_sbus_gait_retry_ms = 0U;
    s_sbus_gait_rearm_required = 0U;
    s_sbus_drive_reverse_pending = 0U;
}

static uint8_t sbus_speed_profile_from_ch3(const SbusState *rc)
{
    if (rc == nullptr) {
        return DOG_GAIT_SPEED_DEFAULT;
    }

    const int16_t speed = rc->norm[SBUS_SPEED_CH];
    if (speed <= SBUS_SPEED_LOW_NORM) {
        return DOG_GAIT_SPEED_LOW;
    }
    if (speed >= SBUS_SPEED_HIGH_NORM) {
        return DOG_GAIT_SPEED_HIGH;
    }
    return DOG_GAIT_SPEED_MID;
}

static void sbus_update_speed_profile(const SbusState *rc, uint8_t changed)
{
    const uint8_t profile = sbus_speed_profile_from_ch3(rc);
    if ((changed != 0U) || (profile != s_sbus_speed_profile)) {
        s_sbus_speed_profile = profile;
        dog_mit_set_gait_speed_profile(profile);
        DebugUart_Printf("SBUS CH3 speed=%s hz=%ld.%01ld step=%ldmm height=%ldmm dwell=%lums stagger=%lums turn=%ldmm\r\n",
                         dog_mit_gait_speed_profile_name(),
                         (long)dog_mit_gait_trot_hz(),
                         (long)(dog_mit_gait_trot_hz() * 10.0f) % 10L,
                         (long)dog_mit_gait_forward_stride_x_mm(),
                         (long)dog_mit_gait_swing_height_mm(),
                         (unsigned long)dog_mit_gait_touchdown_dwell_ms(),
                         (unsigned long)dog_mit_gait_diagonal_stagger_ms(),
                         (long)dog_mit_gait_turn_stride_x_mm());
    }
}

static void sbus_quad_stop_motion(uint8_t print_log)
{
    if (dog_mit_march_in_place_is_active() != 0U) {
        if (dog_mit_march_in_place_is_stopping() == 0U) {
            dog_mit_march_request_stop();
            if (print_log != 0U) {
                DebugUart_Printf("SBUS gait stopping at neutral stance.\r\n");
            }
        }
        return;
    }
    s_sbus_quad_cmd = SBUS_QUAD_STOP;
}

static void sbus_quad_stop_motion_immediate(uint8_t print_log)
{
    if (dog_mit_march_in_place_is_active() != 0U) {
        dog_mit_march_in_place_stop();
        if (print_log != 0U) {
            DebugUart_Printf("SBUS gait stopped immediately.\r\n");
        }
    }
    s_sbus_quad_cmd = SBUS_QUAD_STOP;
}

static void sbus_quad_rx_only(void)
{
    sbus_arm_leave();
    dog_debug_rx_only();
    s_mode = MODE_RX_ONLY;
    sbus_quad_reset_state();
}

static void sbus_update_switch_state(uint8_t main_sw, uint8_t sub_sw)
{
    s_sbus_switch_valid = 1U;
    s_sbus_main_prev = main_sw;
    s_sbus_sub_prev = sub_sw;
}

static uint8_t sbus_should_return_start_pose(uint8_t changed)
{
    if (changed == 0U) {
        return 0U;
    }
    if (s_sbus_main_prev != SBUS_SWITCH_MID) {
        return 0U;
    }
    if (s_sbus_quad_standing == 0U) {
        return 0U;
    }
    if (s_sbus_arm_active != 0U) {
        return 0U;
    }
    if (dog_mit_debug_is_active() == 0U) {
        return 0U;
    }
    if (dog_mit_fault_hold_is_active() != 0U) {
        return 0U;
    }
    if (dog_debug_target() != DOG_DEBUG_TARGET_ALL) {
        return 0U;
    }
    return 1U;
}

static void sbus_quad_low_mode(uint8_t main_sw, uint8_t sub_sw, uint8_t changed)
{
    (void)DogStand_ClearDisable();

    if ((s_sbus_arm_active != 0U) || (dog_mit_march_in_place_is_active() != 0U)) {
        sbus_quad_stop_motion_immediate(changed);
    }

    if (sbus_should_return_start_pose(changed) != 0U) {
        DebugUart_Printf("SBUS main LOW: return to start pose, then RX-only.\r\n");
        if (dog_mit_return_to_stand_start_pose() == 0U) {
            DebugUart_Printf("SBUS main LOW: return to start pose FAIL, RX-only fallback.\r\n");
            sbus_quad_rx_only();
        } else {
            s_mode = MODE_RX_ONLY;
            sbus_quad_reset_state();
        }
    } else if ((changed != 0U) || (s_sbus_arm_active != 0U)) {
        sbus_quad_rx_only();
        DebugUart_Printf("SBUS main LOW: RX-only, quadruped and arm TX stopped.\r\n");
    }

    sbus_update_switch_state(main_sw, sub_sw);
}

static uint8_t sbus_quad_ensure_stand(uint32_t now)
{
    if ((s_sbus_quad_standing != 0U) &&
        (dog_mit_debug_is_active() != 0U) &&
        (dog_mit_fault_hold_is_active() == 0U) &&
        (dog_debug_target() == DOG_DEBUG_TARGET_ALL)) {
        return 1U;
    }

    if ((s_sbus_gait_retry_cmd == SBUS_QUAD_STOP) &&
        (s_sbus_gait_retry_ms != 0U) &&
        ((uint32_t)(now - s_sbus_gait_retry_ms) < SBUS_GAIT_RETRY_MS)) {
        return 0U;
    }

    dog_debug_set_target(DOG_DEBUG_TARGET_ALL);
    uint8_t ok = dog_mit_stand_sequence();
    if (ok == 0U) {
        s_sbus_quad_standing = 0U;
        s_sbus_gait_retry_cmd = SBUS_QUAD_STOP;
        s_sbus_gait_retry_ms = now;
        DebugUart_Printf("SBUS stand FAIL: check online/enc/fault, or use serial e/o/p.\r\n");
        return 0U;
    }

    s_mode = MODE_MIT_DEBUG;
    s_sbus_quad_standing = 1U;
    s_sbus_quad_cmd = SBUS_QUAD_STOP;
    s_sbus_gait_retry_cmd = SBUS_QUAD_STOP;
    DebugUart_Printf("SBUS stand OK %u/%u.\r\n",
                     (unsigned)ok,
                     (unsigned)dog_debug_target_count());
    return 1U;
}

static float sbus_axis_to_drive(int16_t value)
{
    const int32_t magnitude = (value < 0) ? -(int32_t)value : (int32_t)value;
    if (magnitude <= SBUS_MOVE_EXIT_DEADBAND) {
        return 0.0f;
    }
    const float scaled = (float)(magnitude - SBUS_MOVE_EXIT_DEADBAND) /
        (float)(100 - SBUS_MOVE_EXIT_DEADBAND);
    const float limited = (scaled > 1.0f) ? 1.0f : scaled;
    return (value < 0) ? -limited : limited;
}

static void sbus_drive_input_from_sticks(const SbusState *rc, SbusDriveInput *input)
{
    if (input == nullptr) {
        return;
    }
    input->forward = 0.0f;
    input->yaw = 0.0f;
    input->active = 0U;
    if (rc == nullptr) {
        return;
    }

    const int16_t turn = rc->norm[0U];
    const int16_t move = rc->norm[1U];
    const int32_t abs_turn = (turn < 0) ? -(int32_t)turn : (int32_t)turn;
    const int32_t abs_move = (move < 0) ? -(int32_t)move : (int32_t)move;
    const uint8_t drive_active = ((s_sbus_quad_cmd == SBUS_QUAD_DRIVE) ||
                                  (s_sbus_gait_rearm_required != 0U) ||
                                  (dog_mit_march_in_place_is_active() != 0U)) ? 1U : 0U;
    const int32_t threshold = (drive_active != 0U) ?
        SBUS_MOVE_EXIT_DEADBAND : SBUS_MOVE_ENTER_DEADBAND;
    if ((abs_turn <= threshold) && (abs_move <= threshold)) {
        return;
    }

    input->forward = sbus_axis_to_drive(move);
    input->yaw = sbus_axis_to_drive(turn);
    input->active = 1U;
}

static uint8_t sbus_quad_drive_command(const SbusDriveInput *drive, uint32_t now)
{
    if ((drive == nullptr) || (drive->active == 0U)) {
        s_sbus_gait_rearm_required = 0U;
        s_sbus_drive_reverse_pending = 0U;
        s_sbus_gait_retry_cmd = SBUS_QUAD_STOP;
        s_sbus_gait_retry_ms = 0U;
        sbus_quad_stop_motion(1U);
        return 1U;
    }

    if (s_sbus_gait_rearm_required != 0U) {
        return 0U;
    }

    if (dog_mit_march_in_place_is_active() != 0U) {
        if (dog_mit_march_in_place_is_stopping() != 0U) {
            return 0U;
        }
        if (dog_mit_trot_march_is_active() == 0U) {
            sbus_quad_stop_motion(1U);
            DebugUart_Printf("SBUS stick input: stopping in-place march before drive.\r\n");
            return 0U;
        }
        float applied_forward = 0.0f;
        dog_mit_drive_get_command(nullptr, nullptr, &applied_forward, nullptr);
        if ((fabsf(applied_forward) >= 0.20f) && (fabsf(drive->forward) >= 0.20f) &&
            ((applied_forward * drive->forward) < 0.0f)) {
            s_sbus_drive_reverse_pending = 1U;
            sbus_quad_stop_motion(1U);
            DebugUart_Printf("SBUS drive reversing after neutral stop.\r\n");
            return 0U;
        }
        s_sbus_quad_cmd = SBUS_QUAD_DRIVE;
        return dog_mit_drive_update(drive->forward, drive->yaw);
    }

    if ((s_sbus_quad_cmd == SBUS_QUAD_DRIVE) &&
        (s_sbus_drive_reverse_pending == 0U)) {
        s_sbus_quad_cmd = SBUS_QUAD_STOP;
        s_sbus_gait_rearm_required = 1U;
        DebugUart_Printf("SBUS gait stopped by stability gate; center sticks before retry.\r\n");
        return 0U;
    }
    if ((s_sbus_gait_retry_cmd == SBUS_QUAD_DRIVE) &&
        ((uint32_t)(now - s_sbus_gait_retry_ms) < SBUS_GAIT_RETRY_MS)) {
        return 0U;
    }

    const uint8_t ok = dog_mit_drive_start(0U, drive->forward, drive->yaw);
    if (ok == 0U) {
        s_sbus_quad_cmd = SBUS_QUAD_STOP;
        s_sbus_gait_retry_cmd = SBUS_QUAD_DRIVE;
        s_sbus_gait_retry_ms = now;
        s_sbus_gait_rearm_required = 1U;
        s_sbus_drive_reverse_pending = 0U;
        DebugUart_Printf("SBUS drive FAIL; center sticks before retry.\r\n");
        return 0U;
    }

    s_mode = MODE_MIT_DEBUG;
    s_sbus_quad_cmd = SBUS_QUAD_DRIVE;
    s_sbus_gait_retry_cmd = SBUS_QUAD_STOP;
    s_sbus_drive_reverse_pending = 0U;
    DebugUart_Printf("SBUS drive start f=%ld yaw=%ld.\r\n",
                     (long)(drive->forward * 1000.0f),
                     (long)(drive->yaw * 1000.0f));
    return 1U;
}

static uint8_t sbus_quad_start_in_place(uint32_t now)
{
    if (dog_mit_march_in_place_is_active() != 0U) {
        return 1U;
    }
    if ((s_sbus_gait_retry_cmd == SBUS_QUAD_STOP) &&
        (s_sbus_gait_retry_ms != 0U) &&
        ((uint32_t)(now - s_sbus_gait_retry_ms) < SBUS_GAIT_RETRY_MS)) {
        return 0U;
    }

    if (dog_mit_march_in_place_start(0U) == 0U) {
        s_sbus_gait_retry_cmd = SBUS_QUAD_STOP;
        s_sbus_gait_retry_ms = now;
        DebugUart_Printf("SBUS in-place march FAIL: check online/enc/fault.\r\n");
        return 0U;
    }

    s_mode = MODE_MIT_DEBUG;
    s_sbus_quad_cmd = SBUS_QUAD_STOP;
    s_sbus_gait_retry_cmd = SBUS_QUAD_STOP;
    s_sbus_gait_retry_ms = 0U;
    DebugUart_Printf("SBUS main MID->HIGH: in-place march speed=%s.\r\n",
                     dog_mit_gait_speed_profile_name());
    return 1U;
}

static uint8_t command_allowed_while_inhibited(char c)
{
    switch (c) {
    case '!':
    case 'x':
    case 'r':
    case 'e':
    case 'p':
    case 'q':
    case 'Y':
    case 'y':
    case 'c':
    case 'v':
    case 'W':
    case '?':
    case 'L':
    case 'R':
    case 'B':
    case 'N':
    case '1':
    case '2':
    case '3':
    case '4':
    case '8':
    case 'k':
        return 1U;
    default:
        return 0U;
    }
}

static void sbus_safety_trigger(void)
{
    sbus_arm_leave();
    dog_debug_set_target(DOG_DEBUG_TARGET_ALL);
    DogSafety_SetSdEstop(1U);
    sbus_quad_reset_state();
    s_sbus_safety_needs_clear = 1U;
    s_sbus_safety_recovery_since_ms = 0U;
    s_sbus_remote_lockout = 1U;
    s_sbus_remote_lockout_logged = 0U;
    s_mode = MODE_IDLE;
    DebugUart_Printf("SBUS CH9 safety: ESTOP latched; release CH9 and move main LOW to re-arm.\r\n");
}

static void sbus_remote_failsafe(uint32_t now)
{
    Dog_Remote_Sample sample = {};

    if (s_sbus_lost_since_ms == 0U) {
        s_sbus_lost_since_ms = now;
    }

    if (s_sbus_failsafe_stop_sent == 0U) {
        DebugUart_Printf("SBUS lost/failsafe: all motors disabled; restore signal and move main LOW.\r\n");
        DogStand_Disable();
        sbus_arm_leave();
        sbus_quad_reset_state();
        s_mode = MODE_RX_ONLY;
        s_sbus_arm_active = 0U;
        s_sbus_arm_last_ms = 0U;
        s_sbus_failsafe_stop_sent = 1U;
        s_sbus_remote_lockout = 1U;
        s_sbus_remote_lockout_logged = 0U;
        s_sbus_safety_recovery_since_ms = 0U;
    }

    s_sbus_seen_fresh = 0U;
    s_sbus_switch_valid = 0U;
    sample.estop_request = 0U;
    sample.tick_ms = now;
    DogRemote_Update(&sample);
}

void control_task_safety_poll(void)
{
    SbusState rc = {};
    const uint32_t now = HAL_GetTick();

    Sbus_Process();
    (void)Sbus_GetState(&rc);
    if (rc.frame_count == 0U) {
        if ((uint32_t)(now - s_sbus_start_ms) >= SBUS_REMOTE_TIMEOUT_MS) {
            sbus_remote_failsafe(now);
        }
        return;
    }
    if (Sbus_IsFresh(SBUS_REMOTE_TIMEOUT_MS) == 0U) {
        sbus_remote_failsafe(now);
        return;
    }

    const uint8_t safety_state = sbus_safety_update(&rc);
    if (safety_state != SBUS_SAFETY_RELEASED) {
        if ((safety_state == SBUS_SAFETY_TRIGGER) && (DogSafety_IsLatched() == 0U)) {
            sbus_safety_trigger();
        } else if (safety_state == SBUS_SAFETY_TRIGGER) {
            DogSafety_SetSdEstop(1U);
        }
        s_sbus_safety_active = 1U;
        s_sbus_safety_recovery_since_ms = 0U;
        return;
    }

    DogSafety_SetSdEstop(0U);
    s_sbus_safety_active = 0U;
}

static void sbus_control_update(void)
{
    SbusState rc = {};
    Dog_Remote_Sample sample = {};
    const uint32_t now = HAL_GetTick();

    Sbus_Process();
    (void)Sbus_GetState(&rc);

    if (rc.frame_count == 0U) {
        if ((uint32_t)(now - s_sbus_start_ms) >= SBUS_REMOTE_TIMEOUT_MS) {
            sbus_remote_failsafe(now);
        }
        return;
    }

    if (Sbus_IsFresh(SBUS_REMOTE_TIMEOUT_MS) == 0U) {
        sbus_remote_failsafe(now);
        return;
    }

    if ((s_sbus_lost_since_ms != 0U) || (s_sbus_failsafe_stop_sent != 0U)) {
        if (s_sbus_lost_since_ms != 0U) {
            DebugUart_Printf("SBUS restored after %lums.\r\n",
                             (unsigned long)(now - s_sbus_lost_since_ms));
        } else {
            DebugUart_Printf("SBUS restored from failsafe frames.\r\n");
        }
        s_sbus_lost_since_ms = 0U;
        s_sbus_failsafe_stop_sent = 0U;
        s_sbus_remote_lockout = 1U;
        s_sbus_remote_lockout_logged = 0U;
    }

    const uint8_t main_sw = Sbus_Switch3(SBUS_MAIN_MODE_CH);
    const uint8_t sub_sw = Sbus_Switch3(SBUS_SUB_MODE_CH);
    const uint8_t safety_state = sbus_safety_update(&rc);
    const uint8_t changed = ((s_sbus_switch_valid == 0U) ||
                             (main_sw != s_sbus_main_prev) ||
                             (sub_sw != s_sbus_sub_prev)) ? 1U : 0U;
    const uint8_t arm_mode = ((main_sw == SBUS_SWITCH_MID) &&
                              (sub_sw == SBUS_SWITCH_HIGH)) ? 1U : 0U;

    if (s_sbus_seen_fresh == 0U) {
        DebugUart_Printf("SBUS online main=%s sub=%s.\r\n",
                         sbus_switch_name(main_sw),
                         sbus_switch_name(sub_sw));
        s_sbus_remote_lockout = 1U;
        s_sbus_remote_lockout_logged = 0U;
    }
    s_sbus_seen_fresh = 1U;
    sbus_update_speed_profile(&rc, (s_sbus_switch_valid == 0U) ? 1U : 0U);

    sample.mode = (uint8_t)(main_sw * 3U + sub_sw);
    sample.tick_ms = now;

    if (safety_state != SBUS_SAFETY_RELEASED) {
        if ((safety_state == SBUS_SAFETY_TRIGGER) && (DogSafety_IsLatched() == 0U)) {
            sample.estop_request = 1U;
            DogRemote_Update(&sample);
            sbus_safety_trigger();
        } else if (safety_state == SBUS_SAFETY_TRIGGER) {
            DogSafety_SetSdEstop(1U);
        }
        s_sbus_safety_active = 1U;
        s_sbus_safety_recovery_since_ms = 0U;
        sbus_update_switch_state(main_sw, sub_sw);
        return;
    }
    DogSafety_SetSdEstop(0U);
    s_sbus_safety_active = 0U;

    if ((DogSafety_IsLatched() != 0U) && (s_sbus_safety_needs_clear == 0U)) {
        sbus_arm_leave();
        sbus_quad_reset_state();
        s_sbus_safety_needs_clear = 1U;
        s_sbus_safety_recovery_since_ms = 0U;
        s_sbus_remote_lockout = 1U;
        s_sbus_remote_lockout_logged = 0U;
        DebugUart_Printf("Motor safety latch active; release CH9 and move main LOW to recover.\r\n");
    }

    if ((DogStand_IsDisabled() != 0U) && (s_sbus_remote_lockout == 0U)) {
        sbus_arm_leave();
        sbus_quad_reset_state();
        s_sbus_remote_lockout = 1U;
        s_sbus_remote_lockout_logged = 0U;
        DebugUart_Printf("Motor control disabled; move main LOW before enabling again.\r\n");
    }

    if (s_sbus_remote_lockout != 0U) {
        if (main_sw == SBUS_SWITCH_LOW) {
            if (s_sbus_safety_needs_clear != 0U) {
                if (s_sbus_safety_recovery_since_ms == 0U) {
                    s_sbus_safety_recovery_since_ms = now;
                }
                if ((uint32_t)(now - s_sbus_safety_recovery_since_ms) < SBUS_SAFETY_RECOVERY_MS) {
                    DogRemote_Update(&sample);
                    sbus_update_switch_state(main_sw, sub_sw);
                    return;
                }
                (void)DogSafety_RequestRearm();
            }
            if (DogSafety_IsLatched() != 0U) {
                DogRemote_Update(&sample);
                sbus_update_switch_state(main_sw, sub_sw);
                return;
            }
            if (s_sbus_safety_needs_clear != 0U) {
                s_sbus_safety_needs_clear = 0U;
                s_sbus_safety_recovery_since_ms = 0U;
                DebugUart_Printf("SBUS safety latch confirmed clear in main LOW.\r\n");
            }
            s_sbus_remote_lockout = 0U;
            s_sbus_remote_lockout_logged = 0U;
            DebugUart_Printf("SBUS remote lockout cleared by main LOW.\r\n");
        } else {
            s_sbus_safety_recovery_since_ms = 0U;
            if (s_sbus_remote_lockout_logged == 0U) {
                DebugUart_Printf("SBUS remote lockout: move main switch LOW before MID/HIGH control resumes.\r\n");
                s_sbus_remote_lockout_logged = 1U;
            }
            DogRemote_Update(&sample);
            sbus_update_switch_state(main_sw, sub_sw);
            return;
        }
    }

    if (main_sw == SBUS_SWITCH_LOW) {
        sbus_quad_low_mode(main_sw, sub_sw, changed);
        DogRemote_Update(&sample);
        return;
    }

    if (arm_mode != 0U) {
        if (changed != 0U) {
            DebugUart_Printf("SBUS main MID/sub HIGH: arm jog mode, CH6=J0 CH7=J1.\r\n");
        }
        sbus_quad_stop_motion(changed);
        if (dog_mit_march_in_place_is_active() == 0U) {
            sbus_arm_enter(now);
            sbus_arm_update(&rc, now);
        }
    } else {
        if ((s_sbus_arm_active != 0U) || (s_sbus_arm_enable_pending != 0U)) {
            sbus_arm_leave();
        }

        if (main_sw == SBUS_SWITCH_MID) {
            sbus_quad_stop_motion(changed);
            if ((changed != 0U) || (s_sbus_quad_standing == 0U)) {
                DebugUart_Printf("SBUS main MID: MIT stand mode.\r\n");
            }
            (void)sbus_quad_ensure_stand(now);
        } else if (main_sw == SBUS_SWITCH_HIGH) {
            if (sbus_quad_ensure_stand(now) != 0U) {
                SbusDriveInput drive = {};
                sbus_drive_input_from_sticks(&rc, &drive);
                if (drive.active != 0U) {
                    (void)sbus_quad_drive_command(&drive, now);
                } else if (dog_mit_trot_march_is_active() != 0U) {
                    sbus_quad_stop_motion(1U);
                } else if ((changed != 0U) && (s_sbus_main_prev == SBUS_SWITCH_MID)) {
                    (void)sbus_quad_start_in_place(now);
                }
            }
        }
    }

    DogRemote_Update(&sample);
    sbus_update_switch_state(main_sw, sub_sw);
}

static void handle_command(char c)
{
    if ((c == '\r') || (c == '\n') || (c == ' ')) {
        return;
    }

    if (((DogSafety_IsLatched() != 0U) || (s_sbus_safety_active != 0U) ||
         (s_sbus_remote_lockout != 0U)) && (command_allowed_while_inhibited(c) == 0U)) {
        DebugUart_Printf("Command '%c' blocked by safety/remote lockout.\r\n", c);
        return;
    }

    switch (c) {
    case 'L':
        set_leg(DOG_LEG_LF);
        break;
    case 'R':
        set_leg(DOG_LEG_RF);
        break;
    case 'B':
        set_leg(DOG_LEG_LB);
        break;
    case 'N':
        set_leg(DOG_LEG_RB);
        break;
    case '1':
        set_target(DOG_DEBUG_TARGET_SINGLE);
        break;
    case 'k':
        set_target(DOG_DEBUG_TARGET_SINGLE_KNEE);
        break;
    case '2':
        set_target(DOG_DEBUG_TARGET_LEG);
        break;
    case '3':
        set_target(DOG_DEBUG_TARGET_FRONT_PAIR);
        break;
    case '4':
        set_target(DOG_DEBUG_TARGET_REAR_PAIR);
        break;
    case '8':
        set_target(DOG_DEBUG_TARGET_ALL);
        break;
    case 'r':
        (void)DogStand_ClearDisable();
        enter_rx_only();
        break;
    case 'e':
        if (DogSafety_IsLatched() != 0U) {
            if ((s_sbus_remote_lockout != 0U) || (s_sbus_seen_fresh != 0U)) {
                DebugUart_Printf("Safety re-arm requires fresh SBUS with main LOW.\r\n");
            } else if (DogSafety_RequestRearm() != 0U) {
                DebugUart_Printf("Safety re-arm requested; wait for all 8 motors idle/fault-free.\r\n");
            } else {
                DebugUart_Printf("Safety re-arm blocked by active CH9 inhibit.\r\n");
            }
        } else {
            dog_debug_clear_errors();
            DebugUart_Printf("Clear errors sent.\r\n");
        }
        break;
    case 'm':
        dog_debug_position_setup();
        DebugUart_Printf("Position mode setup sent.\r\n");
        break;
    case 'o':
        dog_debug_enter_closed_loop();
        break;
    case 'b':
        mit_debug_boot();
        break;
    case 'h':
        hold_current();
        break;
    case '!':
        DogStand_Disable();
        s_mode = MODE_IDLE;
        DebugUart_Printf("All motors disabled. SD/CH9 is the only ESTOP input.\r\n");
        break;
    case 'a':
        if (dog_debug_target() != DOG_DEBUG_TARGET_SINGLE_KNEE) {
            move_target(-STEP_DEG, 0.0f);
        }
        break;
    case 'd':
        if (dog_debug_target() != DOG_DEBUG_TARGET_SINGLE_KNEE) {
            move_target(STEP_DEG, 0.0f);
        }
        break;
    case 'j':
        if (dog_debug_target() != DOG_DEBUG_TARGET_SINGLE) {
            move_target(0.0f, -STEP_DEG);
        }
        break;
    case 'l':
        if (dog_debug_target() != DOG_DEBUG_TARGET_SINGLE) {
            move_target(0.0f, STEP_DEG);
        }
        break;
    case 'u':
        if (dog_debug_target() == DOG_DEBUG_TARGET_SINGLE) {
            move_target(-STEP_DEG, 0.0f);
        } else if (dog_debug_target() == DOG_DEBUG_TARGET_SINGLE_KNEE) {
            move_target(0.0f, -STEP_DEG);
        } else {
            move_target(-STEP_DEG, -STEP_DEG);
        }
        break;
    case 'i':
        if (dog_debug_target() == DOG_DEBUG_TARGET_SINGLE) {
            move_target(STEP_DEG, 0.0f);
        } else if (dog_debug_target() == DOG_DEBUG_TARGET_SINGLE_KNEE) {
            move_target(0.0f, STEP_DEG);
        } else {
            move_target(STEP_DEG, STEP_DEG);
        }
        break;
    case 'z':
    {
        uint8_t ok = dog_leg_set_target_zero_current();
        DebugUart_Printf("User zero set for %u selected motor(s).\r\n", (unsigned)ok);
        print_status();
        break;
    }
    case 'x':
        dog_debug_idle();
        (void)DogStand_ClearDisable();
        s_mode = MODE_IDLE;
        DebugUart_Printf("Idle; MIT debug stopped.\r\n");
        break;
    case 's':
    {
        uint8_t ok = dog_mit_stand_sequence();
        if (ok == 0U) {
            if (dog_mit_fault_hold_is_active() != 0U) {
                DebugUart_Printf("Stand FAIL: fault-hold. Send x then retry.\r\n");
            } else {
                DebugUart_Printf("Stand FAIL: probe/zero/boot. Check e/o/p, online/enc.\r\n");
            }
        } else {
            s_mode = MODE_MIT_DEBUG;
            DebugUart_Printf("Stand OK %u/%u: foot IK front (%ld,%ld)->(%ld,%ld) rear (%ld,%ld)->(%ld,%ld)mm target=%s\r\n",
                             (unsigned)ok,
                             (unsigned)dog_debug_target_count(),
                             (long)DOG_STAND_FOOT_X_MM,
                             (long)DOG_STAND_FOOT_Z_START_MM,
                             (long)DOG_STAND_FOOT_X_MM,
                             (long)DOG_STAND_FOOT_Z_MM,
                             (long)DOG_STAND_FOOT_X_MM,
                             (long)(DOG_STAND_FOOT_Z_START_MM + DOG_REAR_FOOT_EXTRA_Z_MM),
                             (long)DOG_STAND_FOOT_X_MM,
                             (long)(DOG_STAND_FOOT_Z_MM + DOG_REAR_FOOT_EXTRA_Z_MM),
                             dog_debug_target_name());
            dog_mit_print_all_motor_current("Stand ");
        }
        break;
    }
    case 'f':
    {
        if (dog_mit_debug_is_active() == 0U) {
            DebugUart_Printf("Send 's' first (stand MIT loop).\r\n");
            break;
        }

        const float x_mm = DOG_FOOT_TARGET_X_MM;
        const float z_mm = DOG_FOOT_TARGET_Z_MM;
        const uint8_t leg = DOG_FOOT_GOTO_LEG;
        float hip = 0.0f;
        float knee = 0.0f;
        float fk_x = 0.0f;
        float fk_z = 0.0f;
        uint8_t reachable = dog_leg_foot_xz_is_reachable(x_mm, z_mm);
        uint8_t ik_ok = dog_leg_foot_xz_to_motor_deg(leg, x_mm, z_mm, &hip, &knee);

        DebugUart_Printf("Foot IK tgt=(%ld,%ld)mm leg=%s reachable=%u ik=%u\r\n",
                         (long)x_mm,
                         (long)z_mm,
                         dog_leg_name(leg),
                         (unsigned)reachable,
                         (unsigned)ik_ok);
        if (ik_ok != 0U) {
            (void)dog_leg_foot_xz_from_motor_deg(leg, hip, knee, &fk_x, &fk_z);
            DebugUart_Printf("  motor=%ld/%ld deg fk_xz=(%ld,%ld)mm\r\n",
                             (long)hip,
                             (long)knee,
                             (long)fk_x,
                             (long)fk_z);
        }

        if ((reachable == 0U) || (ik_ok == 0U)) {
            DebugUart_Printf("Foot goto FAIL: unreachable or out of motor limits.\r\n");
            break;
        }

        uint8_t ok = dog_mit_goto_foot_xz(x_mm, z_mm);
        if (ok == 0U) {
            DebugUart_Printf("Foot goto FAIL: move timeout or fault-hold.\r\n");
        } else {
            s_mode = MODE_MIT_DEBUG;
            DebugUart_Printf("Foot goto OK: LF SWING_PID motor=%ld/%ld\r\n",
                             (long)hip,
                             (long)knee);
            dog_leg_print_angle_status_for_leg("STAT ", DOG_FOOT_GOTO_LEG);
        }
        break;
    }
    case 'g':
    {
        if (dog_mit_debug_is_active() == 0U) {
            DebugUart_Printf("Send '8' then 's' first (all-leg stand MIT loop).\r\n");
            break;
        }
        if (dog_debug_target() != DOG_DEBUG_TARGET_ALL) {
            DebugUart_Printf("Send '8' to target all 8 motors before march.\r\n");
            break;
        }
        if (dog_mit_march_in_place_is_active() != 0U) {
            dog_mit_march_in_place_stop();
            DebugUart_Printf("March stopped.\r\n");
            break;
        }

        uint8_t ok = dog_mit_march_in_place_start(0U);
        if (ok == 0U) {
            DebugUart_Printf("March FAIL: need all 8 online/booted, no fault-hold.\r\n");
        } else {
            s_mode = MODE_MIT_DEBUG;
        }
        break;
    }
    case 'J':
    {
        if (dog_mit_march_in_place_is_active() != 0U) {
            dog_mit_march_in_place_stop();
        }
        if (dog_mit_diag_support_is_active() != 0U) {
            dog_mit_diag_support_stop();
        }

        uint8_t ok = dog_mit_jump_test_sequence();
        if (ok != 0U) {
            s_mode = MODE_MIT_DEBUG;
            dog_mit_print_all_motor_current("Jump ");
        }
        break;
    }
    case '[':
    {
        if (dog_mit_debug_is_active() == 0U) {
            DebugUart_Printf("Send '8' then 's' first (all-leg stand MIT loop).\r\n");
            break;
        }
        if (dog_debug_target() != DOG_DEBUG_TARGET_ALL) {
            DebugUart_Printf("Send '8' to target all 8 motors before turn.\r\n");
            break;
        }
        if (dog_mit_march_in_place_is_active() != 0U) {
            dog_mit_march_in_place_stop();
            DebugUart_Printf("March stopped.\r\n");
            break;
        }

        uint8_t ok = dog_mit_turn_left_in_place_start(0U);
        if (ok != 0U) {
            s_mode = MODE_MIT_DEBUG;
            dog_mit_print_all_motor_current("TurnL ");
        }
        break;
    }
    case ']':
    {
        if (dog_mit_debug_is_active() == 0U) {
            DebugUart_Printf("Send '8' then 's' first (all-leg stand MIT loop).\r\n");
            break;
        }
        if (dog_debug_target() != DOG_DEBUG_TARGET_ALL) {
            DebugUart_Printf("Send '8' to target all 8 motors before turn.\r\n");
            break;
        }
        if (dog_mit_march_in_place_is_active() != 0U) {
            dog_mit_march_in_place_stop();
            DebugUart_Printf("March stopped.\r\n");
            break;
        }

        uint8_t ok = dog_mit_turn_right_in_place_start(0U);
        if (ok != 0U) {
            s_mode = MODE_MIT_DEBUG;
            dog_mit_print_all_motor_current("TurnR ");
        }
        break;
    }
    case 'T':
    {
        if (dog_mit_debug_is_active() == 0U) {
            DebugUart_Printf("Send '8' then 's' first (all-leg stand MIT loop).\r\n");
            break;
        }
        if (dog_debug_target() != DOG_DEBUG_TARGET_ALL) {
            DebugUart_Printf("Send '8' to target all 8 motors before trot.\r\n");
            break;
        }
        if (dog_mit_march_in_place_is_active() != 0U) {
            dog_mit_march_in_place_stop();
            DebugUart_Printf("Trot stopped.\r\n");
            break;
        }

        uint8_t ok = dog_mit_trot_in_place_start(0U);
        if (ok != 0U) {
            s_mode = MODE_MIT_DEBUG;
            dog_mit_print_all_motor_current("Trot ");
        }
        break;
    }
    case 'D':
    {
        if (dog_mit_debug_is_active() == 0U) {
            DebugUart_Printf("Send '8' then 's' first (all-leg stand MIT loop).\r\n");
            break;
        }
        if (dog_debug_target() != DOG_DEBUG_TARGET_ALL) {
            DebugUart_Printf("Send '8' to target all 8 motors before diag support.\r\n");
            break;
        }
        if (dog_mit_diag_support_is_active() != 0U) {
            dog_mit_diag_support_stop();
            DebugUart_Printf("Diag LF+RB support stopped.\r\n");
            break;
        }

        uint8_t ok = dog_mit_diag_support_lf_rb_start();
        if (ok == 0U) {
            DebugUart_Printf("Diag FAIL: need all 8 online/booted, no fault-hold.\r\n");
        } else {
            s_mode = MODE_MIT_DEBUG;
        }
        break;
    }
    case 'c':
        s_can_rx_log_enabled = (s_can_rx_log_enabled == 0U) ? 1U : 0U;
        DebugUart_SetLogVerbose(s_can_rx_log_enabled);
        DebugUart_SetCanRxVerbose(s_can_rx_log_enabled);
        DebugUart_Printf("Raw CAN RX log %s\r\n", (s_can_rx_log_enabled != 0U) ? "ON" : "OFF");
        break;
    case 'q':
        dog_motor_query_online_encoders();
        dog_motor_poll_can();
        DebugUart_Printf("Encoder query sent for all online motors.\r\n");
        print_status();
        break;
    case 'p':
        print_status();
        break;
    case 'Y':
    case 'y':
        print_sbus_status();
        break;
    case 'v':
        toggle_vofa();
        break;
    case 'W':
        VofaPid_CycleMotorIndex();
        print_pid_gains();
        break;
    case '?':
        print_help();
        break;
    default:
        DebugUart_Printf("Unknown command '%c'. Send '?' for help.\r\n", c);
        break;
    }
}

static void update_target(void)
{
    sbus_control_update();
    (void)s_mode;
}

static void lf_periodic_status(void)
{
    if (VofaPid_IsEnabled() != 0U) {
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_last_lf_status_ms) < DEBUG_LF_STATUS_MS) {
        return;
    }
    s_last_lf_status_ms = now;

    if (dog_mit_diag_support_is_active() != 0U) {
        dog_diag_support_print_status();
        return;
    }

    if (dog_mit_trot_march_is_active() != 0U) {
        dog_mit_print_all_motor_current("Trot ");
        return;
    }

    dog_lf_print_periodic_status();
}

static void control_init_wait(uint32_t timeout_ms, uint8_t stop_when_host_open)
{
    const uint32_t start_ms = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start_ms) < timeout_ms) {
        control_task_safety_poll();
        DebugUart_Process();
        if ((stop_when_host_open != 0U) && (DebugUart_IsHostOpen() != 0U)) {
            return;
        }
        osDelay(10U);
    }
}

void control_task_init(void)
{
    VofaPid_Init();
    Sbus_Init();
    s_sbus_start_ms = HAL_GetTick();

    control_init_wait(1500U, 1U);
    control_init_wait(500U, 0U);

    print_help();
    enter_rx_only();
}

void control_task(void)
{
    update_target();

    int ch;
    while ((ch = DebugUart_GetByte()) >= 0) {
        if (VofaPid_IsEnabled() != 0U) {
            if (VofaPid_FeedRxByte((uint8_t)ch) != 0U) {
                continue;
            }
        }
        handle_command((char)ch);
    }

    lf_periodic_status();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    (void)htim;
}
