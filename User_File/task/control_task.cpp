#include "control_task.h"

#include "arm_motor_task.h"
#include "debug_uart.h"
#include "motor_task.h"
#include "sbus.h"
#include "tim.h"
#include "usb_frame_protocol.h"
#include "usbd_cdc_if.h"
#include "vofa_pid.h"
#include "wheel_motor_task.h"

#include "cmsis_os2.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define DEBUG_LF_STATUS_MS 1000U
#define STEP_DEG           30.0f

#ifndef USB_CDC_PRODUCTION_INTERFACE
#define USB_CDC_PRODUCTION_INTERFACE 1
#endif

#define USB_RC_TIMEOUT_MS      150U

#define SBUS_REMOTE_TIMEOUT_MS 250U
#define SBUS_FAILSAFE_CONFIRM_MS 100U
#define SBUS_RECOVERY_GOOD_FRAMES 3U
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
#define SBUS_FAULT_CLEAR_RETRY_MS 1000U
#define SBUS_MECHANICAL_PREPARE_MS     200U
#define SBUS_HYBRID_MAX_WHEEL_CONTRIBUTION 0.30f
#define SBUS_WHEEL_DIAMETER_MM          116.0f
#define SBUS_PI                         3.14159265358979323846f

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

enum SbusRobotMode {
    SBUS_MODE_NONE = 0U,
    SBUS_MODE_MOTOR_CHECK,
    SBUS_MODE_LOW_WHEEL,
    SBUS_MODE_LOW_SAFE,
    SBUS_MODE_USB_IDLE,
    SBUS_MODE_STAND_HOLD,
    SBUS_MODE_STAND_WHEEL,
    SBUS_MODE_STAND_ARM,
    SBUS_MODE_GAIT_WHEEL,
    SBUS_MODE_GAIT_ONLY,
    SBUS_MODE_RESERVED_STAND,
};

enum SbusModeEntryState {
    SBUS_ENTRY_INACTIVE = 0U,
    SBUS_ENTRY_ENTERING,
    SBUS_ENTRY_WAIT_LOWER,
    SBUS_ENTRY_WAIT_MECHANICAL,
    SBUS_ENTRY_WAIT_GAIT_STOP,
    SBUS_ENTRY_WAIT_STAND,
    SBUS_ENTRY_WAIT_ARM,
    SBUS_ENTRY_ACTIVE,
    SBUS_ENTRY_BLOCKED,
};

enum SbusModeBlockReason {
    SBUS_BLOCK_NONE = 0U,
    SBUS_BLOCK_LOWER,
    SBUS_BLOCK_MECHANICAL_PERMIT,
    SBUS_BLOCK_GAIT_STOP,
    SBUS_BLOCK_STAND,
    SBUS_BLOCK_ARM_FEEDBACK,
    SBUS_BLOCK_WHEEL_FAULT,
    SBUS_BLOCK_LEG_FAULT,
};

enum SbusLinkState {
    SBUS_LINK_GOOD = 0U,
    SBUS_LINK_TRANSIENT,
    SBUS_LINK_LOST,
};

enum ControlInputSource {
    CONTROL_SOURCE_SBUS = 0U,
    CONTROL_SOURCE_USB,
    CONTROL_SOURCE_NONE,
};

struct SbusDriveInput {
    float forward;
    float yaw;
    uint8_t active;
};

static ControlMode s_mode = MODE_RX_ONLY;
static uint8_t s_can_rx_log_enabled = 0U;
#if !USB_CDC_PRODUCTION_INTERFACE
static uint32_t s_last_lf_status_ms = 0U;
#endif
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
static uint32_t s_sbus_link_bad_since_ms = 0U;
static uint32_t s_sbus_link_last_frame = 0U;
static uint8_t s_sbus_link_good_frames = 0U;
static uint8_t s_sbus_remote_lockout = 0U;
static uint8_t s_sbus_remote_lockout_logged = 0U;
static uint8_t s_sbus_quad_standing = 0U;
static uint8_t s_sbus_quad_cmd = SBUS_QUAD_STOP;
static uint8_t s_sbus_gait_retry_cmd = SBUS_QUAD_STOP;
static uint32_t s_sbus_gait_retry_ms = 0U;
static uint8_t s_sbus_drive_reverse_pending = 0U;
static uint8_t s_sbus_speed_profile = DOG_GAIT_SPEED_DEFAULT;
static uint8_t s_sbus_mechanical_permit = 0U;
static uint32_t s_sbus_mechanical_prepare_since_ms = 0U;
static SbusRobotMode s_sbus_requested_mode = SBUS_MODE_NONE;
static SbusRobotMode s_sbus_active_mode = SBUS_MODE_NONE;
static SbusModeEntryState s_sbus_entry_state = SBUS_ENTRY_INACTIVE;
static SbusModeBlockReason s_sbus_block_reason = SBUS_BLOCK_NONE;
static uint32_t s_sbus_motor_check_last_ms = 0U;
static uint32_t s_sbus_fault_clear_last_ms = 0U;
static uint8_t s_sbus_hybrid_wheel_degraded = 0U;
static uint8_t s_sbus_lowering_pending = 0U;
static ControlInputSource s_control_source = CONTROL_SOURCE_SBUS;
static UsbVirtualRcSample s_usb_applied_rc = {};
static uint8_t s_usb_session_active = 0U;
static uint8_t s_usb_release_hold = 0U;
static uint32_t s_usb_session_id = 0U;
static uint32_t s_usb_last_counter = 0U;
static uint32_t s_usb_last_generation = 0U;
static uint32_t s_usb_last_accepted_ms = 0U;
static uint32_t s_usb_blocked_session_id = 0U;

static const char *sbus_switch_name(uint8_t sw);
static uint8_t sbus_speed_profile_from_ch3(const SbusState *rc);
static void sbus_drive_input_from_sticks(const SbusState *rc, SbusDriveInput *input);
static uint8_t sbus_safety_raw_is_high(const SbusState *rc);
static uint8_t sbus_safety_raw_is_released(const SbusState *rc);
static uint8_t sbus_safety_update(const SbusState *rc);
static void sbus_update_switch_state(uint8_t main_sw, uint8_t sub_sw);
static SbusLinkState sbus_link_state_update(const SbusState *rc, uint32_t now);
static float sbus_axis_to_drive(int16_t value);
static float sbus_wheel_max_rpm(const SbusState *rc);
static void sbus_wheel_hold(void);
static void sbus_wheel_disable(uint8_t lock);
static uint8_t sbus_wheel_update(const SbusState *rc);
static float sbus_hybrid_wheel_contribution(const SbusState *rc,
                                             float forward, float yaw);
static void sbus_hybrid_wheel_update(const SbusState *rc,
                                     const DogGaitSyncState *sync);
static SbusRobotMode sbus_decode_robot_mode(uint8_t main_sw, uint8_t sub_sw);
static const char *sbus_robot_mode_name(SbusRobotMode mode);
static const char *sbus_entry_state_name(SbusModeEntryState state);
static const char *sbus_block_reason_name(SbusModeBlockReason reason);
static void sbus_mode_transition(SbusRobotMode requested);
static void sbus_mode_tick(SbusRobotMode requested, const SbusState *rc,
                           uint8_t changed, uint32_t now);
static const char *control_source_name(ControlInputSource source);
static void usb_control_revoke(uint8_t hold);

static void print_wheel_status(void)
{
    WheelDriveDiag diag = {};
    WheelDrive_GetDiag(&diag);
    DebugUart_Printf("WHEEL: can=%u mode=%u op=%u lock=%u brake=%u hybrid=%u online=%u stopped=%u seen=0x%X profile=%u peak=0x%X therm=0x%X hot=0x%X req=%ld/%ldrpm txfail=%lu busoff=%lu timeout=%lu cmdtimeout=%lu reject=%lu\r\n",
                     (unsigned)diag.can_ready,
                     (unsigned)diag.mode_enabled,
                     (unsigned)diag.operating_mode,
                     (unsigned)diag.locked,
                     (unsigned)diag.brake_active,
                     (unsigned)diag.hybrid_mode,
                     (unsigned)diag.all_online,
                     (unsigned)diag.stopped,
                     (unsigned)diag.feedback_seen_mask,
                     (unsigned)diag.profile,
                     (unsigned)diag.peak_limited_mask,
                     (unsigned)diag.thermal_derated_mask,
                     (unsigned)diag.overtemp_mask,
                     (long)diag.requested_left_rpm,
                     (long)diag.requested_right_rpm,
                     (unsigned long)diag.tx_fail_count,
                     (unsigned long)diag.bus_off_count,
                     (unsigned long)diag.feedback_timeout_count,
                     (unsigned long)diag.command_timeout_count,
                     (unsigned long)diag.rx_reject_count);
    for (uint8_t i = 0U; i < WHEEL_MOTOR_COUNT; ++i) {
        DebugUart_Printf("  W%u enc=%u rounds=%ld req=%ld scale=%ld final=%ld target=%ldrpm speed=%ldrpm cmd=%d lim=%d peak=%ums iq=%d temp=%u age=%lums\r\n",
                         (unsigned)(i + 1U),
                         (unsigned)diag.motor[i].encoder_raw,
                         (long)diag.motor[i].encoder_rounds,
                         (long)diag.requested_target_rpm[i],
                         (long)(diag.phase_scale[i] * 1000.0f),
                         (long)diag.final_target_rpm[i],
                         (long)diag.ramped_target_rpm[i],
                         (long)diag.vehicle_speed_rpm[i],
                         (int)diag.current_cmd[i],
                         (int)diag.current_limit_raw[i],
                         (unsigned)diag.peak_budget_ms[i],
                         (int)diag.motor[i].torque_current_raw,
                         (unsigned)diag.motor[i].temperature_c,
                         (unsigned long)(HAL_GetTick() - diag.motor[i].last_update_ms));
    }
}

static void print_arm_status(void)
{
    static const char *const names[ARM_JOINT_COUNT] = {"J0 DM4310", "J1 EL05"};
    static const char *const frame_types[ARM_JOINT_COUNT] = {"std", "ext"};
    static const char *const id_names[ARM_JOINT_COUNT] = {"feedback_id", "motor_id"};
    static const uint8_t buses[ARM_JOINT_COUNT] = {1U, 2U};
    static const uint16_t ids[ARM_JOINT_COUNT] = {0x10U, 0x7FU};

    DebugUart_Printf("ARM: init=%u diag_period=300ms present_timeout=1000ms feedback_timeout=100ms\r\n",
                     (unsigned)ArmMotor_IsInitialized());
    for (uint8_t joint = 0U; joint < ARM_JOINT_COUNT; ++joint) {
        ArmMotorFeedback feedback = {};
        const uint8_t online = ArmMotor_GetFeedback(joint, &feedback);
        DebugUart_Printf("  %s CAN%u %s %s=0x%02X present=%u enabled=%u online=%u feedback_age_ms=",
                         names[joint],
                         (unsigned)buses[joint],
                         frame_types[joint],
                         id_names[joint],
                         (unsigned)ids[joint],
                         (unsigned)feedback.present,
                         (unsigned)feedback.enabled,
                         (unsigned)online);
        if (feedback.feedback_age_ms == ARM_MOTOR_FEEDBACK_AGE_INVALID) {
            DebugUart_Printf("NA fault=");
        } else {
            DebugUart_Printf("%lu fault=", (unsigned long)feedback.feedback_age_ms);
        }
        if (feedback.fault_valid == 0U) {
            DebugUart_Printf("NA");
        } else {
            DebugUart_Printf("%u", (unsigned)feedback.fault);
        }
        DebugUart_Printf(" err=%u mode=%u angle=%ldmdeg vel=%ldmrad/s torque=%ldmNm temp=%lddC\r\n",
                         (unsigned)feedback.error,
                         (unsigned)feedback.mode,
                         (long)(feedback.angle_deg * 1000.0f),
                         (long)(feedback.vel_rad_s * 1000.0f),
                         (long)(feedback.torque_nm * 1000.0f),
                         (long)(feedback.temperature_c * 10.0f));
    }
}

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
    DebugUart_Printf("  Wheel: CH1 differential yaw, CH2 forward/reverse, CH3 0..200rpm limit\r\n");
    DebugUart_Printf("  Gait: CH1/CH2 blend, CH3 LOW/MID/HIGH profile, CH9 safety\r\n");
    DebugUart_Printf("  SB+SC: L+L=motor check/OFF, L+M=mechanical wheel, L+H=RX-only/HOLD\r\n");
    DebugUart_Printf("  SB+SC: M+L=stand/HOLD, M+M=stand wheel, M+H=stand arm/HOLD\r\n");
    DebugUart_Printf("  SB+SC: H+L=gait only/HOLD, H+M=gait wheel, H+H=reserved stand/HOLD\r\n");
    DebugUart_Printf("  CH6/CH7 arm jog: J0 DM4310 / J1 EL05, %.0f deg/s max\r\n\r\n",
                     (double)SBUS_ARM_RATE_DEG_S);
}

static void enter_rx_only(void)
{
    WheelDrive_SetOperatingMode(WHEEL_OPERATING_OFF);
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
    print_wheel_status();
    print_arm_status();
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
    const uint32_t bad_ms = (s_sbus_link_bad_since_ms == 0U) ? 0U :
                            (uint32_t)(now - s_sbus_link_bad_since_ms);
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
    DogGaitSyncState gait = {};
    dog_mit_get_gait_sync_state(&gait);

    DebugUart_Printf("SBUS stat: frames=%lu parse_err=%lu rx_evt=%lu rx_bytes=%lu last_size=%u flag=0x%02X online=%u get=%u fresh=%u age=%lums lost=%u failsafe=%u debounce=%lums good=%u/%u confirmed=%u\r\n",
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
                     (unsigned)rc.failsafe,
                     (unsigned long)bad_ms,
                     (unsigned)s_sbus_link_good_frames,
                     (unsigned)SBUS_RECOVERY_GOOD_FRAMES,
                     (unsigned)s_sbus_failsafe_stop_sent);
    DebugUart_Printf("  CH1 turn raw=%u norm=%d  CH2 move raw=%u norm=%d  CH3 speed raw=%u norm=%d -> wheel=%ldrpm gait=%s\r\n",
                     (unsigned)rc.ch[0],
                     (int)rc.norm[0],
                     (unsigned)rc.ch[1],
                     (int)rc.norm[1],
                     (unsigned)rc.ch[SBUS_SPEED_CH],
                     (int)rc.norm[SBUS_SPEED_CH],
                     (long)sbus_wheel_max_rpm(&rc),
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
    DebugUart_Printf("  mode: decoded=%s requested=%s active=%s entry=%s block=%s\r\n",
                     sbus_robot_mode_name(sbus_decode_robot_mode(main_sw, sub_sw)),
                     sbus_robot_mode_name(s_sbus_requested_mode),
                     sbus_robot_mode_name(s_sbus_active_mode),
                     sbus_entry_state_name(s_sbus_entry_state),
                     sbus_block_reason_name(s_sbus_block_reason));
    DebugUart_Printf("  decoded: drive=%u stick=(%ld,%ld) req=(%ld,%ld) applied=(%ld,%ld) arm=%u standing=%u active_gait=%u estop=%u disabled=%u current_speed=%s\r\n",
                     (unsigned)drive.active,
                     (long)(drive.forward * 1000.0f),
                     (long)(drive.yaw * 1000.0f),
                     (long)(requested_forward * 1000.0f),
                     (long)(requested_yaw * 1000.0f),
                     (long)(applied_forward * 1000.0f),
                     (long)(applied_yaw * 1000.0f),
                     (unsigned)(s_sbus_requested_mode == SBUS_MODE_STAND_ARM),
                     (unsigned)s_sbus_quad_standing,
                     (unsigned)dog_mit_march_in_place_is_active(),
                     (unsigned)DogSafety_IsLatched(),
                     (unsigned)DogStand_IsDisabled(),
                      dog_mit_gait_speed_profile_name());
    DebugUart_Printf("  mechanical: pose=%u pose_mask=0x%02X pose_ready=%u hip_lift=%ldmdeg permit=%u prep_ms=%lu\r\n",
                     (unsigned)DogStand_IsMechanicalLimitPose(),
                     (unsigned)DogStand_GetMechanicalLimitPoseMask(),
                     (unsigned)DogStand_IsMechanicalLimitPoseReady(),
                     (long)(DOG_LOW_WHEEL_HIP_LIFT_DEG * 1000.0f),
                     (unsigned)s_sbus_mechanical_permit,
                     (unsigned long)((s_sbus_mechanical_prepare_since_ms == 0U) ? 0U :
                                     (now - s_sbus_mechanical_prepare_since_ms)));
    DebugUart_Printf("  gait sync: phase=%u gen=%lu swing=0x%X contact=0x%X search=0x%X fail=0x%X progress=%ld/%ld/%ld/%ld wheel=%ld leg=%ld compat=%ldrpm search_um=%ld/%ld/%ld/%ld degraded=%u\r\n",
                     (unsigned)gait.phase,
                     (unsigned long)gait.half_step_generation,
                     (unsigned)gait.swing_mask,
                     (unsigned)gait.contact_mask,
                     (unsigned)gait.contact_search_mask,
                     (unsigned)gait.contact_failure_mask,
                     (long)(gait.swing_progress[0U] * 1000.0f),
                     (long)(gait.swing_progress[1U] * 1000.0f),
                     (long)(gait.swing_progress[2U] * 1000.0f),
                     (long)(gait.swing_progress[3U] * 1000.0f),
                     (long)(gait.active_wheel_contribution * 1000.0f),
                     (long)(gait.active_leg_contribution * 1000.0f),
                     (long)gait.compatible_wheel_rpm,
                     (long)(gait.contact_search_mm[0U] * 1000.0f),
                     (long)(gait.contact_search_mm[1U] * 1000.0f),
                     (long)(gait.contact_search_mm[2U] * 1000.0f),
                     (long)(gait.contact_search_mm[3U] * 1000.0f),
                     (unsigned)s_sbus_hybrid_wheel_degraded);
    UsbFrameProtocolDiag usb_diag = {};
    UsbVirtualRcSample usb_rc = {};
    UsbFrameProtocol_GetDiag(&usb_diag);
    const uint8_t have_usb_rc = UsbFrameProtocol_GetVirtualRc(&usb_rc);
    const uint32_t usb_age = (have_usb_rc != 0U) ?
        (uint32_t)(now - usb_rc.received_ms) : 0xFFFFFFFFU;
    DebugUart_Printf("  USB: source=%s session=%u id=0x%08lX counter=%lu age=%lums rx=%lu ctrl=%lu imu=%lu hdr=%lu crc=%lu reject=%lu seqgap=%lu parse_to=%lu ring_drop=%lu hold=%u\r\n",
                     control_source_name(s_control_source),
                     (unsigned)s_usb_session_active,
                     (unsigned long)s_usb_session_id,
                     (unsigned long)s_usb_last_counter,
                     (unsigned long)usb_age,
                     (unsigned long)usb_diag.rx_bytes,
                     (unsigned long)usb_diag.valid_control_frames,
                     (unsigned long)usb_diag.valid_imu_frames,
                     (unsigned long)usb_diag.header_error_count,
                     (unsigned long)usb_diag.crc_error_count,
                     (unsigned long)usb_diag.payload_reject_count,
                     (unsigned long)usb_diag.sequence_gap_count,
                     (unsigned long)usb_diag.parser_timeout_count,
                     (unsigned long)CDC_GetRxDropCount_HS(),
                     (unsigned)s_usb_release_hold);
    print_wheel_status();
    print_arm_status();
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

static SbusRobotMode sbus_decode_robot_mode(uint8_t main_sw, uint8_t sub_sw)
{
    if (main_sw == SBUS_SWITCH_LOW) {
        if (sub_sw == SBUS_SWITCH_LOW) return SBUS_MODE_MOTOR_CHECK;
        if (sub_sw == SBUS_SWITCH_MID) return SBUS_MODE_LOW_WHEEL;
        return SBUS_MODE_LOW_SAFE;
    }
    if (main_sw == SBUS_SWITCH_MID) {
        if (sub_sw == SBUS_SWITCH_LOW) return SBUS_MODE_STAND_HOLD;
        if (sub_sw == SBUS_SWITCH_MID) return SBUS_MODE_STAND_WHEEL;
        return SBUS_MODE_STAND_ARM;
    }
    if (sub_sw == SBUS_SWITCH_LOW) return SBUS_MODE_GAIT_ONLY;
    if (sub_sw == SBUS_SWITCH_MID) return SBUS_MODE_GAIT_WHEEL;
    return SBUS_MODE_RESERVED_STAND;
}

static uint8_t sbus_mode_is_low(SbusRobotMode mode)
{
    return ((mode == SBUS_MODE_MOTOR_CHECK) ||
            (mode == SBUS_MODE_LOW_WHEEL) ||
            (mode == SBUS_MODE_LOW_SAFE) ||
            (mode == SBUS_MODE_USB_IDLE)) ? 1U : 0U;
}

static uint8_t sbus_mode_is_mid(SbusRobotMode mode)
{
    return ((mode == SBUS_MODE_STAND_HOLD) ||
            (mode == SBUS_MODE_STAND_WHEEL) ||
            (mode == SBUS_MODE_STAND_ARM)) ? 1U : 0U;
}

static const char *sbus_robot_mode_name(SbusRobotMode mode)
{
    switch (mode) {
    case SBUS_MODE_MOTOR_CHECK:   return "MOTOR_CHECK";
    case SBUS_MODE_LOW_WHEEL:     return "LOW_WHEEL";
    case SBUS_MODE_LOW_SAFE:      return "LOW_SAFE";
    case SBUS_MODE_USB_IDLE:      return "USB_IDLE";
    case SBUS_MODE_STAND_HOLD:    return "STAND_HOLD";
    case SBUS_MODE_STAND_WHEEL:   return "STAND_WHEEL";
    case SBUS_MODE_STAND_ARM:     return "STAND_ARM";
    case SBUS_MODE_GAIT_WHEEL:    return "GAIT_WHEEL";
    case SBUS_MODE_GAIT_ONLY:     return "GAIT_ONLY";
    case SBUS_MODE_RESERVED_STAND:return "RESERVED_STAND";
    default:                      return "NONE";
    }
}

static const char *sbus_entry_state_name(SbusModeEntryState state)
{
    switch (state) {
    case SBUS_ENTRY_ENTERING:        return "ENTERING";
    case SBUS_ENTRY_WAIT_LOWER:      return "WAIT_LOWER";
    case SBUS_ENTRY_WAIT_MECHANICAL: return "WAIT_MECH";
    case SBUS_ENTRY_WAIT_GAIT_STOP:  return "WAIT_GAIT";
    case SBUS_ENTRY_WAIT_STAND:      return "WAIT_STAND";
    case SBUS_ENTRY_WAIT_ARM:        return "WAIT_ARM";
    case SBUS_ENTRY_ACTIVE:          return "ACTIVE";
    case SBUS_ENTRY_BLOCKED:         return "BLOCKED";
    default:                         return "INACTIVE";
    }
}

static const char *sbus_block_reason_name(SbusModeBlockReason reason)
{
    switch (reason) {
    case SBUS_BLOCK_LOWER:            return "LOWER";
    case SBUS_BLOCK_MECHANICAL_PERMIT: return "MECH_PERMIT";
    case SBUS_BLOCK_GAIT_STOP:         return "GAIT_STOP";
    case SBUS_BLOCK_STAND:             return "STAND";
    case SBUS_BLOCK_ARM_FEEDBACK:      return "ARM_FEEDBACK";
    case SBUS_BLOCK_WHEEL_FAULT:       return "WHEEL_FAULT";
    case SBUS_BLOCK_LEG_FAULT:         return "LEG_FAULT";
    default:                           return "NONE";
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
        DebugUart_Printf("SBUS arm feedback timeout: both joints offline; arm output stopped.\r\n");
        sbus_arm_leave();
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
    s_sbus_drive_reverse_pending = 0U;
}

static void sbus_wheel_disable(uint8_t lock)
{
    WheelDrive_SetOperatingMode(WHEEL_OPERATING_OFF);
    if (lock != 0U) {
        WheelDrive_StopAndLock();
    }
}

static void sbus_wheel_hold(void)
{
    WheelDrive_SetOperatingMode(WHEEL_OPERATING_HOLD);
}

static void sbus_mechanical_cancel(void)
{
    s_sbus_mechanical_permit = 0U;
    s_sbus_mechanical_prepare_since_ms = 0U;
    DogStand_ExitMechanicalLimitIdle();
    DogStand_ExitMechanicalLimitPose();
}

static void sbus_mechanical_prepare(const SbusState *rc, uint32_t now)
{
    const uint8_t neutral = ((rc != nullptr) &&
        (rc->norm[0U] >= -SBUS_MOVE_EXIT_DEADBAND) &&
        (rc->norm[0U] <= SBUS_MOVE_EXIT_DEADBAND) &&
        (rc->norm[1U] >= -SBUS_MOVE_EXIT_DEADBAND) &&
        (rc->norm[1U] <= SBUS_MOVE_EXIT_DEADBAND)) ? 1U : 0U;
    if ((neutral == 0U) || (WheelDrive_AllOnline() == 0U) ||
        (WheelDrive_IsStopped() == 0U)) {
        s_sbus_mechanical_permit = 0U;
        s_sbus_mechanical_prepare_since_ms = 0U;
        return;
    }
    if (s_sbus_mechanical_prepare_since_ms == 0U) {
        s_sbus_mechanical_prepare_since_ms = now;
        return;
    }
    if ((uint32_t)(now - s_sbus_mechanical_prepare_since_ms) >=
        SBUS_MECHANICAL_PREPARE_MS) {
        s_sbus_mechanical_permit = 1U;
    }
}

static float sbus_wheel_max_rpm(const SbusState *rc)
{
    if (rc == nullptr) {
        return 0.0f;
    }
    const float rpm = (float)rc->norm[SBUS_SPEED_CH] + 100.0f;
    return fminf(fmaxf(rpm, 0.0f), WHEEL_MAX_OUTPUT_RPM);
}

static uint8_t sbus_wheel_update(const SbusState *rc)
{
    if ((rc == nullptr) || (WheelDrive_IsAvailable() == 0U)) {
        sbus_wheel_hold();
        return 0U;
    }

    const float forward = sbus_axis_to_drive(rc->norm[1U]);
    const float turn = sbus_axis_to_drive(rc->norm[0U]);
    const float max_rpm = sbus_wheel_max_rpm(rc);
    WheelDrive_SetOperatingMode(WHEEL_OPERATING_DRIVE);
    WheelDrive_SetMotion(forward, turn, max_rpm);
    return 1U;
}

static float sbus_smoothstep5(float value)
{
    const float x = fminf(fmaxf(value, 0.0f), 1.0f);
    return x * x * x * (x * (x * 6.0f - 15.0f) + 10.0f);
}

static float sbus_gait_compatible_rpm(float forward, float yaw,
                                      float forward_stride_mm,
                                      float turn_stride_mm,
                                      uint32_t swing_ms)
{
    if (swing_ms == 0U) {
        return 0.0f;
    }
    const float left_mm = forward * forward_stride_mm + yaw * turn_stride_mm;
    const float right_mm = forward * forward_stride_mm - yaw * turn_stride_mm;
    const float travel_mm = fmaxf(fabsf(left_mm), fabsf(right_mm));
    return travel_mm * 1000.0f / (float)swing_ms * 60.0f /
           (SBUS_PI * SBUS_WHEEL_DIAMETER_MM);
}

static float sbus_hybrid_wheel_contribution(const SbusState *rc,
                                             float forward, float yaw)
{
    const float requested_rpm = sbus_gait_compatible_rpm(
        forward, yaw, dog_mit_gait_forward_stride_x_mm(),
        dog_mit_gait_turn_stride_x_mm(), dog_mit_gait_trot_swing_ms());
    DogGaitSyncState sync = {};
    dog_mit_get_gait_sync_state(&sync);
    const float limiting_rpm = fmaxf(requested_rpm, sync.compatible_wheel_rpm);
    if (limiting_rpm <= 1.0e-3f) {
        return 0.0f;
    }
    return fminf(SBUS_HYBRID_MAX_WHEEL_CONTRIBUTION,
                 sbus_wheel_max_rpm(rc) / limiting_rpm);
}

static void sbus_hybrid_wheel_update(const SbusState *rc,
                                     const DogGaitSyncState *sync)
{
    if ((rc == nullptr) || (sync == nullptr) || (sync->active == 0U) ||
        (sync->active_wheel_contribution <= 1.0e-3f) ||
        (WheelDrive_IsAvailable() == 0U)) {
        sbus_wheel_hold();
        return;
    }

    float phase_scale[WHEEL_MOTOR_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};
    if (sync->phase == DOG_GAIT_PHASE_ENTRY_SETTLE) {
        memset(phase_scale, 0, sizeof(phase_scale));
    } else if (sync->phase == DOG_GAIT_PHASE_SWING) {
        static const uint8_t leg_to_wheel[DOG_LEG_COUNT] = {0U, 1U, 3U, 2U};
        for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
            if ((sync->swing_mask & (uint8_t)(1U << leg)) == 0U) {
                continue;
            }
            const float progress = sync->swing_progress[leg];
            float scale = 0.0f;
            if (progress >= 0.90f) {
                scale = 1.0f;
            } else if (progress > 0.60f) {
                scale = sbus_smoothstep5((progress - 0.60f) / 0.30f);
            }
            phase_scale[leg_to_wheel[leg]] = scale;
        }
    } else if (sync->phase == DOG_GAIT_PHASE_STOP_NEUTRAL) {
        const float stop_scale = 1.0f - sync->stop_progress;
        for (uint8_t wheel = 0U; wheel < WHEEL_MOTOR_COUNT; ++wheel) {
            phase_scale[wheel] = stop_scale;
        }
    }

    const float swing_s = (float)sync->active_swing_ms * 0.001f;
    if (swing_s <= 0.0f) {
        sbus_wheel_hold();
        return;
    }
    const float rpm_per_mm = 60.0f /
        (swing_s * SBUS_PI * SBUS_WHEEL_DIAMETER_MM);
    const float left_rpm = (sync->applied_forward * sync->active_forward_stride_x_mm +
                            sync->applied_yaw * sync->active_turn_stride_x_mm) *
                           rpm_per_mm * sync->active_wheel_contribution;
    const float right_rpm = (sync->applied_forward * sync->active_forward_stride_x_mm -
                             sync->applied_yaw * sync->active_turn_stride_x_mm) *
                            rpm_per_mm * sync->active_wheel_contribution;
    const float limit_rpm = sbus_wheel_max_rpm(rc);
    const float targets[WHEEL_MOTOR_COUNT] = {
        fminf(fmaxf(left_rpm, -limit_rpm), limit_rpm),
        fminf(fmaxf(right_rpm, -limit_rpm), limit_rpm),
        fminf(fmaxf(right_rpm, -limit_rpm), limit_rpm),
        fminf(fmaxf(left_rpm, -limit_rpm), limit_rpm),
    };
    WheelDrive_SetOperatingMode(WHEEL_OPERATING_DRIVE);
    WheelDrive_SetWheelTargets(targets, phase_scale);
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

static void sbus_update_switch_state(uint8_t main_sw, uint8_t sub_sw)
{
    s_sbus_switch_valid = 1U;
    s_sbus_main_prev = main_sw;
    s_sbus_sub_prev = sub_sw;
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

    const uint8_t fault_mask = DogStand_GetFaultMask();
    if (fault_mask != 0U) {
        if ((s_sbus_fault_clear_last_ms == 0U) ||
            ((uint32_t)(now - s_sbus_fault_clear_last_ms) >= SBUS_FAULT_CLEAR_RETRY_MS)) {
            s_sbus_fault_clear_last_ms = now;
            dog_debug_clear_errors();
            DebugUart_Printf("SBUS leg fault=0x%02X: clear requested; stand will retry automatically.\r\n",
                             (unsigned)fault_mask);
        }
        return 0U;
    }
    s_sbus_fault_clear_last_ms = 0U;

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
        s_sbus_drive_reverse_pending = 0U;
        s_sbus_gait_retry_cmd = SBUS_QUAD_STOP;
        s_sbus_gait_retry_ms = 0U;
        sbus_quad_stop_motion(1U);
        return 1U;
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
        s_sbus_gait_retry_cmd = SBUS_QUAD_DRIVE;
        s_sbus_gait_retry_ms = now;
        DebugUart_Printf("SBUS gait stopped by stability gate; retrying current stick command.\r\n");
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
        s_sbus_drive_reverse_pending = 0U;
        DebugUart_Printf("SBUS drive FAIL; retrying current stick command.\r\n");
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

static void sbus_mode_set_active(SbusRobotMode mode)
{
    s_sbus_active_mode = mode;
    s_sbus_entry_state = SBUS_ENTRY_ACTIVE;
    s_sbus_block_reason = SBUS_BLOCK_NONE;
}

static void sbus_motor_check_print(uint32_t now)
{
    if ((s_sbus_motor_check_last_ms != 0U) &&
        ((uint32_t)(now - s_sbus_motor_check_last_ms) < 1000U)) {
        return;
    }
    s_sbus_motor_check_last_ms = now;

    WheelDriveDiag wheel = {};
    WheelDrive_GetDiag(&wheel);
    uint8_t arm_online_mask = 0U;
    uint8_t arm_fault_mask = 0U;
    int32_t arm_temperature_deci_c[ARM_JOINT_COUNT] = {};
    for (uint8_t joint = 0U; joint < ARM_JOINT_COUNT; ++joint) {
        ArmMotorFeedback feedback = {};
        if (ArmMotor_GetFeedback(joint, &feedback) != 0U) {
            arm_online_mask |= (uint8_t)(1U << joint);
        }
        if (feedback.error != 0U) {
            arm_fault_mask |= (uint8_t)(1U << joint);
        }
        arm_temperature_deci_c[joint] = (int32_t)(feedback.temperature_c * 10.0f);
    }
    DebugUart_Printf("MOTOR_CHECK: leg feedback=0x%02X ready=0x%02X fault=0x%02X temp=N/A; wheel feedback=0x%X online=%u lock=%u hot=0x%X temp=%u/%u/%u/%uC; arm feedback=0x%X fault=0x%X temp=%ld.%01ld/%ld.%01ldC\r\n",
                     (unsigned)DogStand_GetOnlineMask(),
                     (unsigned)DogStand_GetReadyMask(),
                     (unsigned)DogStand_GetFaultMask(),
                     (unsigned)wheel.feedback_seen_mask,
                     (unsigned)wheel.all_online,
                     (unsigned)wheel.locked,
                     (unsigned)wheel.overtemp_mask,
                     (unsigned)wheel.motor[0].temperature_c,
                     (unsigned)wheel.motor[1].temperature_c,
                     (unsigned)wheel.motor[2].temperature_c,
                     (unsigned)wheel.motor[3].temperature_c,
                     (unsigned)arm_online_mask,
                     (unsigned)arm_fault_mask,
                     (long)(arm_temperature_deci_c[0] / 10),
                     (long)labs(arm_temperature_deci_c[0] % 10),
                     (long)(arm_temperature_deci_c[1] / 10),
                     (long)labs(arm_temperature_deci_c[1] % 10));
}

static void sbus_mode_transition(SbusRobotMode requested)
{
    const SbusRobotMode previous = s_sbus_requested_mode;
    const uint8_t gait_to_gait =
        (((previous == SBUS_MODE_GAIT_WHEEL) || (previous == SBUS_MODE_GAIT_ONLY)) &&
         ((requested == SBUS_MODE_GAIT_WHEEL) || (requested == SBUS_MODE_GAIT_ONLY))) ? 1U : 0U;
    s_sbus_requested_mode = requested;
    s_sbus_active_mode = SBUS_MODE_NONE;
    s_sbus_entry_state = SBUS_ENTRY_ENTERING;
    s_sbus_block_reason = SBUS_BLOCK_NONE;
    s_sbus_motor_check_last_ms = 0U;

    sbus_arm_leave();
    if ((s_sbus_lowering_pending != 0U) && (sbus_mode_is_low(requested) == 0U)) {
        dog_mit_lower_to_start_pose_cancel();
        s_sbus_lowering_pending = 0U;
        s_sbus_quad_standing = 0U;
    }
    if ((sbus_mode_is_mid(previous) != 0U) &&
        (sbus_mode_is_low(requested) != 0U) &&
        (s_sbus_lowering_pending == 0U)) {
        s_sbus_lowering_pending = 1U;
        s_sbus_quad_standing = 0U;
        (void)dog_mit_lower_to_start_pose_start();
    }
    if ((requested != SBUS_MODE_GAIT_WHEEL) &&
        (requested != SBUS_MODE_GAIT_ONLY)) {
        dog_mit_set_gait_wheel_contribution(0.0f);
        s_sbus_hybrid_wheel_degraded = 0U;
    }
    if (requested == SBUS_MODE_MOTOR_CHECK) {
        sbus_quad_stop_motion_immediate(0U);
        sbus_mechanical_cancel();
        ArmMotor_Disable();
        if (s_sbus_lowering_pending == 0U) {
            dog_debug_rx_only();
            s_mode = MODE_RX_ONLY;
        }
        sbus_quad_reset_state();
        sbus_wheel_disable(0U);
    } else {
        if (gait_to_gait == 0U) {
            sbus_wheel_hold();
        }
        if ((requested != SBUS_MODE_LOW_WHEEL) &&
            (requested != SBUS_MODE_USB_IDLE)) {
            sbus_mechanical_cancel();
        }
        if ((requested == SBUS_MODE_LOW_SAFE) ||
            (requested == SBUS_MODE_USB_IDLE)) {
            sbus_quad_stop_motion_immediate(0U);
            if (s_sbus_lowering_pending == 0U) {
                dog_debug_rx_only();
                s_mode = MODE_RX_ONLY;
            }
            sbus_quad_reset_state();
            sbus_wheel_hold();
        } else if (((previous == SBUS_MODE_GAIT_WHEEL) ||
                    (previous == SBUS_MODE_GAIT_ONLY)) &&
                   (requested != SBUS_MODE_GAIT_WHEEL) &&
                   (requested != SBUS_MODE_GAIT_ONLY) &&
                   (dog_mit_march_in_place_is_active() != 0U)) {
            sbus_quad_stop_motion(1U);
        }
    }

    DebugUart_Printf("SBUS mode request %s -> %s.\r\n",
                     sbus_robot_mode_name(previous),
                     sbus_robot_mode_name(requested));
}

static uint8_t sbus_mode_prepare_stand(uint32_t now)
{
    if (dog_mit_march_in_place_is_active() != 0U) {
        sbus_wheel_hold();
        sbus_quad_stop_motion(0U);
        s_sbus_active_mode = SBUS_MODE_NONE;
        s_sbus_entry_state = SBUS_ENTRY_WAIT_GAIT_STOP;
        s_sbus_block_reason = SBUS_BLOCK_GAIT_STOP;
        return 0U;
    }
    s_sbus_active_mode = SBUS_MODE_NONE;
    s_sbus_entry_state = SBUS_ENTRY_WAIT_STAND;
    s_sbus_block_reason = SBUS_BLOCK_STAND;
    return sbus_quad_ensure_stand(now);
}

static void sbus_mode_tick(SbusRobotMode requested, const SbusState *rc,
                           uint8_t changed, uint32_t now)
{
    if (requested != s_sbus_requested_mode) {
        sbus_mode_transition(requested);
    }

    if (DogSafety_IsLatched() == 0U) {
        (void)WheelDrive_TryClearLock();
    }

    if (s_sbus_lowering_pending != 0U) {
        if (requested == SBUS_MODE_MOTOR_CHECK) {
            sbus_wheel_disable(0U);
        } else {
            sbus_wheel_hold();
        }
        s_sbus_active_mode = SBUS_MODE_NONE;
        const uint8_t lower_state = dog_mit_lower_to_start_pose_state();
        if (lower_state == DOG_LOWER_ACTIVE) {
            s_sbus_entry_state = SBUS_ENTRY_WAIT_LOWER;
            s_sbus_block_reason = SBUS_BLOCK_NONE;
            return;
        }
        if (lower_state == DOG_LOWER_COMPLETE) {
            dog_mit_lower_to_start_pose_cancel();
            dog_debug_rx_only();
            s_mode = MODE_RX_ONLY;
            sbus_quad_reset_state();
            s_sbus_lowering_pending = 0U;
            DebugUart_Printf("SBUS MID->LOW controlled lowering complete.\r\n");
        } else {
            s_sbus_entry_state = SBUS_ENTRY_BLOCKED;
            s_sbus_block_reason = SBUS_BLOCK_LOWER;
            return;
        }
    }

    switch (requested) {
    case SBUS_MODE_MOTOR_CHECK:
        sbus_wheel_disable(0U);
        DogStand_ExitMechanicalLimitIdle();
        DogStand_ExitMechanicalLimitPose();
        WheelDrive_SetProfile(WHEEL_PROFILE_NORMAL);
        (void)DogStand_ClearDisable();
        (void)WheelDrive_TryClearLock();
        sbus_mechanical_prepare(rc, now);
        sbus_motor_check_print(now);
        sbus_mode_set_active(requested);
        break;

    case SBUS_MODE_LOW_WHEEL:
        WheelDrive_SetProfile(WHEEL_PROFILE_MECHANICAL_CRAWL);
        s_sbus_mechanical_prepare_since_ms = 0U;
        if ((DogStand_IsMechanicalLimitPose() == 0U) &&
            (s_sbus_mechanical_permit != 0U)) {
            s_sbus_mechanical_permit = 0U;
            sbus_quad_stop_motion_immediate(0U);
            sbus_quad_reset_state();
            if (DogStand_EnterMechanicalLimitPose() == 0U) {
                s_sbus_active_mode = SBUS_MODE_NONE;
                s_sbus_entry_state = SBUS_ENTRY_BLOCKED;
                s_sbus_block_reason = SBUS_BLOCK_LEG_FAULT;
                break;
            }
        }
        if (DogStand_IsMechanicalLimitPoseReady() != 0U) {
            if (sbus_wheel_update(rc) != 0U) {
                sbus_mode_set_active(requested);
            } else {
                s_sbus_active_mode = SBUS_MODE_NONE;
                s_sbus_entry_state = SBUS_ENTRY_BLOCKED;
                s_sbus_block_reason = SBUS_BLOCK_WHEEL_FAULT;
            }
        } else {
            sbus_wheel_hold();
            s_sbus_active_mode = SBUS_MODE_NONE;
            s_sbus_entry_state = SBUS_ENTRY_WAIT_MECHANICAL;
            s_sbus_block_reason = SBUS_BLOCK_MECHANICAL_PERMIT;
        }
        break;

    case SBUS_MODE_LOW_SAFE:
        WheelDrive_SetProfile(WHEEL_PROFILE_NORMAL);
        sbus_wheel_hold();
        if (WheelDrive_IsAvailable() != 0U) {
            sbus_mode_set_active(requested);
        } else {
            s_sbus_active_mode = SBUS_MODE_NONE;
            s_sbus_entry_state = SBUS_ENTRY_BLOCKED;
            s_sbus_block_reason = SBUS_BLOCK_WHEEL_FAULT;
        }
        break;

    case SBUS_MODE_USB_IDLE:
        WheelDrive_SetProfile(WHEEL_PROFILE_NORMAL);
        sbus_wheel_hold();
        sbus_mechanical_prepare(rc, now);
        sbus_mode_set_active(requested);
        break;

    case SBUS_MODE_STAND_HOLD:
    case SBUS_MODE_RESERVED_STAND:
        WheelDrive_SetProfile(WHEEL_PROFILE_NORMAL);
        sbus_wheel_hold();
        if (sbus_mode_prepare_stand(now) != 0U) {
            sbus_mode_set_active(requested);
        }
        break;

    case SBUS_MODE_STAND_WHEEL:
        WheelDrive_SetProfile(WHEEL_PROFILE_NORMAL);
        if (sbus_mode_prepare_stand(now) != 0U) {
            if (sbus_wheel_update(rc) != 0U) {
                sbus_mode_set_active(requested);
            } else {
                s_sbus_active_mode = SBUS_MODE_NONE;
                s_sbus_entry_state = SBUS_ENTRY_BLOCKED;
                s_sbus_block_reason = SBUS_BLOCK_WHEEL_FAULT;
            }
        } else {
            sbus_wheel_hold();
        }
        break;

    case SBUS_MODE_STAND_ARM:
        WheelDrive_SetProfile(WHEEL_PROFILE_NORMAL);
        sbus_wheel_hold();
        if (sbus_mode_prepare_stand(now) != 0U) {
            if (WheelDrive_IsAvailable() == 0U) {
                sbus_arm_leave();
                s_sbus_active_mode = SBUS_MODE_NONE;
                s_sbus_entry_state = SBUS_ENTRY_BLOCKED;
                s_sbus_block_reason = SBUS_BLOCK_WHEEL_FAULT;
                break;
            }
            sbus_arm_enter(now);
            sbus_arm_update(rc, now);
            if (s_sbus_arm_active != 0U) {
                sbus_mode_set_active(requested);
            } else {
                s_sbus_active_mode = SBUS_MODE_NONE;
                s_sbus_entry_state = SBUS_ENTRY_WAIT_ARM;
                s_sbus_block_reason = SBUS_BLOCK_ARM_FEEDBACK;
            }
        }
        break;

    case SBUS_MODE_GAIT_WHEEL:
    case SBUS_MODE_GAIT_ONLY:
    {
        const uint8_t wheel_drive_enabled =
            (requested == SBUS_MODE_GAIT_WHEEL) ? 1U : 0U;
        WheelDrive_SetProfile(WHEEL_PROFILE_NORMAL);
        if (dog_mit_fault_hold_is_active() != 0U) {
            sbus_wheel_hold();
            sbus_quad_stop_motion(0U);
            if (sbus_quad_ensure_stand(now) == 0U) {
                s_sbus_active_mode = SBUS_MODE_NONE;
                s_sbus_entry_state = SBUS_ENTRY_BLOCKED;
                s_sbus_block_reason = SBUS_BLOCK_LEG_FAULT;
                break;
            }
        }
        if ((dog_mit_march_in_place_is_active() == 0U) &&
            (sbus_quad_ensure_stand(now) == 0U)) {
            sbus_wheel_hold();
            s_sbus_entry_state = SBUS_ENTRY_WAIT_STAND;
            s_sbus_block_reason = SBUS_BLOCK_STAND;
            break;
        }

        SbusDriveInput drive = {};
        sbus_drive_input_from_sticks(rc, &drive);
        DogGaitSyncState pre_sync = {};
        dog_mit_get_gait_sync_state(&pre_sync);
        float requested_wheel_contribution = 0.0f;
        if (wheel_drive_enabled != 0U) {
            if (WheelDrive_IsAvailable() == 0U) {
                s_sbus_hybrid_wheel_degraded = 1U;
            } else if (s_sbus_hybrid_wheel_degraded != 0U) {
                if ((pre_sync.active == 0U) ||
                    (pre_sync.active_wheel_contribution <= 1.0e-3f)) {
                    s_sbus_hybrid_wheel_degraded = 0U;
                    requested_wheel_contribution = sbus_hybrid_wheel_contribution(
                        rc, drive.forward, drive.yaw);
                }
            } else {
                requested_wheel_contribution = sbus_hybrid_wheel_contribution(
                    rc, drive.forward, drive.yaw);
            }
        } else {
            s_sbus_hybrid_wheel_degraded = 0U;
        }
        dog_mit_set_gait_wheel_contribution(requested_wheel_contribution);

        if (drive.active != 0U) {
            (void)sbus_quad_drive_command(&drive, now);
        } else if (dog_mit_trot_march_is_active() != 0U) {
            sbus_quad_stop_motion(1U);
        } else if ((changed != 0U) && (s_sbus_main_prev == SBUS_SWITCH_MID)) {
            (void)sbus_quad_start_in_place(now);
        }

        DogGaitSyncState sync = {};
        dog_mit_get_gait_sync_state(&sync);
        if (dog_mit_march_in_place_is_active() != 0U) {
            if ((s_sbus_hybrid_wheel_degraded == 0U) &&
                (sync.active_wheel_contribution > 1.0e-3f)) {
                sbus_hybrid_wheel_update(rc, &sync);
            } else {
                sbus_wheel_hold();
            }
        } else {
            sbus_wheel_hold();
        }
        sbus_mode_set_active(requested);
        break;
    }

    default:
        sbus_wheel_hold();
        s_sbus_entry_state = SBUS_ENTRY_BLOCKED;
        break;
    }
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
    dog_mit_lower_to_start_pose_cancel();
    s_sbus_lowering_pending = 0U;
    sbus_mechanical_cancel();
    sbus_wheel_disable(1U);
    sbus_arm_leave();
    dog_debug_set_target(DOG_DEBUG_TARGET_ALL);
    DogSafety_SetSdEstop(1U);
    sbus_quad_reset_state();
    s_sbus_active_mode = SBUS_MODE_NONE;
    s_sbus_entry_state = SBUS_ENTRY_BLOCKED;
    s_sbus_block_reason = SBUS_BLOCK_NONE;
    s_sbus_safety_needs_clear = 1U;
    s_sbus_safety_recovery_since_ms = 0U;
    s_sbus_remote_lockout = 1U;
    s_sbus_remote_lockout_logged = 0U;
    s_mode = MODE_IDLE;
    DebugUart_Printf("SBUS CH9 safety: ESTOP latched; release CH9 and move main LOW to re-arm.\r\n");
}

static SbusLinkState sbus_link_state_update(const SbusState *rc, uint32_t now)
{
    if (rc == nullptr) {
        return SBUS_LINK_TRANSIENT;
    }

    const uint8_t have_frame = (rc->frame_count != 0U) ? 1U : 0U;
    const uint8_t frame_recent = ((have_frame != 0U) &&
        ((uint32_t)(now - rc->last_update_ms) <= SBUS_REMOTE_TIMEOUT_MS)) ? 1U : 0U;
    const uint8_t link_bad = ((frame_recent == 0U) || (rc->signal_lost != 0U) ||
                              (rc->failsafe != 0U)) ? 1U : 0U;

    if ((have_frame == 0U) &&
        ((uint32_t)(now - s_sbus_start_ms) < SBUS_REMOTE_TIMEOUT_MS)) {
        return SBUS_LINK_TRANSIENT;
    }

    if (link_bad != 0U) {
        s_sbus_link_good_frames = 0U;
        if (s_sbus_link_bad_since_ms == 0U) {
            s_sbus_link_bad_since_ms = (now != 0U) ? now : 1U;
        }
        return ((uint32_t)(now - s_sbus_link_bad_since_ms) >= SBUS_FAILSAFE_CONFIRM_MS) ?
            SBUS_LINK_LOST : SBUS_LINK_TRANSIENT;
    }

    if (s_sbus_link_bad_since_ms == 0U) {
        s_sbus_link_last_frame = rc->frame_count;
        s_sbus_link_good_frames = SBUS_RECOVERY_GOOD_FRAMES;
        return SBUS_LINK_GOOD;
    }

    if (rc->frame_count != s_sbus_link_last_frame) {
        s_sbus_link_last_frame = rc->frame_count;
        if (s_sbus_link_good_frames < SBUS_RECOVERY_GOOD_FRAMES) {
            s_sbus_link_good_frames++;
        }
    }
    if (s_sbus_link_good_frames < SBUS_RECOVERY_GOOD_FRAMES) {
        return SBUS_LINK_TRANSIENT;
    }

    s_sbus_link_bad_since_ms = 0U;
    return SBUS_LINK_GOOD;
}

static void sbus_remote_transient_inhibit(uint32_t now)
{
    Dog_Remote_Sample sample = {};
    sbus_wheel_hold();
    sbus_arm_leave();
    sbus_quad_stop_motion(0U);
    sample.tick_ms = now;
    DogRemote_Update(&sample);
}

static void sbus_remote_failsafe(uint32_t now)
{
    Dog_Remote_Sample sample = {};

    if (s_sbus_lost_since_ms == 0U) {
        s_sbus_lost_since_ms = now;
    }

    if (s_sbus_failsafe_stop_sent == 0U) {
        DebugUart_Printf("SBUS loss confirmed: holding healthy legs and stopping auxiliary outputs; recovery is automatic.\r\n");
        dog_mit_lower_to_start_pose_cancel();
        s_sbus_lowering_pending = 0U;
        dog_mit_protect_hold();
        s_sbus_mechanical_permit = 0U;
        s_sbus_mechanical_prepare_since_ms = 0U;
        sbus_wheel_hold();
        sbus_arm_leave();
        sbus_quad_reset_state();
        s_sbus_active_mode = SBUS_MODE_NONE;
        s_sbus_entry_state = SBUS_ENTRY_BLOCKED;
        s_sbus_block_reason = SBUS_BLOCK_NONE;
        s_mode = MODE_RX_ONLY;
        s_sbus_arm_active = 0U;
        s_sbus_arm_last_ms = 0U;
        s_sbus_failsafe_stop_sent = 1U;
        s_sbus_safety_recovery_since_ms = 0U;
    }

    s_sbus_seen_fresh = 0U;
    s_sbus_switch_valid = 0U;
    sample.estop_request = 0U;
    sample.tick_ms = now;
    DogRemote_Update(&sample);
}

static const char *control_source_name(ControlInputSource source)
{
    switch (source) {
    case CONTROL_SOURCE_SBUS: return "SBUS";
    case CONTROL_SOURCE_USB:  return "USB";
    default:                  return "NONE";
    }
}

static void usb_control_revoke(uint8_t hold)
{
    const uint8_t had_usb_control =
        ((s_usb_session_active != 0U) ||
         (s_control_source == CONTROL_SOURCE_USB)) ? 1U : 0U;
    if (s_usb_session_active != 0U) {
        s_usb_blocked_session_id = s_usb_session_id;
    }
    memset(&s_usb_applied_rc, 0, sizeof(s_usb_applied_rc));
    s_usb_session_active = 0U;
    s_usb_session_id = 0U;
    s_usb_last_counter = 0U;
    s_usb_last_generation = 0U;
    s_usb_last_accepted_ms = 0U;
    if (hold == 0U) {
        s_usb_release_hold = 0U;
    } else if (had_usb_control != 0U) {
        s_usb_release_hold = 1U;
    }
    s_control_source = (s_usb_release_hold != 0U) ?
        CONTROL_SOURCE_NONE : CONTROL_SOURCE_SBUS;
}

static uint8_t usb_counter_is_forward(uint32_t value, uint32_t previous)
{
    return ((int32_t)(value - previous) > 0) ? 1U : 0U;
}

static uint8_t usb_rc_is_safe_zero(const UsbVirtualRcSample *rc)
{
    if (rc == nullptr) {
        return 0U;
    }
    return ((rc->main_switch == 0U) && (rc->sub_switch == 0U) &&
            (rc->command_flags == 0U) &&
            (rc->yaw_permille == 0) && (rc->forward_permille == 0) &&
            (rc->speed_permille == -1000) &&
            (rc->arm_j0_permille == 0) && (rc->arm_j1_permille == 0)) ? 1U : 0U;
}

static uint8_t usb_physical_authorized(const SbusState *rc,
                                       uint8_t main_sw, uint8_t sub_sw)
{
    if ((rc == nullptr) || (main_sw != SBUS_SWITCH_LOW) ||
        (sub_sw != SBUS_SWITCH_LOW)) {
        return 0U;
    }
    const uint8_t channels[] = {0U, 1U, SBUS_ARM_J0_CH, SBUS_ARM_J1_CH};
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(channels) / sizeof(channels[0])); ++i) {
        const int16_t value = rc->norm[channels[i]];
        if ((value < -SBUS_MOVE_EXIT_DEADBAND) ||
            (value > SBUS_MOVE_EXIT_DEADBAND)) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t usb_control_refresh(const SbusState *physical_rc,
                                   uint8_t physical_main,
                                   uint8_t physical_sub,
                                   uint32_t now)
{
    if (usb_physical_authorized(physical_rc, physical_main, physical_sub) == 0U) {
        if ((s_usb_session_active != 0U) || (s_usb_release_hold != 0U)) {
            usb_control_revoke(0U);
        }
        return 0U;
    }

    UsbVirtualRcSample latest = {};
    if (UsbFrameProtocol_GetVirtualRc(&latest) == 0U) {
        return 0U;
    }

    if (s_usb_session_active == 0U) {
        if (((uint32_t)(now - latest.received_ms) <= USB_RC_TIMEOUT_MS) &&
            (latest.session_id != s_usb_blocked_session_id) &&
            (usb_rc_is_safe_zero(&latest) != 0U)) {
            s_usb_session_active = 1U;
            s_usb_release_hold = 0U;
            s_usb_session_id = latest.session_id;
            s_usb_last_counter = latest.command_counter;
            s_usb_last_generation = latest.generation;
            s_usb_last_accepted_ms = latest.received_ms;
            s_usb_applied_rc = latest;
            s_control_source = CONTROL_SOURCE_USB;
        } else {
            return 0U;
        }
    } else if ((latest.session_id == s_usb_session_id) &&
               (latest.generation != s_usb_last_generation)) {
        s_usb_last_generation = latest.generation;
        if (usb_counter_is_forward(latest.command_counter, s_usb_last_counter) != 0U) {
            s_usb_last_counter = latest.command_counter;
            s_usb_last_accepted_ms = latest.received_ms;
            s_usb_applied_rc = latest;
        }
    }

    if ((uint32_t)(now - s_usb_last_accepted_ms) > USB_RC_TIMEOUT_MS) {
        usb_control_revoke(1U);
        return 0U;
    }
    s_control_source = CONTROL_SOURCE_USB;
    return 1U;
}

static int16_t usb_permille_to_norm(int16_t value)
{
    const int32_t rounded = (value >= 0) ?
        ((int32_t)value + 5) / 10 : ((int32_t)value - 5) / 10;
    return (int16_t)rounded;
}

static SbusRobotMode usb_decode_robot_mode(uint8_t main_sw, uint8_t sub_sw)
{
    const uint8_t mode = (uint8_t)(main_sw * 3U + sub_sw);
    switch (mode) {
    case 1U: return SBUS_MODE_LOW_WHEEL;
    case 2U: return SBUS_MODE_LOW_SAFE;
    case 3U: return SBUS_MODE_STAND_HOLD;
    case 4U: return SBUS_MODE_STAND_WHEEL;
    case 5U: return SBUS_MODE_STAND_ARM;
    case 6U: return SBUS_MODE_GAIT_ONLY;
    case 7U: return SBUS_MODE_GAIT_WHEEL;
    case 8U: return SBUS_MODE_RESERVED_STAND;
    default: return SBUS_MODE_USB_IDLE;
    }
}

static void usb_build_control_view(const SbusState *physical_rc,
                                   SbusState *control_rc,
                                   uint8_t *main_sw,
                                   uint8_t *sub_sw,
                                   SbusRobotMode *requested_mode,
                                   uint8_t *virtual_mode)
{
    if ((physical_rc == nullptr) || (control_rc == nullptr) ||
        (main_sw == nullptr) || (sub_sw == nullptr) ||
        (requested_mode == nullptr) || (virtual_mode == nullptr)) {
        return;
    }

    *control_rc = *physical_rc;
    control_rc->norm[0U] = 0;
    control_rc->norm[1U] = 0;
    control_rc->norm[SBUS_SPEED_CH] = -100;
    control_rc->norm[SBUS_ARM_J0_CH] = 0;
    control_rc->norm[SBUS_ARM_J1_CH] = 0;

    const uint8_t motion_enable =
        ((s_usb_applied_rc.command_flags & USB_RC_FLAG_MOTION_ENABLE) != 0U) ? 1U : 0U;
    const uint8_t deadman =
        ((s_usb_applied_rc.command_flags & USB_RC_FLAG_DEADMAN_HELD) != 0U) ? 1U : 0U;
    const uint8_t smooth_stop =
        ((s_usb_applied_rc.command_flags & USB_RC_FLAG_SMOOTH_STOP) != 0U) ? 1U : 0U;

    if (motion_enable == 0U) {
        *main_sw = SBUS_SWITCH_LOW;
        *sub_sw = SBUS_SWITCH_LOW;
        *virtual_mode = 0U;
        *requested_mode = SBUS_MODE_USB_IDLE;
        return;
    }

    *main_sw = s_usb_applied_rc.main_switch;
    *sub_sw = s_usb_applied_rc.sub_switch;
    *virtual_mode = (uint8_t)(*main_sw * 3U + *sub_sw);
    *requested_mode = usb_decode_robot_mode(*main_sw, *sub_sw);
    control_rc->norm[SBUS_SPEED_CH] =
        usb_permille_to_norm(s_usb_applied_rc.speed_permille);

    if ((deadman != 0U) && (smooth_stop == 0U)) {
        control_rc->norm[0U] = usb_permille_to_norm(s_usb_applied_rc.yaw_permille);
        control_rc->norm[1U] = usb_permille_to_norm(s_usb_applied_rc.forward_permille);
        control_rc->norm[SBUS_ARM_J0_CH] =
            usb_permille_to_norm(s_usb_applied_rc.arm_j0_permille);
        control_rc->norm[SBUS_ARM_J1_CH] =
            usb_permille_to_norm(s_usb_applied_rc.arm_j1_permille);
    }
}

void control_task_safety_poll(void)
{
    SbusState rc = {};
    const uint32_t now = HAL_GetTick();

    Sbus_Process();
    (void)Sbus_GetState(&rc);
    const SbusLinkState link_state = sbus_link_state_update(&rc, now);
    if (link_state == SBUS_LINK_LOST) {
        sbus_remote_failsafe(now);
        return;
    }
    if (link_state == SBUS_LINK_TRANSIENT) {
        sbus_remote_transient_inhibit(now);
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
    SbusState physical_rc = {};
    Dog_Remote_Sample sample = {};
    const uint32_t now = HAL_GetTick();

    Sbus_Process();
    (void)Sbus_GetState(&physical_rc);

    const SbusLinkState link_state = sbus_link_state_update(&physical_rc, now);
    if (link_state == SBUS_LINK_LOST) {
        usb_control_revoke(1U);
        sbus_remote_failsafe(now);
        return;
    }
    if (link_state == SBUS_LINK_TRANSIENT) {
        usb_control_revoke(1U);
        sbus_remote_transient_inhibit(now);
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
    }

    const uint8_t physical_main = Sbus_Switch3(SBUS_MAIN_MODE_CH);
    const uint8_t physical_sub = Sbus_Switch3(SBUS_SUB_MODE_CH);
    const uint8_t safety_state = sbus_safety_update(&physical_rc);

    if (s_sbus_seen_fresh == 0U) {
        DebugUart_Printf("SBUS online main=%s sub=%s.\r\n",
                         sbus_switch_name(physical_main),
                         sbus_switch_name(physical_sub));
    }
    s_sbus_seen_fresh = 1U;

    sample.mode = (uint8_t)(physical_main * 3U + physical_sub);
    sample.tick_ms = now;

    if (safety_state != SBUS_SAFETY_RELEASED) {
        usb_control_revoke(1U);
        if ((safety_state == SBUS_SAFETY_TRIGGER) && (DogSafety_IsLatched() == 0U)) {
            sample.estop_request = 1U;
            DogRemote_Update(&sample);
            sbus_safety_trigger();
        } else if (safety_state == SBUS_SAFETY_TRIGGER) {
            DogSafety_SetSdEstop(1U);
        }
        s_sbus_safety_active = 1U;
        s_sbus_safety_recovery_since_ms = 0U;
        sbus_update_switch_state(physical_main, physical_sub);
        return;
    }
    DogSafety_SetSdEstop(0U);
    s_sbus_safety_active = 0U;

    if ((DogSafety_IsLatched() != 0U) && (s_sbus_safety_needs_clear == 0U)) {
        usb_control_revoke(1U);
        sbus_arm_leave();
        sbus_quad_reset_state();
        s_sbus_safety_needs_clear = 1U;
        s_sbus_safety_recovery_since_ms = 0U;
        s_sbus_remote_lockout = 1U;
        s_sbus_remote_lockout_logged = 0U;
        DebugUart_Printf("Motor safety latch active; release CH9 and move main LOW to recover.\r\n");
    }

    if ((DogStand_IsDisabled() != 0U) && (s_sbus_remote_lockout == 0U)) {
        usb_control_revoke(1U);
        sbus_arm_leave();
        sbus_quad_reset_state();
        s_sbus_remote_lockout = 1U;
        s_sbus_remote_lockout_logged = 0U;
        DebugUart_Printf("Motor control disabled; move main LOW before enabling again.\r\n");
    }

    if (s_sbus_remote_lockout != 0U) {
        usb_control_revoke(1U);
        const uint8_t low_low = ((physical_main == SBUS_SWITCH_LOW) &&
                                 (physical_sub == SBUS_SWITCH_LOW)) ? 1U : 0U;
        const uint8_t sticks_neutral =
            ((physical_rc.norm[0U] >= -SBUS_MOVE_EXIT_DEADBAND) &&
             (physical_rc.norm[0U] <= SBUS_MOVE_EXIT_DEADBAND) &&
             (physical_rc.norm[1U] >= -SBUS_MOVE_EXIT_DEADBAND) &&
             (physical_rc.norm[1U] <= SBUS_MOVE_EXIT_DEADBAND)) ? 1U : 0U;
        const uint8_t recovery_ready = (s_sbus_safety_needs_clear != 0U) ?
            low_low : sticks_neutral;
        if (recovery_ready != 0U) {
            if (s_sbus_safety_needs_clear != 0U) {
                if (s_sbus_safety_recovery_since_ms == 0U) {
                    s_sbus_safety_recovery_since_ms = now;
                }
                if ((uint32_t)(now - s_sbus_safety_recovery_since_ms) < SBUS_SAFETY_RECOVERY_MS) {
                    DogRemote_Update(&sample);
                    sbus_update_switch_state(physical_main, physical_sub);
                    return;
                }
                if (WheelDrive_TryClearLock() == 0U) {
                    DogRemote_Update(&sample);
                    sbus_update_switch_state(physical_main, physical_sub);
                    return;
                }
                (void)DogSafety_RequestRearm();
            }
            if (DogSafety_IsLatched() != 0U) {
                DogRemote_Update(&sample);
                sbus_update_switch_state(physical_main, physical_sub);
                return;
            }
            if (s_sbus_safety_needs_clear != 0U) {
                s_sbus_safety_needs_clear = 0U;
                s_sbus_safety_recovery_since_ms = 0U;
                DebugUart_Printf("SBUS safety latch confirmed clear in main LOW.\r\n");
            }
            s_sbus_remote_lockout = 0U;
            s_sbus_remote_lockout_logged = 0U;
            DebugUart_Printf("SBUS protection hold cleared with neutral sticks.\r\n");
        } else {
            s_sbus_safety_recovery_since_ms = 0U;
            if (s_sbus_remote_lockout_logged == 0U) {
                DebugUart_Printf((s_sbus_safety_needs_clear != 0U) ?
                    "SBUS ESTOP recovery: select LOW+LOW.\r\n" :
                    "SBUS protection hold: center CH1/CH2 before control resumes.\r\n");
                s_sbus_remote_lockout_logged = 1U;
            }
            DogRemote_Update(&sample);
            sbus_update_switch_state(physical_main, physical_sub);
            return;
        }
    }

    const ControlInputSource previous_source = s_control_source;
    const uint8_t usb_active = usb_control_refresh(
        &physical_rc, physical_main, physical_sub, now);
    if ((usb_active == 0U) && (s_usb_release_hold != 0U) &&
        (usb_physical_authorized(&physical_rc, physical_main, physical_sub) != 0U)) {
        s_control_source = CONTROL_SOURCE_NONE;
        sbus_remote_transient_inhibit(now);
        sbus_update_switch_state(physical_main, physical_sub);
        return;
    }

    SbusState control_rc = physical_rc;
    uint8_t control_main = physical_main;
    uint8_t control_sub = physical_sub;
    uint8_t control_mode = (uint8_t)(physical_main * 3U + physical_sub);
    SbusRobotMode requested_mode = sbus_decode_robot_mode(physical_main, physical_sub);
    if (usb_active != 0U) {
        usb_build_control_view(&physical_rc, &control_rc,
                               &control_main, &control_sub,
                               &requested_mode, &control_mode);
        s_control_source = CONTROL_SOURCE_USB;
    } else {
        s_control_source = CONTROL_SOURCE_SBUS;
    }

    const uint8_t changed = ((s_sbus_switch_valid == 0U) ||
                             (control_main != s_sbus_main_prev) ||
                             (control_sub != s_sbus_sub_prev) ||
                             (previous_source != s_control_source)) ? 1U : 0U;
    sbus_update_speed_profile(&control_rc, changed);
    sample.mode = control_mode;
    sbus_mode_tick(requested_mode, &control_rc, changed, now);

    DogRemote_Update(&sample);
    sbus_update_switch_state(control_main, control_sub);
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
            } else {
                WheelDriveDiag wheel_diag = {};
                WheelDrive_GetDiag(&wheel_diag);
                if ((wheel_diag.locked != 0U) || (wheel_diag.stopped == 0U)) {
                    DebugUart_Printf("Safety re-arm requires all four wheels stopped, then fresh SBUS main LOW.\r\n");
                } else if (DogSafety_RequestRearm() != 0U) {
                    DebugUart_Printf("Safety re-arm requested; wait for all 8 motors idle/fault-free.\r\n");
                } else {
                    DebugUart_Printf("Safety re-arm blocked by active CH9 inhibit.\r\n");
                }
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
        sbus_wheel_disable(1U);
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

static void usb_cdc_process_input(void)
{
    const uint32_t now = HAL_GetTick();
    UsbFrameProtocol_Tick(now);

    int ch;
    while ((ch = DebugUart_GetByte()) >= 0) {
        if (UsbFrameProtocol_FeedByte((uint8_t)ch, now) != 0U) {
            continue;
        }
#if USB_CDC_PRODUCTION_INTERFACE
        if ((ch == 'p') || (ch == 'Y') || (ch == 'y')) {
            handle_command((char)ch);
        }
#else
        if (VofaPid_IsEnabled() != 0U) {
            if (VofaPid_FeedRxByte((uint8_t)ch) != 0U) {
                continue;
            }
        }
        handle_command((char)ch);
#endif
    }
}

static void lf_periodic_status(void)
{
#if USB_CDC_PRODUCTION_INTERFACE
    return;
#else
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
#endif
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
    UsbFrameProtocol_Init();
    s_control_source = CONTROL_SOURCE_SBUS;
    s_usb_session_active = 0U;
    s_usb_release_hold = 0U;
    s_usb_session_id = 0U;
    s_usb_last_counter = 0U;
    s_usb_last_generation = 0U;
    s_usb_last_accepted_ms = 0U;
    s_usb_blocked_session_id = 0U;
    s_sbus_start_ms = HAL_GetTick();

#if USB_CDC_PRODUCTION_INTERFACE
    DebugUart_SetLogVerbose(0U);
    DebugUart_SetCanRxVerbose(0U);
    s_can_rx_log_enabled = 0U;
#endif

    control_init_wait(1500U, 1U);
    control_init_wait(500U, 0U);

#if !USB_CDC_PRODUCTION_INTERFACE
    print_help();
#endif
    enter_rx_only();
}

void control_task(void)
{
    usb_cdc_process_input();
    update_target();
    lf_periodic_status();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    (void)htim;
}
