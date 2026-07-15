#include "obstacle_task.h"

#include "debug_uart.h"
#include "motor_task.h"
#include "obstacle_config.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

static_assert(DOG_OBSTACLE_BRIDGE_ACTIVE_GAP_MM > 0.0f,
              "Bridge B active gap must be positive");
static_assert(DOG_OBSTACLE_BRIDGE_ACTIVE_GAP_MM <=
                  DOG_OBSTACLE_BRIDGE_TEST_GAP_LIMIT_MM,
              "Bridge B active gap exceeds the approved test limit");
static_assert(DOG_OBSTACLE_BRIDGE_TEST_GAP_LIMIT_MM <=
                  DOG_OBSTACLE_BRIDGE_FINAL_GAP_MM,
              "Bridge B test limit exceeds the formal gap");
static_assert(DOG_OBSTACLE_STAIR_STEP_HEIGHT_MM > 0.0f,
              "Stair height must be positive");
static_assert(DOG_OBSTACLE_STAIR_TREAD_MM >
                  (2.0f * DOG_OBSTACLE_STAIR_EDGE_MARGIN_MM),
              "Stair tread is too short for the wheel edge margins");
static_assert(DOG_OBSTACLE_STAIR_LEVEL_COUNT == 3U,
              "The current T-stair sequence is designed for three levels");
static_assert(DOG_OBSTACLE_STAIR_TREAD_MM *
                  DOG_OBSTACLE_STAIR_LEVEL_COUNT ==
                  DOG_OBSTACLE_STAIR_RUN_MM,
              "Three stair treads must fill the 900 mm stair run");
static_assert((2.0f * DOG_OBSTACLE_STAIR_RUN_MM) +
                  DOG_OBSTACLE_STAIR_TOP_PLATFORM_MM ==
                  DOG_OBSTACLE_STAIR_TOTAL_LENGTH_MM,
              "Stair runs and top platform must match the total length");
static_assert(DOG_OBSTACLE_STAIR_WIDTH_MM > 0.0f,
              "Stair flight width must be positive");
static_assert(DOG_OBSTACLE_STAIR_FRONT_FORWARD_X_MM -
                  (-DOG_OBSTACLE_STAIR_COMPACT_X_MM) ==
                  DOG_OBSTACLE_STAIR_TREAD_MM,
              "Front stair stride must equal the tread depth");
static_assert(DOG_OBSTACLE_STAIR_COMPACT_X_MM -
                  DOG_OBSTACLE_STAIR_REAR_SHIFT_X_MM ==
                  DOG_OBSTACLE_STAIR_TREAD_MM,
              "Rear stair stride must equal the tread depth");
static_assert(DOG_OBSTACLE_WHEELBASE_MM -
                  (2.0f * DOG_OBSTACLE_STAIR_COMPACT_X_MM) ==
                  DOG_OBSTACLE_STAIR_TREAD_MM -
                  (2.0f * DOG_OBSTACLE_STAIR_EDGE_MARGIN_MM),
              "Compact stair stance must preserve both edge margins");

static constexpr uint8_t kFrontLegMask =
    (uint8_t)((1U << DOG_LEG_LF) | (1U << DOG_LEG_RF));
static constexpr uint8_t kRearLegMask =
    (uint8_t)((1U << DOG_LEG_LB) | (1U << DOG_LEG_RB));
static constexpr uint8_t kLeftFrontLegMask = (uint8_t)(1U << DOG_LEG_LF);
static constexpr uint8_t kRightFrontLegMask = (uint8_t)(1U << DOG_LEG_RF);
static constexpr uint8_t kLeftRearLegMask = (uint8_t)(1U << DOG_LEG_LB);
static constexpr uint8_t kRightRearLegMask = (uint8_t)(1U << DOG_LEG_RB);
static constexpr uint8_t kCheckpointUnprepared = 0xFFU;

enum BridgeMotionKind {
    BRIDGE_MOTION_NONE = 0U,
    BRIDGE_MOTION_PREPARE,
    BRIDGE_MOTION_CHECKPOINT,
    BRIDGE_MOTION_UNPREPARE,
    BRIDGE_MOTION_RECOVER,
};

static volatile uint8_t s_requested_mode = 0U;
static volatile uint8_t s_requested_selection = DOG_OBSTACLE_BRIDGE_B;
static volatile int8_t s_requested_step = 0;
static volatile uint8_t s_requested_safety_abort = 0U;
static volatile uint32_t s_request_epoch = 0U;

static DogObstacleStatus s_status = {};
static DogObstacleStatus s_published_status = {};
static Dog_Foot_Target s_checkpoint[5U][DOG_LEG_COUNT] = {};
static Dog_Foot_Target s_normal_stand[DOG_LEG_COUNT] = {};
static uint8_t s_geometry_ready = 0U;
static uint8_t s_motion_kind = BRIDGE_MOTION_NONE;
static uint8_t s_motion_source_checkpoint = kCheckpointUnprepared;
static int8_t s_motion_direction = 0;
static uint8_t s_recovery_leg_mask = DOG_LEG_MASK_ALL;
static float s_recovery_clearance_mm = 0.0f;
static uint32_t s_recovery_duration_ms = 0U;
static uint8_t s_bridge_exit_recovery_attempted = 0U;

static constexpr uint8_t kStairWaypointCount =
    (uint8_t)(DOG_STAIR_PHASE_TOP_READY + 1U);
static constexpr uint8_t kStairMaxSegments = 24U;
static_assert(kStairMaxSegments >= 22U,
              "Stair sequence capacity is too small for single-leg top entry");

struct StairSegment {
    uint8_t source_phase;
    uint8_t target_phase;
    uint8_t leg_mask;
    uint32_t duration_ms;
    float clearance_mm;
    uint32_t dwell_ms;
};

struct StairContext {
    Dog_Foot_Target waypoint[kStairWaypointCount][DOG_LEG_COUNT];
    StairSegment sequence[kStairMaxSegments];
    uint8_t geometry_ready;
    uint8_t current_phase;
    uint8_t motion_source_phase;
    uint8_t motion_target_phase;
    uint8_t sequence_count;
    uint8_t sequence_index;
    uint8_t sequence_recovering;
    uint8_t motion_active;
    uint8_t failed_segment_active;
    uint8_t paused;
    uint8_t recovery_goal_phase;
    uint32_t dwell_started_ms;
    uint32_t dwell_duration_ms;
};

static StairContext s_stair = {};

static float stair_front_normal_z(void)
{
    return DOG_STAND_FOOT_Z_MM;
}

static float stair_rear_normal_z(void)
{
    return DOG_STAND_FOOT_Z_MM + DOG_REAR_FOOT_EXTRA_Z_MM;
}

static void stair_set_waypoint(uint8_t phase, float front_x, float front_z,
                               float rear_x, float rear_z)
{
    if (phase >= kStairWaypointCount) {
        return;
    }
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        const uint8_t rear = ((leg == DOG_LEG_LB) ||
                              (leg == DOG_LEG_RB)) ? 1U : 0U;
        s_stair.waypoint[phase][leg].x_mm = (rear != 0U) ? rear_x : front_x;
        s_stair.waypoint[phase][leg].z_mm = (rear != 0U) ? rear_z : front_z;
    }
}

static void stair_copy_waypoint(uint8_t target_phase, uint8_t source_phase)
{
    if ((target_phase >= kStairWaypointCount) ||
        (source_phase >= kStairWaypointCount)) {
        return;
    }
    memcpy(s_stair.waypoint[target_phase], s_stair.waypoint[source_phase],
           sizeof(s_stair.waypoint[target_phase]));
}

static void stair_set_leg_waypoint(uint8_t phase, uint8_t leg,
                                   float x_mm, float z_mm)
{
    if ((phase >= kStairWaypointCount) || (leg >= DOG_LEG_COUNT)) {
        return;
    }
    s_stair.waypoint[phase][leg].x_mm = x_mm;
    s_stair.waypoint[phase][leg].z_mm = z_mm;
}

static void stair_build_geometry(void)
{
    const float front_z = stair_front_normal_z();
    const float rear_z = stair_rear_normal_z();
    const float front_upper_z = front_z - DOG_OBSTACLE_STAIR_STEP_HEIGHT_MM;
    const float rear_upper_z = rear_z - DOG_OBSTACLE_STAIR_STEP_HEIGHT_MM;
    const float front_lift_z = front_upper_z - DOG_OBSTACLE_STAIR_CLEARANCE_MM;
    const float rear_lift_z = rear_upper_z - DOG_OBSTACLE_STAIR_CLEARANCE_MM;
    const float compact = DOG_OBSTACLE_STAIR_COMPACT_X_MM;
    const float front_forward = DOG_OBSTACLE_STAIR_FRONT_FORWARD_X_MM;
    const float rear_shift = DOG_OBSTACLE_STAIR_REAR_SHIFT_X_MM;

    stair_set_waypoint(DOG_STAIR_PHASE_IDLE, 0.0f, front_z, 0.0f, rear_z);
    stair_set_waypoint(DOG_STAIR_PHASE_PREP_BODY_SHIFT,
                       -compact, front_z, -compact, rear_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_PREP_LEFT_REAR_COMPACT,
                        DOG_STAIR_PHASE_PREP_BODY_SHIFT);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_PREP_LEFT_REAR_COMPACT,
                           DOG_LEG_LB, compact, rear_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_PREP_REAR_COMPACT,
                        DOG_STAIR_PHASE_PREP_LEFT_REAR_COMPACT);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_PREP_REAR_COMPACT,
                           DOG_LEG_RB, compact, rear_z);
    stair_set_waypoint(DOG_STAIR_PHASE_LEVEL_READY,
                       -compact, front_z, compact, rear_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_LEFT_FRONT_LIFT,
                        DOG_STAIR_PHASE_LEVEL_READY);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_LEFT_FRONT_LIFT,
                           DOG_LEG_LF, -compact, front_lift_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_LEFT_FRONT_FORWARD,
                        DOG_STAIR_PHASE_LEFT_FRONT_LIFT);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_LEFT_FRONT_FORWARD,
                           DOG_LEG_LF, front_forward, front_lift_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_LEFT_FRONT_LAND,
                        DOG_STAIR_PHASE_LEFT_FRONT_FORWARD);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_LEFT_FRONT_LAND,
                           DOG_LEG_LF, front_forward, front_upper_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_RIGHT_FRONT_LIFT,
                        DOG_STAIR_PHASE_LEFT_FRONT_LAND);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_RIGHT_FRONT_LIFT,
                           DOG_LEG_RF, -compact, front_lift_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_RIGHT_FRONT_FORWARD,
                        DOG_STAIR_PHASE_RIGHT_FRONT_LIFT);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_RIGHT_FRONT_FORWARD,
                           DOG_LEG_RF, front_forward, front_lift_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_RIGHT_FRONT_LAND,
                        DOG_STAIR_PHASE_RIGHT_FRONT_FORWARD);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_RIGHT_FRONT_LAND,
                           DOG_LEG_RF, front_forward, front_upper_z);
    stair_set_waypoint(DOG_STAIR_PHASE_BODY_SHIFT,
                       -compact, front_upper_z, rear_shift, rear_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_LEFT_REAR_LIFT,
                        DOG_STAIR_PHASE_BODY_SHIFT);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_LEFT_REAR_LIFT,
                           DOG_LEG_LB, rear_shift, rear_lift_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_LEFT_REAR_FORWARD,
                        DOG_STAIR_PHASE_LEFT_REAR_LIFT);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_LEFT_REAR_FORWARD,
                           DOG_LEG_LB, compact, rear_lift_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_LEFT_REAR_LAND,
                        DOG_STAIR_PHASE_LEFT_REAR_FORWARD);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_LEFT_REAR_LAND,
                           DOG_LEG_LB, compact, rear_upper_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_RIGHT_REAR_LIFT,
                        DOG_STAIR_PHASE_LEFT_REAR_LAND);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_RIGHT_REAR_LIFT,
                           DOG_LEG_RB, rear_shift, rear_lift_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_RIGHT_REAR_FORWARD,
                        DOG_STAIR_PHASE_RIGHT_REAR_LIFT);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_RIGHT_REAR_FORWARD,
                           DOG_LEG_RB, compact, rear_lift_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_RIGHT_REAR_LAND,
                        DOG_STAIR_PHASE_RIGHT_REAR_FORWARD);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_RIGHT_REAR_LAND,
                           DOG_LEG_RB, compact, rear_upper_z);
    stair_set_waypoint(DOG_STAIR_PHASE_BODY_RAISE,
                       -compact, front_z, compact, rear_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_TOP_LEFT_FRONT_ADVANCE,
                        DOG_STAIR_PHASE_BODY_RAISE);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_TOP_LEFT_FRONT_ADVANCE,
                           DOG_LEG_LF, front_forward, front_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_TOP_RIGHT_FRONT_ADVANCE,
                        DOG_STAIR_PHASE_TOP_LEFT_FRONT_ADVANCE);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_TOP_RIGHT_FRONT_ADVANCE,
                           DOG_LEG_RF, front_forward, front_z);
    stair_set_waypoint(DOG_STAIR_PHASE_TOP_BODY_SHIFT,
                       -compact, front_z, rear_shift, rear_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_TOP_LEFT_REAR_ADVANCE,
                        DOG_STAIR_PHASE_TOP_BODY_SHIFT);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_TOP_LEFT_REAR_ADVANCE,
                           DOG_LEG_LB, compact, rear_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_TOP_RIGHT_REAR_ADVANCE,
                        DOG_STAIR_PHASE_TOP_LEFT_REAR_ADVANCE);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_TOP_RIGHT_REAR_ADVANCE,
                           DOG_LEG_RB, compact, rear_z);
    stair_set_waypoint(DOG_STAIR_PHASE_TOP_BODY_NORMALIZE,
                       -(2.0f * compact), front_z, 0.0f, rear_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_TOP_LEFT_FRONT_NORMAL,
                        DOG_STAIR_PHASE_TOP_BODY_NORMALIZE);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_TOP_LEFT_FRONT_NORMAL,
                           DOG_LEG_LF, 0.0f, front_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_TOP_RIGHT_FRONT_NORMAL,
                        DOG_STAIR_PHASE_TOP_LEFT_FRONT_NORMAL);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_TOP_RIGHT_FRONT_NORMAL,
                           DOG_LEG_RF, 0.0f, front_z);
    stair_set_waypoint(DOG_STAIR_PHASE_TOP_READY,
                       0.0f, front_z, 0.0f, rear_z);
    s_stair.geometry_ready = 1U;
}

static void stair_context_reset(void)
{
    memset(&s_stair, 0, sizeof(s_stair));
    stair_build_geometry();
    s_stair.current_phase = DOG_STAIR_PHASE_IDLE;
    s_stair.motion_source_phase = DOG_STAIR_PHASE_IDLE;
    s_stair.motion_target_phase = DOG_STAIR_PHASE_IDLE;
    s_stair.recovery_goal_phase = DOG_STAIR_PHASE_IDLE;
}

static void obstacle_publish_status(void)
{
    taskENTER_CRITICAL();
    s_published_status = s_status;
    taskEXIT_CRITICAL();
}

struct ObstacleStatusPublishGuard {
    ~ObstacleStatusPublishGuard()
    {
        obstacle_publish_status();
    }
};

static float bridge_front_z(void)
{
    return DOG_STAND_FOOT_Z_MM - DOG_OBSTACLE_BRIDGE_BODY_LOWER_MM;
}

static float bridge_rear_z(void)
{
    return DOG_STAND_FOOT_Z_MM + DOG_REAR_FOOT_EXTRA_Z_MM -
           DOG_OBSTACLE_BRIDGE_BODY_LOWER_MM;
}

static void status_write_begin(void)
{
    taskENTER_CRITICAL();
}

static void status_write_end(void)
{
    taskEXIT_CRITICAL();
}

static uint8_t obstacle_reserve_motion_start(uint32_t request_epoch)
{
    uint8_t reserved = 0U;
    taskENTER_CRITICAL();
    if ((s_requested_mode != 0U) && (s_request_epoch == request_epoch)) {
        s_status.can_exit = 0U;
        s_published_status = s_status;
        reserved = 1U;
    }
    taskEXIT_CRITICAL();
    return reserved;
}

static void obstacle_set_state(uint8_t state)
{
    if (s_status.state == state) {
        return;
    }
    s_status.state = state;
    DebugUart_Printf("OBSTACLE state=%s selected=%s cp=%u gaps=%u.\r\n",
                     DogObstacle_StateName(state),
                     DogObstacle_TypeName(s_status.selected),
                     (unsigned)s_status.checkpoint,
                     (unsigned)s_status.completed_gaps);
}

static void obstacle_fault(uint8_t fault)
{
    if ((s_status.state == DOG_OBSTACLE_FAULT) &&
        (s_status.fault == fault) &&
        (s_status.motion_state == DOG_FOOT_MOTION_FAULT)) {
        return;
    }
    dog_foot_motion_cancel();
    if (s_status.selected == DOG_OBSTACLE_STAIRS) {
        if ((s_stair.motion_active != 0U) ||
            ((s_stair.sequence_count != 0U) &&
             (s_stair.sequence_index < s_stair.sequence_count))) {
            s_stair.failed_segment_active = 1U;
        }
        s_stair.motion_active = 0U;
        s_stair.paused = 0U;
        s_stair.dwell_started_ms = 0U;
        s_stair.dwell_duration_ms = 0U;
        s_status.phase = s_stair.current_phase;
        s_status.checkpoint = s_stair.current_phase;
    }
    s_status.fault = fault;
    s_status.motion_state = DOG_FOOT_MOTION_FAULT;
    if (s_status.selected == DOG_OBSTACLE_STAIRS) {
        s_status.can_exit = ((s_stair.current_phase == DOG_STAIR_PHASE_IDLE) &&
                             (s_stair.sequence_count == 0U)) ? 1U : 0U;
    } else {
        s_status.can_exit = ((s_status.prepared == 0U) &&
                             (s_motion_kind == BRIDGE_MOTION_NONE)) ? 1U : 0U;
    }
    obstacle_set_state(DOG_OBSTACLE_FAULT);
}

static void obstacle_control_abort(uint8_t fault)
{
    if ((s_status.state == DOG_OBSTACLE_FAULT) &&
        (s_status.fault == fault) &&
        (s_motion_kind == BRIDGE_MOTION_NONE) &&
        (s_status.prepared == 0U)) {
        s_status.can_exit = 1U;
        return;
    }
    dog_foot_motion_cancel();
    s_status.fault = fault;
    s_status.motion_state = DOG_FOOT_MOTION_FAULT;
    s_status.prepared = 0U;
    s_status.checkpoint = kCheckpointUnprepared;
    s_status.target_checkpoint = kCheckpointUnprepared;
    s_status.can_exit = 1U;
    s_motion_kind = BRIDGE_MOTION_NONE;
    s_motion_source_checkpoint = kCheckpointUnprepared;
    s_motion_direction = 0;
    s_recovery_leg_mask = DOG_LEG_MASK_ALL;
    s_recovery_clearance_mm = 0.0f;
    s_recovery_duration_ms = 0U;
    s_bridge_exit_recovery_attempted = 0U;
    stair_context_reset();
    s_status.phase = DOG_STAIR_PHASE_IDLE;
    s_status.completed_levels = 0U;
    obstacle_set_state(DOG_OBSTACLE_FAULT);
}

static void obstacle_reset_selection(uint8_t selection)
{
    dog_foot_motion_cancel();
    s_status.selected = selection;
    s_status.fault = DOG_OBSTACLE_FAULT_NONE;
    s_status.prepared = 0U;
    s_status.checkpoint = kCheckpointUnprepared;
    s_status.target_checkpoint = kCheckpointUnprepared;
    s_status.motion_state = DOG_FOOT_MOTION_IDLE;
    s_status.can_exit = 1U;
    s_status.completed_gaps = 0U;
    s_status.phase = DOG_STAIR_PHASE_IDLE;
    s_status.completed_levels = 0U;
    s_motion_kind = BRIDGE_MOTION_NONE;
    s_motion_source_checkpoint = kCheckpointUnprepared;
    s_motion_direction = 0;
    s_recovery_leg_mask = DOG_LEG_MASK_ALL;
    s_recovery_clearance_mm = 0.0f;
    s_recovery_duration_ms = 0U;
    s_bridge_exit_recovery_attempted = 0U;
    stair_context_reset();
    obstacle_set_state(DOG_OBSTACLE_DISABLED);
}

static void bridge_build_geometry(void)
{
    const float gap = DOG_OBSTACLE_BRIDGE_ACTIVE_GAP_MM;
    const float pitch = DOG_OBSTACLE_BRIDGE_BOARD_MM + gap;
    const float stance_delta = pitch - DOG_OBSTACLE_WHEELBASE_MM;
    const float first_shift = pitch * 0.5f;
    const float second_shift = pitch - first_shift;
    const float stable_front_x = stance_delta * 0.5f;
    const float stable_rear_x = -stance_delta * 0.5f;

    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        const uint8_t rear = ((leg == DOG_LEG_LB) || (leg == DOG_LEG_RB)) ? 1U : 0U;
        const float stable_x = (rear != 0U) ? stable_rear_x : stable_front_x;
        const float z_mm = (rear != 0U) ? bridge_rear_z() : bridge_front_z();
        s_normal_stand[leg].x_mm = DOG_STAND_FOOT_X_MM;
        s_normal_stand[leg].z_mm = (rear != 0U) ?
            (DOG_STAND_FOOT_Z_MM + DOG_REAR_FOOT_EXTRA_Z_MM) : DOG_STAND_FOOT_Z_MM;
        s_checkpoint[0U][leg] = {stable_x, z_mm};
        s_checkpoint[1U][leg] = {stable_x - first_shift, z_mm};
        s_checkpoint[2U][leg] = s_checkpoint[1U][leg];
        s_checkpoint[3U][leg] = s_checkpoint[2U][leg];
        if (rear == 0U) {
            s_checkpoint[2U][leg].x_mm += pitch;
            s_checkpoint[3U][leg].x_mm += pitch;
        } else {
            s_checkpoint[3U][leg].x_mm += pitch;
        }
        s_checkpoint[4U][leg] = {
            s_checkpoint[3U][leg].x_mm - second_shift,
            z_mm,
        };
    }
    s_geometry_ready = 1U;
}

static uint8_t bridge_point_valid(uint8_t leg, float x_mm, float z_mm)
{
    return dog_leg_foot_xz_to_motor_deg(leg, x_mm, z_mm, nullptr, nullptr);
}

static uint8_t bridge_geometry_valid(void)
{
    if ((DOG_OBSTACLE_BRIDGE_ACTIVE_GAP_MM <= 0.0f) ||
        (DOG_OBSTACLE_BRIDGE_ACTIVE_GAP_MM > DOG_OBSTACLE_BRIDGE_FINAL_GAP_MM) ||
        (DOG_OBSTACLE_BRIDGE_BOARD_MM <= 0.0f) ||
        (DOG_OBSTACLE_WHEELBASE_MM <= 0.0f) ||
        (DOG_OBSTACLE_WHEEL_AXLE_HEIGHT_MM <= 0.0f)) {
        return 0U;
    }
    if (s_geometry_ready == 0U) {
        bridge_build_geometry();
    }
    for (uint8_t cp = 0U; cp < 5U; ++cp) {
        for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
            if (bridge_point_valid(leg, s_checkpoint[cp][leg].x_mm,
                                   s_checkpoint[cp][leg].z_mm) == 0U) {
                return 0U;
            }
        }
    }
    for (uint8_t leg = 0U; leg < DOG_LEG_COUNT; ++leg) {
        const uint8_t rear = ((leg == DOG_LEG_LB) || (leg == DOG_LEG_RB)) ? 1U : 0U;
        const uint8_t from_cp = (rear != 0U) ? 2U : 1U;
        const uint8_t to_cp = (rear != 0U) ? 3U : 2U;
        const float apex_x = 0.5f *
            (s_checkpoint[from_cp][leg].x_mm + s_checkpoint[to_cp][leg].x_mm);
        const float apex_z = 0.5f *
            (s_checkpoint[from_cp][leg].z_mm + s_checkpoint[to_cp][leg].z_mm) -
            DOG_OBSTACLE_BRIDGE_SWING_CLEARANCE_MM;
        if (bridge_point_valid(leg, apex_x, apex_z) == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t bridge_transition_mask(uint8_t from_cp, uint8_t to_cp)
{
    if (((from_cp == 1U) && (to_cp == 2U)) ||
        ((from_cp == 2U) && (to_cp == 1U))) {
        return kFrontLegMask;
    }
    if (((from_cp == 2U) && (to_cp == 3U)) ||
        ((from_cp == 3U) && (to_cp == 2U))) {
        return kRearLegMask;
    }
    return DOG_LEG_MASK_ALL;
}

static float bridge_transition_clearance(uint8_t from_cp, uint8_t to_cp)
{
    return (bridge_transition_mask(from_cp, to_cp) == DOG_LEG_MASK_ALL) ? 0.0f :
        DOG_OBSTACLE_BRIDGE_SWING_CLEARANCE_MM;
}

static uint32_t bridge_transition_duration(uint8_t from_cp, uint8_t to_cp)
{
    return (bridge_transition_mask(from_cp, to_cp) == DOG_LEG_MASK_ALL) ?
        DOG_OBSTACLE_BRIDGE_SHIFT_MS : DOG_OBSTACLE_BRIDGE_SWING_MS;
}

static uint8_t bridge_start_motion(uint8_t from_cp, uint8_t to_cp,
                                   uint8_t kind, int8_t direction)
{
    const uint8_t mask = bridge_transition_mask(from_cp, to_cp);
    const float clearance = bridge_transition_clearance(from_cp, to_cp);
    const uint32_t duration = bridge_transition_duration(from_cp, to_cp);
    if (dog_foot_motion_start(mask, s_checkpoint[to_cp], clearance, duration,
                              duration + DOG_OBSTACLE_BRIDGE_TIMEOUT_MARGIN_MS) == 0U) {
        obstacle_fault(DOG_OBSTACLE_FAULT_MOTION_START);
        return 0U;
    }
    s_motion_kind = kind;
    s_motion_source_checkpoint = from_cp;
    s_motion_direction = direction;
    s_status.target_checkpoint = to_cp;
    s_status.motion_state = DOG_FOOT_MOTION_ACTIVE;
    s_status.can_exit = 0U;
    obstacle_set_state((direction < 0) ? DOG_OBSTACLE_ROLLBACK : DOG_OBSTACLE_MOVING);
    return 1U;
}

static uint8_t bridge_start_prepare(void)
{
    const uint32_t duration = DOG_OBSTACLE_BRIDGE_PREPARE_MS;
    if (dog_foot_motion_start(DOG_LEG_MASK_ALL, s_checkpoint[0U], 0.0f, duration,
                              duration + DOG_OBSTACLE_BRIDGE_TIMEOUT_MARGIN_MS) == 0U) {
        obstacle_fault(DOG_OBSTACLE_FAULT_MOTION_START);
        return 0U;
    }
    s_motion_kind = BRIDGE_MOTION_PREPARE;
    s_motion_source_checkpoint = kCheckpointUnprepared;
    s_motion_direction = 1;
    s_status.target_checkpoint = 0U;
    s_status.motion_state = DOG_FOOT_MOTION_ACTIVE;
    s_status.can_exit = 0U;
    obstacle_set_state(DOG_OBSTACLE_MOVING);
    return 1U;
}

static uint8_t bridge_start_unprepare(void)
{
    const uint32_t duration = DOG_OBSTACLE_BRIDGE_PREPARE_MS;
    if (dog_foot_motion_start(DOG_LEG_MASK_ALL, s_normal_stand, 0.0f, duration,
                              duration + DOG_OBSTACLE_BRIDGE_TIMEOUT_MARGIN_MS) == 0U) {
        obstacle_fault(DOG_OBSTACLE_FAULT_MOTION_START);
        return 0U;
    }
    s_motion_kind = BRIDGE_MOTION_UNPREPARE;
    s_motion_source_checkpoint = s_status.checkpoint;
    s_motion_direction = -1;
    s_status.target_checkpoint = kCheckpointUnprepared;
    s_status.motion_state = DOG_FOOT_MOTION_ACTIVE;
    s_status.can_exit = 0U;
    obstacle_set_state(DOG_OBSTACLE_ROLLBACK);
    return 1U;
}

static uint8_t bridge_start_recovery(uint8_t checkpoint, uint8_t leg_mask,
                                     float clearance_mm, uint32_t duration_ms)
{
    if (checkpoint >= 5U) {
        return 0U;
    }
    if (dog_foot_motion_start(leg_mask, s_checkpoint[checkpoint], clearance_mm,
                              duration_ms,
                              duration_ms + DOG_OBSTACLE_BRIDGE_TIMEOUT_MARGIN_MS) == 0U) {
        return 0U;
    }
    s_motion_kind = BRIDGE_MOTION_RECOVER;
    s_motion_source_checkpoint = checkpoint;
    s_motion_direction = -1;
    s_recovery_leg_mask = leg_mask;
    s_recovery_clearance_mm = clearance_mm;
    s_recovery_duration_ms = duration_ms;
    s_status.target_checkpoint = checkpoint;
    s_status.motion_state = DOG_FOOT_MOTION_ACTIVE;
    s_status.can_exit = 0U;
    obstacle_set_state(DOG_OBSTACLE_ROLLBACK);
    return 1U;
}

static uint8_t bridge_recover_interrupted_motion(void)
{
    const uint8_t interrupted_kind = s_motion_kind;
    const uint8_t source_checkpoint = s_motion_source_checkpoint;
    const uint8_t target_checkpoint = s_status.target_checkpoint;

    if (interrupted_kind == BRIDGE_MOTION_NONE) {
        return 1U;
    }
    if (dog_foot_motion_prepare() == 0U) {
        return 0U;
    }
    if (interrupted_kind == BRIDGE_MOTION_PREPARE) {
        return bridge_start_unprepare();
    }
    if (interrupted_kind == BRIDGE_MOTION_UNPREPARE) {
        if (source_checkpoint >= 5U) {
            return bridge_start_unprepare();
        }
        return bridge_start_recovery(source_checkpoint, DOG_LEG_MASK_ALL, 0.0f,
                                     DOG_OBSTACLE_BRIDGE_PREPARE_MS);
    }
    if (interrupted_kind == BRIDGE_MOTION_RECOVER) {
        return bridge_start_recovery(source_checkpoint, s_recovery_leg_mask,
                                     s_recovery_clearance_mm,
                                     s_recovery_duration_ms);
    }
    if ((source_checkpoint >= 5U) || (target_checkpoint >= 5U)) {
        return 0U;
    }
    return bridge_start_recovery(
        source_checkpoint,
        bridge_transition_mask(source_checkpoint, target_checkpoint),
        bridge_transition_clearance(source_checkpoint, target_checkpoint),
        bridge_transition_duration(source_checkpoint, target_checkpoint));
}

static void bridge_forward_step(void)
{
    if (s_status.prepared == 0U) {
        (void)bridge_start_prepare();
        return;
    }
    const uint8_t from_cp = s_status.checkpoint;
    uint8_t to_cp = 0U;
    if ((from_cp == 0U) || (from_cp == 4U)) {
        to_cp = 1U;
    } else if (from_cp < 4U) {
        to_cp = (uint8_t)(from_cp + 1U);
    } else {
        return;
    }
    (void)bridge_start_motion(from_cp, to_cp, BRIDGE_MOTION_CHECKPOINT, 1);
}

static void bridge_backward_step(void)
{
    if (s_status.prepared == 0U) {
        return;
    }
    const uint8_t from_cp = s_status.checkpoint;
    if (from_cp == 0U) {
        (void)bridge_start_unprepare();
        return;
    }
    const uint8_t to_cp = (uint8_t)(from_cp - 1U);
    (void)bridge_start_motion(from_cp, to_cp, BRIDGE_MOTION_CHECKPOINT, -1);
}

static void bridge_motion_complete(void)
{
    dog_foot_motion_cancel();
    s_status.fault = DOG_OBSTACLE_FAULT_NONE;
    s_status.motion_state = DOG_FOOT_MOTION_IDLE;
    if (s_motion_kind == BRIDGE_MOTION_PREPARE) {
        s_status.prepared = 1U;
        s_status.checkpoint = 0U;
    } else if (s_motion_kind == BRIDGE_MOTION_UNPREPARE) {
        s_status.prepared = 0U;
        s_status.checkpoint = kCheckpointUnprepared;
        s_status.can_exit = 1U;
    } else if (s_motion_kind == BRIDGE_MOTION_CHECKPOINT) {
        const uint8_t completed_target = s_status.target_checkpoint;
        if ((s_motion_direction > 0) && (completed_target == 4U)) {
            s_status.completed_gaps++;
        } else if ((s_motion_direction < 0) &&
                   (s_motion_source_checkpoint == 4U) &&
                   (s_status.completed_gaps > 0U)) {
            s_status.completed_gaps--;
        }
        s_status.checkpoint = completed_target;
    } else if (s_motion_kind == BRIDGE_MOTION_RECOVER) {
        s_status.checkpoint = s_status.target_checkpoint;
    }
    s_motion_kind = BRIDGE_MOTION_NONE;
    s_motion_direction = 0;
    s_recovery_leg_mask = DOG_LEG_MASK_ALL;
    s_recovery_clearance_mm = 0.0f;
    s_recovery_duration_ms = 0U;
    s_bridge_exit_recovery_attempted = 0U;
    s_status.target_checkpoint = s_status.checkpoint;
    obstacle_set_state(DOG_OBSTACLE_READY);
}

static void bridge_exit_tick(void)
{
    if ((s_status.state == DOG_OBSTACLE_MOVING) ||
        (s_status.state == DOG_OBSTACLE_ROLLBACK)) {
        return;
    }
    if (s_status.state == DOG_OBSTACLE_FAULT) {
        if (s_bridge_exit_recovery_attempted != 0U) {
            if (s_bridge_exit_recovery_attempted == 1U) {
                DebugUart_Printf("OBSTACLE exit recovery stopped after repeated motion failure; use CH9 safety abort.\r\n");
                s_bridge_exit_recovery_attempted = 2U;
            }
            s_status.can_exit = 0U;
            return;
        }
        if (bridge_recover_interrupted_motion() == 0U) {
            return;
        }
        if ((s_status.state == DOG_OBSTACLE_MOVING) ||
            (s_status.state == DOG_OBSTACLE_ROLLBACK)) {
            s_bridge_exit_recovery_attempted = 1U;
            return;
        }
        s_status.fault = DOG_OBSTACLE_FAULT_NONE;
        obstacle_set_state(DOG_OBSTACLE_READY);
    }
    if (s_status.prepared == 0U) {
        s_status.can_exit = 1U;
        obstacle_set_state(DOG_OBSTACLE_DISABLED);
        return;
    }
    if ((s_status.checkpoint == 0U) || (s_status.checkpoint == 4U)) {
        s_bridge_exit_recovery_attempted = 1U;
        (void)bridge_start_unprepare();
    } else {
        s_bridge_exit_recovery_attempted = 1U;
        bridge_backward_step();
    }
}

static uint8_t stair_path_valid(uint8_t source, uint8_t target,
                                uint8_t leg_mask, float clearance_mm)
{
    if ((source >= kStairWaypointCount) || (target >= kStairWaypointCount)) {
        return 0U;
    }
    return dog_foot_motion_path_is_valid(
        leg_mask, s_stair.waypoint[source], s_stair.waypoint[target],
        clearance_mm);
}

static uint8_t stair_geometry_valid(void)
{
    if (s_stair.geometry_ready == 0U) {
        stair_build_geometry();
    }
    struct StairPathCheck {
        uint8_t source;
        uint8_t target;
        uint8_t mask;
        float clearance;
    };
    static const StairPathCheck checks[] = {
        {DOG_STAIR_PHASE_IDLE, DOG_STAIR_PHASE_PREP_BODY_SHIFT,
         DOG_LEG_MASK_ALL, 0.0f},
        {DOG_STAIR_PHASE_PREP_BODY_SHIFT,
         DOG_STAIR_PHASE_PREP_LEFT_REAR_COMPACT,
         kLeftRearLegMask, DOG_OBSTACLE_STAIR_FLAT_CLEARANCE_MM},
        {DOG_STAIR_PHASE_PREP_LEFT_REAR_COMPACT,
         DOG_STAIR_PHASE_PREP_REAR_COMPACT,
         kRightRearLegMask, DOG_OBSTACLE_STAIR_FLAT_CLEARANCE_MM},
        {DOG_STAIR_PHASE_LEVEL_READY, DOG_STAIR_PHASE_LEFT_FRONT_LIFT,
         kLeftFrontLegMask, 0.0f},
        {DOG_STAIR_PHASE_LEFT_FRONT_LIFT,
         DOG_STAIR_PHASE_LEFT_FRONT_FORWARD,
         kLeftFrontLegMask, 0.0f},
        {DOG_STAIR_PHASE_LEFT_FRONT_FORWARD,
         DOG_STAIR_PHASE_LEFT_FRONT_LAND,
         kLeftFrontLegMask, 0.0f},
        {DOG_STAIR_PHASE_LEFT_FRONT_LAND,
         DOG_STAIR_PHASE_RIGHT_FRONT_LIFT,
         kRightFrontLegMask, 0.0f},
        {DOG_STAIR_PHASE_RIGHT_FRONT_LIFT,
         DOG_STAIR_PHASE_RIGHT_FRONT_FORWARD,
         kRightFrontLegMask, 0.0f},
        {DOG_STAIR_PHASE_RIGHT_FRONT_FORWARD,
         DOG_STAIR_PHASE_RIGHT_FRONT_LAND,
         kRightFrontLegMask, 0.0f},
        {DOG_STAIR_PHASE_RIGHT_FRONT_LAND, DOG_STAIR_PHASE_BODY_SHIFT,
         DOG_LEG_MASK_ALL, 0.0f},
        {DOG_STAIR_PHASE_BODY_SHIFT, DOG_STAIR_PHASE_LEFT_REAR_LIFT,
         kLeftRearLegMask, 0.0f},
        {DOG_STAIR_PHASE_LEFT_REAR_LIFT,
         DOG_STAIR_PHASE_LEFT_REAR_FORWARD,
         kLeftRearLegMask, 0.0f},
        {DOG_STAIR_PHASE_LEFT_REAR_FORWARD,
         DOG_STAIR_PHASE_LEFT_REAR_LAND,
         kLeftRearLegMask, 0.0f},
        {DOG_STAIR_PHASE_LEFT_REAR_LAND,
         DOG_STAIR_PHASE_RIGHT_REAR_LIFT,
         kRightRearLegMask, 0.0f},
        {DOG_STAIR_PHASE_RIGHT_REAR_LIFT,
         DOG_STAIR_PHASE_RIGHT_REAR_FORWARD,
         kRightRearLegMask, 0.0f},
        {DOG_STAIR_PHASE_RIGHT_REAR_FORWARD,
         DOG_STAIR_PHASE_RIGHT_REAR_LAND,
         kRightRearLegMask, 0.0f},
        {DOG_STAIR_PHASE_RIGHT_REAR_LAND, DOG_STAIR_PHASE_BODY_RAISE,
         DOG_LEG_MASK_ALL, 0.0f},
        {DOG_STAIR_PHASE_BODY_RAISE,
         DOG_STAIR_PHASE_TOP_LEFT_FRONT_ADVANCE,
         kLeftFrontLegMask, DOG_OBSTACLE_STAIR_FLAT_CLEARANCE_MM},
        {DOG_STAIR_PHASE_TOP_LEFT_FRONT_ADVANCE,
         DOG_STAIR_PHASE_TOP_RIGHT_FRONT_ADVANCE,
         kRightFrontLegMask, DOG_OBSTACLE_STAIR_FLAT_CLEARANCE_MM},
        {DOG_STAIR_PHASE_TOP_RIGHT_FRONT_ADVANCE,
         DOG_STAIR_PHASE_TOP_BODY_SHIFT,
         DOG_LEG_MASK_ALL, 0.0f},
        {DOG_STAIR_PHASE_TOP_BODY_SHIFT,
         DOG_STAIR_PHASE_TOP_LEFT_REAR_ADVANCE,
         kLeftRearLegMask, DOG_OBSTACLE_STAIR_FLAT_CLEARANCE_MM},
        {DOG_STAIR_PHASE_TOP_LEFT_REAR_ADVANCE,
         DOG_STAIR_PHASE_TOP_RIGHT_REAR_ADVANCE,
         kRightRearLegMask, DOG_OBSTACLE_STAIR_FLAT_CLEARANCE_MM},
        {DOG_STAIR_PHASE_TOP_RIGHT_REAR_ADVANCE,
         DOG_STAIR_PHASE_TOP_BODY_NORMALIZE,
         DOG_LEG_MASK_ALL, 0.0f},
        {DOG_STAIR_PHASE_TOP_BODY_NORMALIZE,
         DOG_STAIR_PHASE_TOP_LEFT_FRONT_NORMAL,
         kLeftFrontLegMask, DOG_OBSTACLE_STAIR_FLAT_CLEARANCE_MM},
        {DOG_STAIR_PHASE_TOP_LEFT_FRONT_NORMAL,
         DOG_STAIR_PHASE_TOP_RIGHT_FRONT_NORMAL,
         kRightFrontLegMask, DOG_OBSTACLE_STAIR_FLAT_CLEARANCE_MM},
    };
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(checks) / sizeof(checks[0])); ++i) {
        if (stair_path_valid(checks[i].source, checks[i].target,
                             checks[i].mask, checks[i].clearance) == 0U) {
            DebugUart_Printf("STAIRS precheck FAIL path=%u->%u.\r\n",
                             (unsigned)checks[i].source,
                             (unsigned)checks[i].target);
            return 0U;
        }
    }
    return 1U;
}

static void stair_sequence_clear(uint8_t recovering, uint8_t goal_phase)
{
    memset(s_stair.sequence, 0, sizeof(s_stair.sequence));
    s_stair.sequence_count = 0U;
    s_stair.sequence_index = 0U;
    s_stair.sequence_recovering = recovering;
    s_stair.motion_active = 0U;
    s_stair.failed_segment_active = 0U;
    s_stair.paused = 0U;
    s_stair.recovery_goal_phase = goal_phase;
    s_stair.dwell_started_ms = 0U;
    s_stair.dwell_duration_ms = 0U;
}

static uint8_t stair_sequence_add(uint8_t source, uint8_t target,
                                  uint8_t leg_mask, uint32_t duration_ms,
                                  float clearance_mm, uint32_t dwell_ms)
{
    if ((s_stair.sequence_count >= kStairMaxSegments) ||
        (source >= kStairWaypointCount) || (target >= kStairWaypointCount)) {
        return 0U;
    }
    StairSegment *segment = &s_stair.sequence[s_stair.sequence_count++];
    segment->source_phase = source;
    segment->target_phase = target;
    segment->leg_mask = leg_mask;
    segment->duration_ms = duration_ms;
    segment->clearance_mm = clearance_mm;
    segment->dwell_ms = dwell_ms;
    return 1U;
}

static uint8_t stair_add_level_sequence(uint8_t source_phase)
{
    return
        stair_sequence_add(source_phase, DOG_STAIR_PHASE_LEFT_FRONT_LIFT,
                           kLeftFrontLegMask, DOG_OBSTACLE_STAIR_LIFT_MS,
                           0.0f, 0U) &&
        stair_sequence_add(DOG_STAIR_PHASE_LEFT_FRONT_LIFT,
                           DOG_STAIR_PHASE_LEFT_FRONT_FORWARD,
                           kLeftFrontLegMask, DOG_OBSTACLE_STAIR_FORWARD_MS,
                           0.0f, 0U) &&
        stair_sequence_add(DOG_STAIR_PHASE_LEFT_FRONT_FORWARD,
                           DOG_STAIR_PHASE_LEFT_FRONT_LAND,
                           kLeftFrontLegMask, DOG_OBSTACLE_STAIR_LAND_MS,
                           0.0f, DOG_OBSTACLE_STAIR_DWELL_MS) &&
        stair_sequence_add(DOG_STAIR_PHASE_LEFT_FRONT_LAND,
                           DOG_STAIR_PHASE_RIGHT_FRONT_LIFT,
                           kRightFrontLegMask, DOG_OBSTACLE_STAIR_LIFT_MS,
                           0.0f, 0U) &&
        stair_sequence_add(DOG_STAIR_PHASE_RIGHT_FRONT_LIFT,
                           DOG_STAIR_PHASE_RIGHT_FRONT_FORWARD,
                           kRightFrontLegMask, DOG_OBSTACLE_STAIR_FORWARD_MS,
                           0.0f, 0U) &&
        stair_sequence_add(DOG_STAIR_PHASE_RIGHT_FRONT_FORWARD,
                           DOG_STAIR_PHASE_RIGHT_FRONT_LAND,
                           kRightFrontLegMask, DOG_OBSTACLE_STAIR_LAND_MS,
                           0.0f, DOG_OBSTACLE_STAIR_DWELL_MS) &&
        stair_sequence_add(DOG_STAIR_PHASE_RIGHT_FRONT_LAND,
                           DOG_STAIR_PHASE_BODY_SHIFT,
                           DOG_LEG_MASK_ALL, DOG_OBSTACLE_STAIR_BODY_SHIFT_MS,
                           0.0f, 0U) &&
        stair_sequence_add(DOG_STAIR_PHASE_BODY_SHIFT,
                           DOG_STAIR_PHASE_LEFT_REAR_LIFT,
                           kLeftRearLegMask, DOG_OBSTACLE_STAIR_LIFT_MS,
                           0.0f, 0U) &&
        stair_sequence_add(DOG_STAIR_PHASE_LEFT_REAR_LIFT,
                           DOG_STAIR_PHASE_LEFT_REAR_FORWARD,
                           kLeftRearLegMask, DOG_OBSTACLE_STAIR_FORWARD_MS,
                           0.0f, 0U) &&
        stair_sequence_add(DOG_STAIR_PHASE_LEFT_REAR_FORWARD,
                           DOG_STAIR_PHASE_LEFT_REAR_LAND,
                           kLeftRearLegMask, DOG_OBSTACLE_STAIR_LAND_MS,
                           0.0f, DOG_OBSTACLE_STAIR_DWELL_MS) &&
        stair_sequence_add(DOG_STAIR_PHASE_LEFT_REAR_LAND,
                           DOG_STAIR_PHASE_RIGHT_REAR_LIFT,
                           kRightRearLegMask, DOG_OBSTACLE_STAIR_LIFT_MS,
                           0.0f, 0U) &&
        stair_sequence_add(DOG_STAIR_PHASE_RIGHT_REAR_LIFT,
                           DOG_STAIR_PHASE_RIGHT_REAR_FORWARD,
                           kRightRearLegMask, DOG_OBSTACLE_STAIR_FORWARD_MS,
                           0.0f, 0U) &&
        stair_sequence_add(DOG_STAIR_PHASE_RIGHT_REAR_FORWARD,
                           DOG_STAIR_PHASE_RIGHT_REAR_LAND,
                           kRightRearLegMask, DOG_OBSTACLE_STAIR_LAND_MS,
                           0.0f, DOG_OBSTACLE_STAIR_DWELL_MS) &&
        stair_sequence_add(DOG_STAIR_PHASE_RIGHT_REAR_LAND,
                           DOG_STAIR_PHASE_BODY_RAISE,
                           DOG_LEG_MASK_ALL, DOG_OBSTACLE_STAIR_BODY_RAISE_MS,
                           0.0f, DOG_OBSTACLE_STAIR_DWELL_MS);
}

static uint8_t stair_add_top_sequence(void)
{
    return
        stair_sequence_add(DOG_STAIR_PHASE_BODY_RAISE,
                           DOG_STAIR_PHASE_TOP_LEFT_FRONT_ADVANCE,
                           kLeftFrontLegMask, DOG_OBSTACLE_STAIR_FORWARD_MS,
                           DOG_OBSTACLE_STAIR_FLAT_CLEARANCE_MM,
                           DOG_OBSTACLE_STAIR_DWELL_MS) &&
        stair_sequence_add(DOG_STAIR_PHASE_TOP_LEFT_FRONT_ADVANCE,
                           DOG_STAIR_PHASE_TOP_RIGHT_FRONT_ADVANCE,
                           kRightFrontLegMask, DOG_OBSTACLE_STAIR_FORWARD_MS,
                           DOG_OBSTACLE_STAIR_FLAT_CLEARANCE_MM,
                           DOG_OBSTACLE_STAIR_DWELL_MS) &&
        stair_sequence_add(DOG_STAIR_PHASE_TOP_RIGHT_FRONT_ADVANCE,
                           DOG_STAIR_PHASE_TOP_BODY_SHIFT,
                           DOG_LEG_MASK_ALL, DOG_OBSTACLE_STAIR_BODY_SHIFT_MS,
                           0.0f, 0U) &&
        stair_sequence_add(DOG_STAIR_PHASE_TOP_BODY_SHIFT,
                           DOG_STAIR_PHASE_TOP_LEFT_REAR_ADVANCE,
                           kLeftRearLegMask, DOG_OBSTACLE_STAIR_FORWARD_MS,
                           DOG_OBSTACLE_STAIR_FLAT_CLEARANCE_MM,
                           DOG_OBSTACLE_STAIR_DWELL_MS) &&
        stair_sequence_add(DOG_STAIR_PHASE_TOP_LEFT_REAR_ADVANCE,
                           DOG_STAIR_PHASE_TOP_RIGHT_REAR_ADVANCE,
                           kRightRearLegMask, DOG_OBSTACLE_STAIR_FORWARD_MS,
                           DOG_OBSTACLE_STAIR_FLAT_CLEARANCE_MM,
                           DOG_OBSTACLE_STAIR_DWELL_MS) &&
        stair_sequence_add(DOG_STAIR_PHASE_TOP_RIGHT_REAR_ADVANCE,
                           DOG_STAIR_PHASE_TOP_BODY_NORMALIZE,
                           DOG_LEG_MASK_ALL, DOG_OBSTACLE_STAIR_PREPARE_MS,
                           0.0f, 0U) &&
        stair_sequence_add(DOG_STAIR_PHASE_TOP_BODY_NORMALIZE,
                           DOG_STAIR_PHASE_TOP_LEFT_FRONT_NORMAL,
                           kLeftFrontLegMask, DOG_OBSTACLE_STAIR_FORWARD_MS,
                           DOG_OBSTACLE_STAIR_FLAT_CLEARANCE_MM,
                           DOG_OBSTACLE_STAIR_DWELL_MS) &&
        stair_sequence_add(DOG_STAIR_PHASE_TOP_LEFT_FRONT_NORMAL,
                           DOG_STAIR_PHASE_TOP_RIGHT_FRONT_NORMAL,
                           kRightFrontLegMask, DOG_OBSTACLE_STAIR_FORWARD_MS,
                           DOG_OBSTACLE_STAIR_FLAT_CLEARANCE_MM,
                           DOG_OBSTACLE_STAIR_DWELL_MS);
}

static uint8_t stair_build_forward_sequence(void)
{
    const uint8_t next_level = (uint8_t)(s_status.completed_levels + 1U);
    if ((next_level == 0U) ||
        (next_level > DOG_OBSTACLE_STAIR_LEVEL_COUNT)) {
        return 0U;
    }
    stair_sequence_clear(0U, s_stair.current_phase);
    uint8_t level_source = DOG_STAIR_PHASE_LEVEL_READY;
    if ((s_status.completed_levels == 0U) &&
        (s_stair.current_phase == DOG_STAIR_PHASE_IDLE)) {
        if ((stair_sequence_add(DOG_STAIR_PHASE_IDLE,
                                DOG_STAIR_PHASE_PREP_BODY_SHIFT,
                                DOG_LEG_MASK_ALL,
                                DOG_OBSTACLE_STAIR_PREPARE_MS,
                                0.0f, 0U) == 0U) ||
            (stair_sequence_add(DOG_STAIR_PHASE_PREP_BODY_SHIFT,
                                DOG_STAIR_PHASE_PREP_LEFT_REAR_COMPACT,
                                kLeftRearLegMask,
                                DOG_OBSTACLE_STAIR_PREPARE_MS,
                                DOG_OBSTACLE_STAIR_FLAT_CLEARANCE_MM,
                                DOG_OBSTACLE_STAIR_DWELL_MS) == 0U) ||
            (stair_sequence_add(DOG_STAIR_PHASE_PREP_LEFT_REAR_COMPACT,
                                DOG_STAIR_PHASE_PREP_REAR_COMPACT,
                                kRightRearLegMask,
                                DOG_OBSTACLE_STAIR_PREPARE_MS,
                                DOG_OBSTACLE_STAIR_FLAT_CLEARANCE_MM,
                                DOG_OBSTACLE_STAIR_DWELL_MS) == 0U)) {
            return 0U;
        }
        level_source = DOG_STAIR_PHASE_PREP_REAR_COMPACT;
    }
    if (stair_add_level_sequence(level_source) == 0U) {
        return 0U;
    }
    if ((next_level == DOG_OBSTACLE_STAIR_LEVEL_COUNT) &&
        (stair_add_top_sequence() == 0U)) {
        return 0U;
    }
    return 1U;
}

static uint8_t stair_start_current_segment(int8_t direction)
{
    if ((s_stair.sequence_index >= s_stair.sequence_count) ||
        (s_stair.motion_active != 0U)) {
        return 0U;
    }
    const StairSegment *segment = &s_stair.sequence[s_stair.sequence_index];
    if (dog_foot_motion_start(
            segment->leg_mask, s_stair.waypoint[segment->target_phase],
            segment->clearance_mm, segment->duration_ms,
            segment->duration_ms + DOG_OBSTACLE_STAIR_TIMEOUT_MARGIN_MS) == 0U) {
        obstacle_fault(DOG_OBSTACLE_FAULT_MOTION_START);
        return 0U;
    }
    s_stair.motion_active = 1U;
    s_stair.failed_segment_active = 0U;
    s_stair.motion_source_phase = segment->source_phase;
    s_stair.motion_target_phase = segment->target_phase;
    s_status.prepared = 1U;
    s_status.phase = segment->target_phase;
    s_status.target_checkpoint = segment->target_phase;
    s_status.motion_state = DOG_FOOT_MOTION_ACTIVE;
    s_status.can_exit = 0U;
    obstacle_set_state((direction < 0) ? DOG_OBSTACLE_ROLLBACK :
                                         DOG_OBSTACLE_MOVING);
    return 1U;
}

static void stair_sequence_finish(void)
{
    s_stair.sequence_count = 0U;
    s_stair.sequence_index = 0U;
    s_stair.motion_active = 0U;
    s_stair.failed_segment_active = 0U;
    s_stair.paused = 0U;
    s_stair.dwell_started_ms = 0U;
    s_stair.dwell_duration_ms = 0U;
    s_status.fault = DOG_OBSTACLE_FAULT_NONE;
    s_status.motion_state = DOG_FOOT_MOTION_IDLE;

    if (s_stair.sequence_recovering != 0U) {
        s_stair.current_phase = s_stair.recovery_goal_phase;
        s_status.phase = s_stair.current_phase;
        s_status.checkpoint = s_stair.current_phase;
        s_status.target_checkpoint = s_stair.current_phase;
        s_status.prepared = (s_stair.current_phase == DOG_STAIR_PHASE_IDLE) ? 0U : 1U;
        s_status.can_exit = (s_status.prepared == 0U) ? 1U : 0U;
        s_stair.sequence_recovering = 0U;
        obstacle_set_state(DOG_OBSTACLE_READY);
        return;
    }

    if (s_status.completed_levels < DOG_OBSTACLE_STAIR_LEVEL_COUNT) {
        s_status.completed_levels++;
    }
    if (s_status.completed_levels >= DOG_OBSTACLE_STAIR_LEVEL_COUNT) {
        s_stair.current_phase = DOG_STAIR_PHASE_TOP_READY;
        s_status.phase = DOG_STAIR_PHASE_TOP_READY;
        s_status.checkpoint = DOG_STAIR_PHASE_TOP_READY;
        s_status.target_checkpoint = DOG_STAIR_PHASE_TOP_READY;
        s_status.prepared = 0U;
        s_status.can_exit = 1U;
    } else {
        s_stair.current_phase = DOG_STAIR_PHASE_LEVEL_READY;
        s_status.phase = DOG_STAIR_PHASE_LEVEL_READY;
        s_status.checkpoint = DOG_STAIR_PHASE_LEVEL_READY;
        s_status.target_checkpoint = DOG_STAIR_PHASE_LEVEL_READY;
        s_status.prepared = 1U;
        s_status.can_exit = 0U;
    }
    obstacle_set_state(DOG_OBSTACLE_READY);
}

static void stair_continue_sequence(uint32_t now_ms, uint8_t mode_requested)
{
    if ((s_stair.sequence_count == 0U) ||
        (s_stair.motion_active != 0U)) {
        return;
    }
    if (s_stair.dwell_duration_ms != 0U) {
        if (s_stair.dwell_started_ms == 0U) {
            s_stair.dwell_started_ms = (now_ms != 0U) ? now_ms : 1U;
            return;
        }
        if ((uint32_t)(now_ms - s_stair.dwell_started_ms) <
            s_stair.dwell_duration_ms) {
            return;
        }
        s_stair.dwell_started_ms = 0U;
        s_stair.dwell_duration_ms = 0U;
    }
    if (s_stair.sequence_index >= s_stair.sequence_count) {
        stair_sequence_finish();
        return;
    }
    if (mode_requested == 0U) {
        s_stair.paused = 1U;
        s_status.phase = s_stair.current_phase;
        s_status.checkpoint = s_stair.current_phase;
        s_status.target_checkpoint = s_stair.current_phase;
        s_status.motion_state = DOG_FOOT_MOTION_IDLE;
        s_status.can_exit = 0U;
        obstacle_set_state(DOG_OBSTACLE_READY);
        return;
    }
    s_stair.paused = 0U;
    (void)stair_start_current_segment(
        (s_stair.sequence_recovering != 0U) ? -1 : 1);
}

static void stair_motion_complete(uint32_t now_ms, uint8_t mode_requested)
{
    if ((s_stair.motion_active == 0U) ||
        (s_stair.sequence_index >= s_stair.sequence_count)) {
        obstacle_fault(DOG_OBSTACLE_FAULT_MOTION_RUNTIME);
        return;
    }
    const StairSegment *segment = &s_stair.sequence[s_stair.sequence_index];
    dog_foot_motion_cancel();
    s_stair.motion_active = 0U;
    s_stair.current_phase = segment->target_phase;
    s_status.phase = s_stair.current_phase;
    s_status.checkpoint = s_stair.current_phase;
    s_status.target_checkpoint = s_stair.current_phase;
    s_status.motion_state = DOG_FOOT_MOTION_IDLE;
    s_stair.sequence_index++;
    s_stair.dwell_started_ms = 0U;
    s_stair.dwell_duration_ms = segment->dwell_ms;
    stair_continue_sequence(now_ms, mode_requested);
}

static uint8_t stair_build_recovery_sequence(void)
{
    if ((s_stair.sequence_count == 0U) ||
        (dog_foot_motion_prepare() == 0U)) {
        return 0U;
    }
    StairSegment original[kStairMaxSegments] = {};
    const uint8_t original_count = s_stair.sequence_count;
    const uint8_t original_index = s_stair.sequence_index;
    const uint8_t was_recovering = s_stair.sequence_recovering;
    const uint8_t failed_active = s_stair.failed_segment_active;
    memcpy(original, s_stair.sequence, sizeof(original));
    const uint8_t goal = (was_recovering != 0U) ?
        s_stair.recovery_goal_phase : original[0].source_phase;
    stair_sequence_clear(1U, goal);

    if ((failed_active != 0U) && (original_index < original_count)) {
        const StairSegment *failed = &original[original_index];
        if (stair_sequence_add(s_stair.current_phase, failed->source_phase,
                               failed->leg_mask, failed->duration_ms,
                               failed->clearance_mm,
                               DOG_OBSTACLE_STAIR_DWELL_MS) == 0U) {
            return 0U;
        }
    }
    if (was_recovering != 0U) {
        for (uint8_t i = original_index; i < original_count; ++i) {
            const StairSegment *segment = &original[i];
            if (stair_sequence_add(segment->source_phase,
                                   segment->target_phase,
                                   segment->leg_mask, segment->duration_ms,
                                   segment->clearance_mm, segment->dwell_ms) == 0U) {
                return 0U;
            }
        }
    } else {
        for (uint8_t i = original_index; i > 0U; --i) {
            const StairSegment *segment = &original[i - 1U];
            if (stair_sequence_add(segment->target_phase,
                                   segment->source_phase,
                                   segment->leg_mask, segment->duration_ms,
                                   segment->clearance_mm,
                                   DOG_OBSTACLE_STAIR_DWELL_MS) == 0U) {
                return 0U;
            }
        }
    }
    s_stair.recovery_goal_phase = goal;
    s_status.fault = DOG_OBSTACLE_FAULT_NONE;
    s_status.motion_state = DOG_FOOT_MOTION_IDLE;
    s_status.can_exit = 0U;
    if (s_stair.sequence_count == 0U) {
        stair_sequence_finish();
        return 1U;
    }
    return stair_start_current_segment(-1);
}

static void stair_exit_tick(void)
{
    if ((s_stair.motion_active != 0U) ||
        (s_stair.sequence_count != 0U)) {
        s_stair.paused = 1U;
        s_status.can_exit = 0U;
        return;
    }
    if ((s_stair.current_phase == DOG_STAIR_PHASE_IDLE) ||
        (s_stair.current_phase == DOG_STAIR_PHASE_TOP_READY)) {
        s_status.prepared = 0U;
        s_status.can_exit = 1U;
        obstacle_set_state(DOG_OBSTACLE_DISABLED);
        return;
    }
    s_status.can_exit = 0U;
    obstacle_set_state(DOG_OBSTACLE_READY);
}

void DogObstacle_Init(void)
{
    taskENTER_CRITICAL();
    s_requested_mode = 0U;
    s_requested_selection = DOG_OBSTACLE_BRIDGE_B;
    s_requested_step = 0;
    s_requested_safety_abort = 0U;
    s_request_epoch++;
    taskEXIT_CRITICAL();
    memset(&s_status, 0, sizeof(s_status));
    s_status.selected = DOG_OBSTACLE_BRIDGE_B;
    s_status.state = DOG_OBSTACLE_DISABLED;
    s_status.checkpoint = kCheckpointUnprepared;
    s_status.target_checkpoint = kCheckpointUnprepared;
    s_status.can_exit = 1U;
    s_status.active_gap_mm = DOG_OBSTACLE_BRIDGE_ACTIVE_GAP_MM;
    s_status.final_gap_mm = DOG_OBSTACLE_BRIDGE_FINAL_GAP_MM;
    s_status.phase = DOG_STAIR_PHASE_IDLE;
    s_status.total_levels = DOG_OBSTACLE_STAIR_LEVEL_COUNT;
    s_status.step_height_mm = DOG_OBSTACLE_STAIR_STEP_HEIGHT_MM;
    s_status.tread_depth_mm = DOG_OBSTACLE_STAIR_TREAD_MM;
    s_geometry_ready = 0U;
    s_motion_kind = BRIDGE_MOTION_NONE;
    s_motion_source_checkpoint = kCheckpointUnprepared;
    s_motion_direction = 0;
    s_recovery_leg_mask = DOG_LEG_MASK_ALL;
    s_recovery_clearance_mm = 0.0f;
    s_recovery_duration_ms = 0U;
    s_bridge_exit_recovery_attempted = 0U;
    stair_context_reset();
    obstacle_publish_status();
}

void DogObstacle_SetModeRequested(uint8_t active)
{
    const uint8_t requested = (active != 0U) ? 1U : 0U;
    taskENTER_CRITICAL();
    if (s_requested_mode != requested) {
        s_requested_mode = requested;
        s_requested_step = 0;
        s_request_epoch++;
    }
    taskEXIT_CRITICAL();
}

void DogObstacle_Select(uint8_t obstacle)
{
    if (obstacle > DOG_OBSTACLE_GRAVEL) {
        obstacle = DOG_OBSTACLE_BRIDGE_B;
    }
    taskENTER_CRITICAL();
    if (s_requested_selection != obstacle) {
        s_requested_selection = obstacle;
        s_requested_step = 0;
        s_request_epoch++;
    }
    taskEXIT_CRITICAL();
}

void DogObstacle_RequestStep(int8_t direction)
{
    const int8_t request = (direction > 0) ? 1 : ((direction < 0) ? -1 : 0);
    if (request == 0) {
        return;
    }
    taskENTER_CRITICAL();
    if ((s_requested_mode != 0U) && (s_requested_step == 0)) {
        s_requested_step = request;
    }
    taskEXIT_CRITICAL();
}

void DogObstacle_RequestSafetyAbort(void)
{
    taskENTER_CRITICAL();
    s_requested_safety_abort = 1U;
    s_requested_mode = 0U;
    s_requested_step = 0;
    s_request_epoch++;
    taskEXIT_CRITICAL();
}

void DogObstacle_Tick(uint32_t now_ms)
{
    uint8_t requested_mode = 0U;
    uint8_t requested_selection = DOG_OBSTACLE_BRIDGE_B;
    int8_t requested_step = 0;
    uint8_t requested_safety_abort = 0U;
    uint32_t request_epoch = 0U;
    ObstacleStatusPublishGuard publish_guard;
    taskENTER_CRITICAL();
    requested_mode = s_requested_mode;
    requested_selection = s_requested_selection;
    requested_step = s_requested_step;
    requested_safety_abort = s_requested_safety_abort;
    request_epoch = s_request_epoch;
    s_requested_step = 0;
    s_requested_safety_abort = 0U;
    taskEXIT_CRITICAL();

    s_status.mode_requested = requested_mode;
    if (DogSafety_IsLatched() != 0U) {
        obstacle_control_abort(DOG_OBSTACLE_FAULT_SAFETY);
        return;
    }
    if (dog_mit_fault_hold_is_active() != 0U) {
        obstacle_control_abort(DOG_OBSTACLE_FAULT_MOTION_RUNTIME);
        return;
    }
    if (requested_safety_abort != 0U) {
        obstacle_control_abort(DOG_OBSTACLE_FAULT_SAFETY);
        return;
    }
    const uint8_t stair_mode_requested =
        ((requested_mode != 0U) &&
         (requested_selection == DOG_OBSTACLE_STAIRS)) ? 1U : 0U;

    if ((s_status.selected == DOG_OBSTACLE_BRIDGE_B) &&
        ((s_status.state == DOG_OBSTACLE_MOVING) ||
         (s_status.state == DOG_OBSTACLE_ROLLBACK))) {
        s_status.motion_state = dog_foot_motion_state();
        if (s_status.motion_state == DOG_FOOT_MOTION_COMPLETE) {
            bridge_motion_complete();
        } else if (s_status.motion_state == DOG_FOOT_MOTION_FAULT) {
            obstacle_fault(DOG_OBSTACLE_FAULT_MOTION_RUNTIME);
        } else if (s_status.motion_state == DOG_FOOT_MOTION_ACTIVE) {
            return;
        } else {
            obstacle_fault(DOG_OBSTACLE_FAULT_MOTION_RUNTIME);
        }
        /* Stick edges generated during a motion never chain the next phase. */
        requested_step = 0;
    }

    if ((s_status.selected == DOG_OBSTACLE_STAIRS) &&
        (s_stair.sequence_count != 0U) &&
        (s_status.state != DOG_OBSTACLE_FAULT)) {
        requested_step = 0;
        if (s_stair.motion_active != 0U) {
            s_status.motion_state = dog_foot_motion_state();
            if (s_status.motion_state == DOG_FOOT_MOTION_COMPLETE) {
                stair_motion_complete(now_ms, stair_mode_requested);
            } else if (s_status.motion_state == DOG_FOOT_MOTION_FAULT) {
                obstacle_fault(DOG_OBSTACLE_FAULT_MOTION_RUNTIME);
            } else if (s_status.motion_state == DOG_FOOT_MOTION_ACTIVE) {
                return;
            } else {
                obstacle_fault(DOG_OBSTACLE_FAULT_MOTION_RUNTIME);
            }
        } else {
            stair_continue_sequence(now_ms, stair_mode_requested);
        }
    }

    if ((s_status.can_exit != 0U) &&
        (s_status.prepared == 0U) &&
        (s_status.state != DOG_OBSTACLE_MOVING) &&
        (s_status.state != DOG_OBSTACLE_ROLLBACK)) {
        if (s_status.selected != requested_selection) {
            obstacle_reset_selection(requested_selection);
        }
    }

    if ((requested_mode == 0U) ||
        ((s_status.selected == DOG_OBSTACLE_STAIRS) &&
         (stair_mode_requested == 0U))) {
        if (s_status.selected == DOG_OBSTACLE_BRIDGE_B) {
            bridge_exit_tick();
        } else if (s_status.selected == DOG_OBSTACLE_STAIRS) {
            stair_exit_tick();
        } else {
            s_status.can_exit = 1U;
            obstacle_set_state(DOG_OBSTACLE_DISABLED);
        }
        return;
    }

    s_status.can_exit = ((s_status.prepared == 0U) &&
                         (s_status.state != DOG_OBSTACLE_MOVING) &&
                         (s_status.state != DOG_OBSTACLE_ROLLBACK)) ? 1U : 0U;
    if (s_status.selected == DOG_OBSTACLE_GRAVEL) {
        s_status.fault = DOG_OBSTACLE_FAULT_NONE;
        s_status.can_exit = 1U;
        obstacle_set_state(DOG_OBSTACLE_UNAVAILABLE);
        return;
    }

    if (s_status.state == DOG_OBSTACLE_DISABLED) {
        obstacle_set_state(DOG_OBSTACLE_PRECHECK);
    }
    if (s_status.state == DOG_OBSTACLE_PRECHECK) {
        if (dog_foot_motion_prepare() == 0U) {
            s_status.fault = DOG_OBSTACLE_FAULT_PRECHECK;
            return;
        }
        const uint8_t geometry_valid =
            (s_status.selected == DOG_OBSTACLE_STAIRS) ?
                stair_geometry_valid() : bridge_geometry_valid();
        if (geometry_valid == 0U) {
            obstacle_fault(DOG_OBSTACLE_FAULT_PRECHECK);
            return;
        }
        s_status.fault = DOG_OBSTACLE_FAULT_NONE;
        s_status.motion_state = DOG_FOOT_MOTION_IDLE;
        obstacle_set_state(DOG_OBSTACLE_READY);
    }

    if (s_status.state == DOG_OBSTACLE_FAULT) {
        if (requested_step < 0) {
            if (obstacle_reserve_motion_start(request_epoch) == 0U) {
                return;
            }
            if (s_status.selected == DOG_OBSTACLE_STAIRS) {
                if ((s_stair.sequence_count == 0U) &&
                    (s_stair.current_phase == DOG_STAIR_PHASE_IDLE)) {
                    s_status.fault = DOG_OBSTACLE_FAULT_NONE;
                    s_status.motion_state = DOG_FOOT_MOTION_IDLE;
                    obstacle_set_state(DOG_OBSTACLE_PRECHECK);
                } else {
                    (void)stair_build_recovery_sequence();
                }
            } else {
                if ((s_motion_kind == BRIDGE_MOTION_NONE) &&
                    (s_status.prepared == 0U)) {
                    s_status.fault = DOG_OBSTACLE_FAULT_NONE;
                    obstacle_set_state(DOG_OBSTACLE_PRECHECK);
                    return;
                }
                if (bridge_recover_interrupted_motion() == 0U) {
                    return;
                }
                if ((s_status.state != DOG_OBSTACLE_MOVING) &&
                    (s_status.state != DOG_OBSTACLE_ROLLBACK)) {
                    s_status.fault = DOG_OBSTACLE_FAULT_NONE;
                    obstacle_set_state(DOG_OBSTACLE_READY);
                    bridge_backward_step();
                }
            }
        }
        return;
    }

    if (s_status.state != DOG_OBSTACLE_READY) {
        return;
    }
    if ((s_status.selected == DOG_OBSTACLE_STAIRS) &&
        (s_stair.sequence_count != 0U)) {
        stair_continue_sequence(now_ms, stair_mode_requested);
        return;
    }
    if (requested_step > 0) {
        if (obstacle_reserve_motion_start(request_epoch) == 0U) {
            return;
        }
        if (s_status.selected == DOG_OBSTACLE_STAIRS) {
            if (s_status.completed_levels >= DOG_OBSTACLE_STAIR_LEVEL_COUNT) {
                return;
            }
            if ((stair_build_forward_sequence() == 0U) ||
                (stair_start_current_segment(1) == 0U)) {
                if (s_status.state != DOG_OBSTACLE_FAULT) {
                    obstacle_fault(DOG_OBSTACLE_FAULT_MOTION_START);
                }
            }
        } else {
            bridge_forward_step();
        }
    } else if (requested_step < 0) {
        if (s_status.selected == DOG_OBSTACLE_STAIRS) {
            return;
        }
        if (s_status.prepared == 0U) {
            return;
        }
        if (obstacle_reserve_motion_start(request_epoch) == 0U) {
            return;
        }
        bridge_backward_step();
    }
}

void DogObstacle_GetStatus(DogObstacleStatus *status)
{
    if (status == nullptr) {
        return;
    }
    status_write_begin();
    *status = s_published_status;
    status_write_end();
}

uint8_t DogObstacle_CanExit(void)
{
    uint8_t can_exit = 0U;
    status_write_begin();
    can_exit = s_published_status.can_exit;
    status_write_end();
    return can_exit;
}

const char *DogObstacle_TypeName(uint8_t obstacle)
{
    switch (obstacle) {
    case DOG_OBSTACLE_BRIDGE_B: return "BRIDGE_B";
    case DOG_OBSTACLE_STAIRS:   return "STAIRS";
    case DOG_OBSTACLE_GRAVEL:   return "GRAVEL";
    default:                    return "UNKNOWN";
    }
}

const char *DogObstacle_StateName(uint8_t state)
{
    switch (state) {
    case DOG_OBSTACLE_DISABLED:    return "DISABLED";
    case DOG_OBSTACLE_PRECHECK:    return "PRECHECK";
    case DOG_OBSTACLE_READY:       return "READY";
    case DOG_OBSTACLE_MOVING:      return "MOVING";
    case DOG_OBSTACLE_ROLLBACK:    return "ROLLBACK";
    case DOG_OBSTACLE_UNAVAILABLE: return "UNAVAILABLE";
    case DOG_OBSTACLE_FAULT:       return "FAULT";
    default:                       return "UNKNOWN";
    }
}

const char *DogObstacle_StairPhaseName(uint8_t phase)
{
    switch (phase) {
    case DOG_STAIR_PHASE_IDLE:                    return "IDLE";
    case DOG_STAIR_PHASE_PREP_BODY_SHIFT:         return "PREP_BODY_SHIFT";
    case DOG_STAIR_PHASE_PREP_LEFT_REAR_COMPACT:  return "PREP_LEFT_REAR_COMPACT";
    case DOG_STAIR_PHASE_PREP_REAR_COMPACT:       return "PREP_REAR_COMPACT";
    case DOG_STAIR_PHASE_LEVEL_READY:             return "LEVEL_READY";
    case DOG_STAIR_PHASE_LEFT_FRONT_LIFT:         return "LEFT_FRONT_LIFT";
    case DOG_STAIR_PHASE_LEFT_FRONT_FORWARD:      return "LEFT_FRONT_FORWARD";
    case DOG_STAIR_PHASE_LEFT_FRONT_LAND:         return "LEFT_FRONT_LAND";
    case DOG_STAIR_PHASE_RIGHT_FRONT_LIFT:        return "RIGHT_FRONT_LIFT";
    case DOG_STAIR_PHASE_RIGHT_FRONT_FORWARD:     return "RIGHT_FRONT_FORWARD";
    case DOG_STAIR_PHASE_RIGHT_FRONT_LAND:        return "RIGHT_FRONT_LAND";
    case DOG_STAIR_PHASE_BODY_SHIFT:              return "BODY_SHIFT";
    case DOG_STAIR_PHASE_LEFT_REAR_LIFT:          return "LEFT_REAR_LIFT";
    case DOG_STAIR_PHASE_LEFT_REAR_FORWARD:       return "LEFT_REAR_FORWARD";
    case DOG_STAIR_PHASE_LEFT_REAR_LAND:          return "LEFT_REAR_LAND";
    case DOG_STAIR_PHASE_RIGHT_REAR_LIFT:         return "RIGHT_REAR_LIFT";
    case DOG_STAIR_PHASE_RIGHT_REAR_FORWARD:      return "RIGHT_REAR_FORWARD";
    case DOG_STAIR_PHASE_RIGHT_REAR_LAND:         return "RIGHT_REAR_LAND";
    case DOG_STAIR_PHASE_BODY_RAISE:              return "BODY_RAISE";
    case DOG_STAIR_PHASE_TOP_LEFT_FRONT_ADVANCE:  return "TOP_LEFT_FRONT_ADVANCE";
    case DOG_STAIR_PHASE_TOP_RIGHT_FRONT_ADVANCE: return "TOP_RIGHT_FRONT_ADVANCE";
    case DOG_STAIR_PHASE_TOP_BODY_SHIFT:          return "TOP_BODY_SHIFT";
    case DOG_STAIR_PHASE_TOP_LEFT_REAR_ADVANCE:   return "TOP_LEFT_REAR_ADVANCE";
    case DOG_STAIR_PHASE_TOP_RIGHT_REAR_ADVANCE:  return "TOP_RIGHT_REAR_ADVANCE";
    case DOG_STAIR_PHASE_TOP_BODY_NORMALIZE:      return "TOP_BODY_NORMALIZE";
    case DOG_STAIR_PHASE_TOP_LEFT_FRONT_NORMAL:   return "TOP_LEFT_FRONT_NORMAL";
    case DOG_STAIR_PHASE_TOP_RIGHT_FRONT_NORMAL:  return "TOP_RIGHT_FRONT_NORMAL";
    case DOG_STAIR_PHASE_TOP_READY:               return "TOP_READY";
    case DOG_STAIR_PHASE_RECOVERY:                return "RECOVERY";
    default:                                      return "UNKNOWN";
    }
}
