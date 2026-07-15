#include "obstacle_task.h"

#include "debug_uart.h"
#include "motor_task.h"
#include "obstacle_config.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

static_assert(DOG_OBSTACLE_STAIR_LOW_LANDING_MM > 0.0f,
              "Low stair landing height must be positive");
static_assert(DOG_OBSTACLE_STAIR_MID_LANDING_MM > 0.0f,
              "Mid stair landing height must be positive");
static_assert(DOG_OBSTACLE_STAIR_HIGH_LANDING_MM > 0.0f,
              "High stair landing height must be positive");
static_assert(DOG_OBSTACLE_STAIR_LOW_SWING_PEAK_MM >=
                  DOG_OBSTACLE_STAIR_LOW_LANDING_MM,
              "Low stair swing peak must reach the landing");
static_assert(DOG_OBSTACLE_STAIR_MID_SWING_PEAK_MM >=
                  DOG_OBSTACLE_STAIR_MID_LANDING_MM,
              "Mid stair swing peak must reach the landing");
static_assert(DOG_OBSTACLE_STAIR_HIGH_SWING_PEAK_MM >=
                  DOG_OBSTACLE_STAIR_HIGH_LANDING_MM,
              "High stair swing peak must reach the landing");
static_assert(DOG_OBSTACLE_STAIR_TREAD_MM >
                  (2.0f * DOG_OBSTACLE_STAIR_EDGE_MARGIN_MM),
              "Stair tread is too short for the wheel edge margins");
static_assert(DOG_OBSTACLE_STAIR_LEVEL_COUNT == 1U,
              "The obstacle task supports one stair level per session");
static_assert(DOG_OBSTACLE_STAIR_WIDTH_MM > 0.0f,
              "Stair width must be positive");
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

static constexpr uint8_t kLeftFrontLegMask = (uint8_t)(1U << DOG_LEG_LF);
static constexpr uint8_t kRightFrontLegMask = (uint8_t)(1U << DOG_LEG_RF);
static constexpr uint8_t kLeftRearLegMask = (uint8_t)(1U << DOG_LEG_LB);
static constexpr uint8_t kRightRearLegMask = (uint8_t)(1U << DOG_LEG_RB);

static volatile uint8_t s_requested_mode = 0U;
static volatile uint8_t s_requested_step = 0U;
static volatile uint8_t s_requested_safety_abort = 0U;
static volatile uint8_t s_requested_profile = DOG_STAIR_PROFILE_MID;
static volatile uint32_t s_request_epoch = 0U;

static DogObstacleStatus s_status = {};
static DogObstacleStatus s_published_status = {};

static constexpr uint8_t kStairWaypointCount =
    (uint8_t)(DOG_STAIR_PHASE_TOP_READY + 1U);
static constexpr uint8_t kStairMaxSegments = 28U;
static_assert(kStairMaxSegments >= 26U,
              "Stair sequence capacity is too small for high-profile ascent");

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
    uint8_t motion_target_phase;
    uint8_t sequence_count;
    uint8_t sequence_index;
    uint8_t motion_active;
    uint8_t paused;
    uint32_t dwell_started_ms;
    uint32_t dwell_duration_ms;
};

static StairContext s_stair = {};

static uint8_t stair_profile_sanitize(uint8_t profile)
{
    if ((profile == DOG_STAIR_PROFILE_LOW) ||
        (profile == DOG_STAIR_PROFILE_MID) ||
        (profile == DOG_STAIR_PROFILE_HIGH)) {
        return profile;
    }
    return DOG_STAIR_PROFILE_MID;
}

static void stair_apply_profile(uint8_t profile)
{
    s_status.profile = stair_profile_sanitize(profile);
    switch (s_status.profile) {
    case DOG_STAIR_PROFILE_LOW:
        s_status.landing_height_mm = DOG_OBSTACLE_STAIR_LOW_LANDING_MM;
        s_status.swing_peak_mm = DOG_OBSTACLE_STAIR_LOW_SWING_PEAK_MM;
        s_status.body_preraise_mm = DOG_OBSTACLE_STAIR_LOW_BODY_RAISE_MM;
        break;
    case DOG_STAIR_PROFILE_HIGH:
        s_status.landing_height_mm = DOG_OBSTACLE_STAIR_HIGH_LANDING_MM;
        s_status.swing_peak_mm = DOG_OBSTACLE_STAIR_HIGH_SWING_PEAK_MM;
        s_status.body_preraise_mm = DOG_OBSTACLE_STAIR_HIGH_BODY_RAISE_MM;
        break;
    case DOG_STAIR_PROFILE_MID:
    default:
        s_status.landing_height_mm = DOG_OBSTACLE_STAIR_MID_LANDING_MM;
        s_status.swing_peak_mm = DOG_OBSTACLE_STAIR_MID_SWING_PEAK_MM;
        s_status.body_preraise_mm = DOG_OBSTACLE_STAIR_MID_BODY_RAISE_MM;
        break;
    }
}

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
    const float front_normal_z = stair_front_normal_z();
    const float rear_normal_z = stair_rear_normal_z();
    const float front_support_z = front_normal_z + s_status.body_preraise_mm;
    const float rear_support_z = rear_normal_z + s_status.body_preraise_mm;
    const float front_upper_z = front_support_z - s_status.landing_height_mm;
    const float rear_upper_z = rear_support_z - s_status.landing_height_mm;
    const float front_lift_z = front_support_z - s_status.swing_peak_mm;
    const float rear_lift_z = rear_support_z - s_status.swing_peak_mm;
    const float compact = DOG_OBSTACLE_STAIR_COMPACT_X_MM;
    const float front_forward = DOG_OBSTACLE_STAIR_FRONT_FORWARD_X_MM;
    const float rear_shift = DOG_OBSTACLE_STAIR_REAR_SHIFT_X_MM;

    stair_set_waypoint(DOG_STAIR_PHASE_IDLE,
                       0.0f, front_normal_z, 0.0f, rear_normal_z);
    stair_set_waypoint(DOG_STAIR_PHASE_PREP_BODY_RAISE,
                       0.0f, front_support_z, 0.0f, rear_support_z);
    stair_set_waypoint(DOG_STAIR_PHASE_PREP_BODY_SHIFT,
                       -compact, front_support_z, -compact, rear_support_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_PREP_LEFT_REAR_COMPACT,
                        DOG_STAIR_PHASE_PREP_BODY_SHIFT);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_PREP_LEFT_REAR_COMPACT,
                           DOG_LEG_LB, compact, rear_support_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_PREP_REAR_COMPACT,
                        DOG_STAIR_PHASE_PREP_LEFT_REAR_COMPACT);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_PREP_REAR_COMPACT,
                           DOG_LEG_RB, compact, rear_support_z);
    stair_set_waypoint(DOG_STAIR_PHASE_LEVEL_READY,
                       -compact, front_support_z, compact, rear_support_z);

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
                       -compact, front_upper_z, rear_shift, rear_support_z);
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
                       -compact, front_normal_z, compact, rear_normal_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_TOP_LEFT_FRONT_ADVANCE,
                        DOG_STAIR_PHASE_BODY_RAISE);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_TOP_LEFT_FRONT_ADVANCE,
                           DOG_LEG_LF, front_forward, front_normal_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_TOP_RIGHT_FRONT_ADVANCE,
                        DOG_STAIR_PHASE_TOP_LEFT_FRONT_ADVANCE);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_TOP_RIGHT_FRONT_ADVANCE,
                           DOG_LEG_RF, front_forward, front_normal_z);
    stair_set_waypoint(DOG_STAIR_PHASE_TOP_BODY_SHIFT,
                       -compact, front_normal_z, rear_shift, rear_normal_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_TOP_LEFT_REAR_ADVANCE,
                        DOG_STAIR_PHASE_TOP_BODY_SHIFT);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_TOP_LEFT_REAR_ADVANCE,
                           DOG_LEG_LB, compact, rear_normal_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_TOP_RIGHT_REAR_ADVANCE,
                        DOG_STAIR_PHASE_TOP_LEFT_REAR_ADVANCE);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_TOP_RIGHT_REAR_ADVANCE,
                           DOG_LEG_RB, compact, rear_normal_z);
    stair_set_waypoint(DOG_STAIR_PHASE_TOP_BODY_NORMALIZE,
                       -(2.0f * compact), front_normal_z, 0.0f, rear_normal_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_TOP_LEFT_FRONT_NORMAL,
                        DOG_STAIR_PHASE_TOP_BODY_NORMALIZE);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_TOP_LEFT_FRONT_NORMAL,
                           DOG_LEG_LF, 0.0f, front_normal_z);
    stair_copy_waypoint(DOG_STAIR_PHASE_TOP_RIGHT_FRONT_NORMAL,
                        DOG_STAIR_PHASE_TOP_LEFT_FRONT_NORMAL);
    stair_set_leg_waypoint(DOG_STAIR_PHASE_TOP_RIGHT_FRONT_NORMAL,
                           DOG_LEG_RF, 0.0f, front_normal_z);
    stair_set_waypoint(DOG_STAIR_PHASE_TOP_READY,
                       0.0f, front_normal_z, 0.0f, rear_normal_z);
    s_stair.geometry_ready = 1U;
}

static void stair_context_reset(void)
{
    memset(&s_stair, 0, sizeof(s_stair));
    stair_build_geometry();
    s_stair.current_phase = DOG_STAIR_PHASE_IDLE;
    s_stair.motion_target_phase = DOG_STAIR_PHASE_IDLE;
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
    DebugUart_Printf("STAIR state=%s phase=%s target=%s level=%u/%u.\r\n",
                     DogObstacle_StateName(state),
                     DogObstacle_StairPhaseName(s_status.phase),
                     DogObstacle_StairPhaseName(s_status.target_phase),
                     (unsigned)s_status.completed_levels,
                     (unsigned)s_status.total_levels);
}

static void obstacle_fault(uint8_t fault)
{
    if ((s_status.state == DOG_OBSTACLE_FAULT) &&
        (s_status.fault == fault) &&
        (s_status.motion_state == DOG_FOOT_MOTION_FAULT)) {
        return;
    }
    dog_foot_motion_cancel();
    s_stair.motion_active = 0U;
    s_stair.paused = 0U;
    s_stair.dwell_started_ms = 0U;
    s_stair.dwell_duration_ms = 0U;
    s_status.phase = s_stair.current_phase;
    s_status.target_phase = s_stair.current_phase;
    s_status.fault = fault;
    s_status.motion_state = DOG_FOOT_MOTION_FAULT;
    s_status.can_exit = ((s_stair.current_phase == DOG_STAIR_PHASE_IDLE) &&
                         (s_stair.sequence_count == 0U)) ? 1U : 0U;
    obstacle_set_state(DOG_OBSTACLE_FAULT);
}

static void obstacle_control_abort(uint8_t fault)
{
    if ((s_status.state == DOG_OBSTACLE_FAULT) &&
        (s_status.fault == fault) && (s_status.prepared == 0U) &&
        (s_status.can_exit != 0U)) {
        return;
    }
    dog_foot_motion_cancel();
    s_status.fault = fault;
    s_status.motion_state = DOG_FOOT_MOTION_FAULT;
    s_status.prepared = 0U;
    s_status.can_exit = 1U;
    s_status.phase = DOG_STAIR_PHASE_IDLE;
    s_status.target_phase = DOG_STAIR_PHASE_IDLE;
    s_status.completed_levels = 0U;
    stair_context_reset();
    obstacle_set_state(DOG_OBSTACLE_FAULT);
}

static void obstacle_reset_session(uint8_t profile)
{
    dog_foot_motion_cancel();
    s_status.fault = DOG_OBSTACLE_FAULT_NONE;
    s_status.prepared = 0U;
    s_status.motion_state = DOG_FOOT_MOTION_IDLE;
    s_status.can_exit = 1U;
    s_status.phase = DOG_STAIR_PHASE_IDLE;
    s_status.target_phase = DOG_STAIR_PHASE_IDLE;
    s_status.completed_levels = 0U;
    s_status.total_levels = DOG_OBSTACLE_STAIR_LEVEL_COUNT;
    s_status.tread_depth_mm = DOG_OBSTACLE_STAIR_TREAD_MM;
    stair_apply_profile(profile);
    stair_context_reset();
    obstacle_set_state(DOG_OBSTACLE_DISABLED);
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
        {DOG_STAIR_PHASE_IDLE, DOG_STAIR_PHASE_PREP_BODY_RAISE,
         DOG_LEG_MASK_ALL, 0.0f},
        {DOG_STAIR_PHASE_PREP_BODY_RAISE, DOG_STAIR_PHASE_PREP_BODY_SHIFT,
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
    for (uint8_t i = 0U;
         i < (uint8_t)(sizeof(checks) / sizeof(checks[0])); ++i) {
        if (stair_path_valid(checks[i].source, checks[i].target,
                             checks[i].mask, checks[i].clearance) == 0U) {
            DebugUart_Printf("STAIR precheck FAIL path=%u->%u profile=%u.\r\n",
                             (unsigned)checks[i].source,
                             (unsigned)checks[i].target,
                             (unsigned)s_status.profile);
            return 0U;
        }
    }
    return 1U;
}

static void stair_sequence_clear(void)
{
    memset(s_stair.sequence, 0, sizeof(s_stair.sequence));
    s_stair.sequence_count = 0U;
    s_stair.sequence_index = 0U;
    s_stair.motion_active = 0U;
    s_stair.paused = 0U;
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
    if ((s_status.completed_levels != 0U) ||
        (s_stair.current_phase != DOG_STAIR_PHASE_IDLE)) {
        return 0U;
    }
    stair_sequence_clear();

    uint8_t prepare_source = DOG_STAIR_PHASE_IDLE;
    if (s_status.body_preraise_mm > 0.0f) {
        if (stair_sequence_add(DOG_STAIR_PHASE_IDLE,
                               DOG_STAIR_PHASE_PREP_BODY_RAISE,
                               DOG_LEG_MASK_ALL,
                               DOG_OBSTACLE_STAIR_BODY_PRERAISE_MS,
                               0.0f, 0U) == 0U) {
            return 0U;
        }
        prepare_source = DOG_STAIR_PHASE_PREP_BODY_RAISE;
    }
    if ((stair_sequence_add(prepare_source,
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
                            DOG_OBSTACLE_STAIR_DWELL_MS) == 0U) ||
        (stair_add_level_sequence(DOG_STAIR_PHASE_PREP_REAR_COMPACT) == 0U) ||
        (stair_add_top_sequence() == 0U)) {
        stair_sequence_clear();
        return 0U;
    }
    return 1U;
}

static uint8_t stair_start_current_segment(void)
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
    s_stair.motion_target_phase = segment->target_phase;
    s_status.prepared = 1U;
    s_status.target_phase = segment->target_phase;
    s_status.motion_state = DOG_FOOT_MOTION_ACTIVE;
    s_status.can_exit = 0U;
    obstacle_set_state(DOG_OBSTACLE_MOVING);
    return 1U;
}

static void stair_sequence_finish(void)
{
    stair_sequence_clear();
    s_stair.current_phase = DOG_STAIR_PHASE_TOP_READY;
    s_stair.motion_target_phase = DOG_STAIR_PHASE_TOP_READY;
    s_status.fault = DOG_OBSTACLE_FAULT_NONE;
    s_status.motion_state = DOG_FOOT_MOTION_IDLE;
    s_status.phase = DOG_STAIR_PHASE_TOP_READY;
    s_status.target_phase = DOG_STAIR_PHASE_TOP_READY;
    s_status.completed_levels = DOG_OBSTACLE_STAIR_LEVEL_COUNT;
    s_status.prepared = 0U;
    s_status.can_exit = 1U;
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
        s_status.target_phase = s_stair.current_phase;
        s_status.motion_state = DOG_FOOT_MOTION_IDLE;
        s_status.can_exit = 0U;
        obstacle_set_state(DOG_OBSTACLE_READY);
        return;
    }
    s_stair.paused = 0U;
    (void)stair_start_current_segment();
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
    s_status.target_phase = s_stair.current_phase;
    s_status.motion_state = DOG_FOOT_MOTION_IDLE;
    s_stair.sequence_index++;
    s_stair.dwell_started_ms = 0U;
    s_stair.dwell_duration_ms = segment->dwell_ms;
    stair_continue_sequence(now_ms, mode_requested);
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
    s_requested_step = 0U;
    s_requested_safety_abort = 0U;
    s_requested_profile = DOG_STAIR_PROFILE_MID;
    s_request_epoch++;
    taskEXIT_CRITICAL();

    memset(&s_status, 0, sizeof(s_status));
    s_status.state = DOG_OBSTACLE_DISABLED;
    s_status.can_exit = 1U;
    s_status.phase = DOG_STAIR_PHASE_IDLE;
    s_status.target_phase = DOG_STAIR_PHASE_IDLE;
    s_status.total_levels = DOG_OBSTACLE_STAIR_LEVEL_COUNT;
    s_status.tread_depth_mm = DOG_OBSTACLE_STAIR_TREAD_MM;
    stair_apply_profile(DOG_STAIR_PROFILE_MID);
    stair_context_reset();
    obstacle_publish_status();
}

void DogObstacle_SetModeRequested(uint8_t active)
{
    const uint8_t requested = (active != 0U) ? 1U : 0U;
    taskENTER_CRITICAL();
    if (s_requested_mode != requested) {
        s_requested_mode = requested;
        s_requested_step = 0U;
        s_request_epoch++;
    }
    taskEXIT_CRITICAL();
}

void DogObstacle_SetStairProfile(uint8_t profile)
{
    const uint8_t requested = stair_profile_sanitize(profile);
    taskENTER_CRITICAL();
    if ((s_requested_mode == 0U) && (s_requested_profile != requested)) {
        s_requested_profile = requested;
        s_requested_step = 0U;
        s_request_epoch++;
    }
    taskEXIT_CRITICAL();
}

void DogObstacle_RequestStep(void)
{
    taskENTER_CRITICAL();
    if ((s_requested_mode != 0U) && (s_requested_step == 0U)) {
        s_requested_step = 1U;
    }
    taskEXIT_CRITICAL();
}

void DogObstacle_RequestSafetyAbort(void)
{
    taskENTER_CRITICAL();
    s_requested_safety_abort = 1U;
    s_requested_mode = 0U;
    s_requested_step = 0U;
    s_request_epoch++;
    taskEXIT_CRITICAL();
}

void DogObstacle_Tick(uint32_t now_ms)
{
    uint8_t requested_mode = 0U;
    uint8_t requested_step = 0U;
    uint8_t requested_safety_abort = 0U;
    uint8_t requested_profile = DOG_STAIR_PROFILE_MID;
    uint32_t request_epoch = 0U;
    ObstacleStatusPublishGuard publish_guard;

    taskENTER_CRITICAL();
    requested_mode = s_requested_mode;
    requested_step = s_requested_step;
    requested_safety_abort = s_requested_safety_abort;
    requested_profile = s_requested_profile;
    request_epoch = s_request_epoch;
    s_requested_step = 0U;
    s_requested_safety_abort = 0U;
    taskEXIT_CRITICAL();

    const uint8_t entering = ((requested_mode != 0U) &&
                              (s_status.mode_requested == 0U)) ? 1U : 0U;
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

    if ((entering != 0U) && (s_status.can_exit != 0U)) {
        obstacle_reset_session(requested_profile);
        s_status.mode_requested = requested_mode;
    }

    if ((s_stair.sequence_count != 0U) &&
        (s_status.state != DOG_OBSTACLE_FAULT)) {
        requested_step = 0U;
        if (s_stair.motion_active != 0U) {
            s_status.motion_state = dog_foot_motion_state();
            if (s_status.motion_state == DOG_FOOT_MOTION_COMPLETE) {
                stair_motion_complete(now_ms, requested_mode);
            } else if (s_status.motion_state == DOG_FOOT_MOTION_FAULT) {
                obstacle_fault(DOG_OBSTACLE_FAULT_MOTION_RUNTIME);
            } else if (s_status.motion_state == DOG_FOOT_MOTION_ACTIVE) {
                return;
            } else {
                obstacle_fault(DOG_OBSTACLE_FAULT_MOTION_RUNTIME);
            }
        } else {
            stair_continue_sequence(now_ms, requested_mode);
        }
    }

    if (requested_mode == 0U) {
        stair_exit_tick();
        return;
    }

    s_status.can_exit = ((s_status.prepared == 0U) &&
                         (s_status.state != DOG_OBSTACLE_MOVING)) ? 1U : 0U;

    if (s_status.state == DOG_OBSTACLE_DISABLED) {
        obstacle_set_state(DOG_OBSTACLE_PRECHECK);
    }
    if (s_status.state == DOG_OBSTACLE_PRECHECK) {
        if (dog_foot_motion_prepare() == 0U) {
            s_status.fault = DOG_OBSTACLE_FAULT_PRECHECK;
            return;
        }
        if (stair_geometry_valid() == 0U) {
            obstacle_fault(DOG_OBSTACLE_FAULT_PRECHECK);
            return;
        }
        s_status.fault = DOG_OBSTACLE_FAULT_NONE;
        s_status.motion_state = DOG_FOOT_MOTION_IDLE;
        obstacle_set_state(DOG_OBSTACLE_READY);
    }

    if (s_status.state == DOG_OBSTACLE_FAULT) {
        return;
    }
    if (s_status.state != DOG_OBSTACLE_READY) {
        return;
    }
    if (s_stair.sequence_count != 0U) {
        stair_continue_sequence(now_ms, requested_mode);
        return;
    }
    if (requested_step == 0U) {
        return;
    }
    if (s_status.completed_levels >= DOG_OBSTACLE_STAIR_LEVEL_COUNT) {
        return;
    }
    if (obstacle_reserve_motion_start(request_epoch) == 0U) {
        return;
    }
    if ((stair_build_forward_sequence() == 0U) ||
        (stair_start_current_segment() == 0U)) {
        if (s_status.state != DOG_OBSTACLE_FAULT) {
            obstacle_fault(DOG_OBSTACLE_FAULT_MOTION_START);
        }
    }
}

void DogObstacle_GetStatus(DogObstacleStatus *status)
{
    if (status == nullptr) {
        return;
    }
    taskENTER_CRITICAL();
    *status = s_published_status;
    taskEXIT_CRITICAL();
}

uint8_t DogObstacle_CanExit(void)
{
    uint8_t can_exit = 0U;
    taskENTER_CRITICAL();
    can_exit = s_published_status.can_exit;
    taskEXIT_CRITICAL();
    return can_exit;
}

const char *DogObstacle_StateName(uint8_t state)
{
    switch (state) {
    case DOG_OBSTACLE_DISABLED: return "DISABLED";
    case DOG_OBSTACLE_PRECHECK: return "PRECHECK";
    case DOG_OBSTACLE_READY:    return "READY";
    case DOG_OBSTACLE_MOVING:   return "MOVING";
    case DOG_OBSTACLE_FAULT:    return "FAULT";
    default:                    return "UNKNOWN";
    }
}

const char *DogObstacle_StairPhaseName(uint8_t phase)
{
    switch (phase) {
    case DOG_STAIR_PHASE_IDLE:                    return "IDLE";
    case DOG_STAIR_PHASE_PREP_BODY_RAISE:         return "PREP_BODY_RAISE";
    case DOG_STAIR_PHASE_PREP_BODY_SHIFT:         return "PREP_BODY_SHIFT";
    case DOG_STAIR_PHASE_PREP_LEFT_REAR_COMPACT:  return "PREP_LR_COMPACT";
    case DOG_STAIR_PHASE_PREP_REAR_COMPACT:       return "PREP_REAR_COMPACT";
    case DOG_STAIR_PHASE_LEVEL_READY:             return "LEVEL_READY";
    case DOG_STAIR_PHASE_LEFT_FRONT_LIFT:         return "LF_LIFT";
    case DOG_STAIR_PHASE_LEFT_FRONT_FORWARD:      return "LF_FORWARD";
    case DOG_STAIR_PHASE_LEFT_FRONT_LAND:         return "LF_LAND";
    case DOG_STAIR_PHASE_RIGHT_FRONT_LIFT:        return "RF_LIFT";
    case DOG_STAIR_PHASE_RIGHT_FRONT_FORWARD:     return "RF_FORWARD";
    case DOG_STAIR_PHASE_RIGHT_FRONT_LAND:        return "RF_LAND";
    case DOG_STAIR_PHASE_BODY_SHIFT:              return "BODY_SHIFT";
    case DOG_STAIR_PHASE_LEFT_REAR_LIFT:          return "LR_LIFT";
    case DOG_STAIR_PHASE_LEFT_REAR_FORWARD:       return "LR_FORWARD";
    case DOG_STAIR_PHASE_LEFT_REAR_LAND:          return "LR_LAND";
    case DOG_STAIR_PHASE_RIGHT_REAR_LIFT:         return "RR_LIFT";
    case DOG_STAIR_PHASE_RIGHT_REAR_FORWARD:      return "RR_FORWARD";
    case DOG_STAIR_PHASE_RIGHT_REAR_LAND:         return "RR_LAND";
    case DOG_STAIR_PHASE_BODY_RAISE:              return "BODY_RAISE";
    case DOG_STAIR_PHASE_TOP_LEFT_FRONT_ADVANCE:  return "TOP_LF_ADV";
    case DOG_STAIR_PHASE_TOP_RIGHT_FRONT_ADVANCE: return "TOP_RF_ADV";
    case DOG_STAIR_PHASE_TOP_BODY_SHIFT:          return "TOP_BODY_SHIFT";
    case DOG_STAIR_PHASE_TOP_LEFT_REAR_ADVANCE:   return "TOP_LR_ADV";
    case DOG_STAIR_PHASE_TOP_RIGHT_REAR_ADVANCE:  return "TOP_RR_ADV";
    case DOG_STAIR_PHASE_TOP_BODY_NORMALIZE:      return "TOP_BODY_NORM";
    case DOG_STAIR_PHASE_TOP_LEFT_FRONT_NORMAL:   return "TOP_LF_NORMAL";
    case DOG_STAIR_PHASE_TOP_RIGHT_FRONT_NORMAL:  return "TOP_RF_NORMAL";
    case DOG_STAIR_PHASE_TOP_READY:               return "TOP_READY";
    default:                                      return "UNKNOWN";
    }
}
