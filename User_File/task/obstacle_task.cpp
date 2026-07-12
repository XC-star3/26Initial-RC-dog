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

static constexpr uint8_t kFrontLegMask =
    (uint8_t)((1U << DOG_LEG_LF) | (1U << DOG_LEG_RF));
static constexpr uint8_t kRearLegMask =
    (uint8_t)((1U << DOG_LEG_LB) | (1U << DOG_LEG_RB));
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
    s_status.fault = fault;
    s_status.motion_state = DOG_FOOT_MOTION_FAULT;
    s_status.can_exit = ((s_status.prepared == 0U) &&
                         (s_motion_kind == BRIDGE_MOTION_NONE)) ? 1U : 0U;
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
    obstacle_set_state(DOG_OBSTACLE_FAULT);
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
        if (bridge_recover_interrupted_motion() == 0U) {
            return;
        }
        if ((s_status.state == DOG_OBSTACLE_MOVING) ||
            (s_status.state == DOG_OBSTACLE_ROLLBACK)) {
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
        (void)bridge_start_unprepare();
    } else {
        bridge_backward_step();
    }
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
    s_geometry_ready = 0U;
    s_motion_kind = BRIDGE_MOTION_NONE;
    s_motion_source_checkpoint = kCheckpointUnprepared;
    s_motion_direction = 0;
    s_recovery_leg_mask = DOG_LEG_MASK_ALL;
    s_recovery_clearance_mm = 0.0f;
    s_recovery_duration_ms = 0U;
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
    (void)now_ms;
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

    const uint8_t motion_was_active =
        ((s_status.state == DOG_OBSTACLE_MOVING) ||
         (s_status.state == DOG_OBSTACLE_ROLLBACK)) ? 1U : 0U;
    if (motion_was_active != 0U) {
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

    if ((s_status.prepared == 0U) &&
        (s_status.state != DOG_OBSTACLE_MOVING) &&
        (s_status.state != DOG_OBSTACLE_ROLLBACK)) {
        if (s_status.selected != requested_selection) {
            s_status.selected = requested_selection;
            s_status.fault = DOG_OBSTACLE_FAULT_NONE;
            obstacle_set_state(DOG_OBSTACLE_DISABLED);
        }
    }

    if (requested_mode == 0U) {
        if (s_status.selected == DOG_OBSTACLE_BRIDGE_B) {
            bridge_exit_tick();
        } else {
            s_status.can_exit = 1U;
            obstacle_set_state(DOG_OBSTACLE_DISABLED);
        }
        return;
    }

    s_status.can_exit = ((s_status.prepared == 0U) &&
                         (s_status.state != DOG_OBSTACLE_MOVING) &&
                         (s_status.state != DOG_OBSTACLE_ROLLBACK)) ? 1U : 0U;
    if (s_status.selected != DOG_OBSTACLE_BRIDGE_B) {
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
        if (bridge_geometry_valid() == 0U) {
            obstacle_fault(DOG_OBSTACLE_FAULT_PRECHECK);
            return;
        }
        s_status.fault = DOG_OBSTACLE_FAULT_NONE;
        s_status.motion_state = DOG_FOOT_MOTION_IDLE;
        obstacle_set_state(DOG_OBSTACLE_READY);
    }

    if (s_status.state == DOG_OBSTACLE_FAULT) {
        if (requested_step < 0) {
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
        return;
    }

    if (s_status.state != DOG_OBSTACLE_READY) {
        return;
    }
    if (requested_step > 0) {
        if (obstacle_reserve_motion_start(request_epoch) == 0U) {
            return;
        }
        bridge_forward_step();
    } else if (requested_step < 0) {
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
