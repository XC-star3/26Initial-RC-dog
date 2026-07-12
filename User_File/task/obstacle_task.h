#ifndef OBSTACLE_TASK_H
#define OBSTACLE_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum Dog_Obstacle_Type {
    DOG_OBSTACLE_BRIDGE_B = 0U,
    DOG_OBSTACLE_STAIRS = 1U,
    DOG_OBSTACLE_GRAVEL = 2U,
};

enum Dog_Obstacle_State {
    DOG_OBSTACLE_DISABLED = 0U,
    DOG_OBSTACLE_PRECHECK,
    DOG_OBSTACLE_READY,
    DOG_OBSTACLE_MOVING,
    DOG_OBSTACLE_ROLLBACK,
    DOG_OBSTACLE_UNAVAILABLE,
    DOG_OBSTACLE_FAULT,
};

enum Dog_Obstacle_Fault {
    DOG_OBSTACLE_FAULT_NONE = 0U,
    DOG_OBSTACLE_FAULT_PRECHECK,
    DOG_OBSTACLE_FAULT_MOTION_START,
    DOG_OBSTACLE_FAULT_MOTION_RUNTIME,
    DOG_OBSTACLE_FAULT_SAFETY,
};

enum Dog_Stair_Phase {
    DOG_STAIR_PHASE_IDLE = 0U,
    DOG_STAIR_PHASE_PREP_BODY_SHIFT,
    DOG_STAIR_PHASE_PREP_REAR_COMPACT,
    DOG_STAIR_PHASE_LEVEL_READY,
    DOG_STAIR_PHASE_FRONT_LIFT,
    DOG_STAIR_PHASE_FRONT_FORWARD,
    DOG_STAIR_PHASE_FRONT_LAND,
    DOG_STAIR_PHASE_BODY_SHIFT,
    DOG_STAIR_PHASE_REAR_LIFT,
    DOG_STAIR_PHASE_REAR_FORWARD,
    DOG_STAIR_PHASE_REAR_LAND,
    DOG_STAIR_PHASE_BODY_RAISE,
    DOG_STAIR_PHASE_TOP_FRONT_ADVANCE,
    DOG_STAIR_PHASE_TOP_BODY_SHIFT,
    DOG_STAIR_PHASE_TOP_REAR_ADVANCE,
    DOG_STAIR_PHASE_TOP_BODY_NORMALIZE,
    DOG_STAIR_PHASE_TOP_FRONT_NORMAL,
    DOG_STAIR_PHASE_TOP_READY,
    DOG_STAIR_PHASE_RECOVERY,
};

struct DogObstacleStatus {
    uint8_t mode_requested;
    uint8_t selected;
    uint8_t state;
    uint8_t fault;
    uint8_t prepared;
    uint8_t checkpoint;
    uint8_t target_checkpoint;
    uint8_t motion_state;
    uint8_t can_exit;
    uint16_t completed_gaps;
    float active_gap_mm;
    float final_gap_mm;
    uint8_t phase;
    uint8_t completed_levels;
    uint8_t total_levels;
    float step_height_mm;
    float tread_depth_mm;
};

void DogObstacle_Init(void);
void DogObstacle_SetModeRequested(uint8_t active);
void DogObstacle_Select(uint8_t obstacle);
void DogObstacle_RequestStep(int8_t direction);
void DogObstacle_RequestSafetyAbort(void);
void DogObstacle_Tick(uint32_t now_ms);
void DogObstacle_GetStatus(DogObstacleStatus *status);
uint8_t DogObstacle_CanExit(void);
const char *DogObstacle_TypeName(uint8_t obstacle);
const char *DogObstacle_StateName(uint8_t state);
const char *DogObstacle_StairPhaseName(uint8_t phase);

#ifdef __cplusplus
}
#endif

#endif
