#pragma once

#include <cstdint>

namespace RCDog
{

inline constexpr char kSbusTopic[] = "rcdog.input.sbus.v1";
inline constexpr char kControlTopic[] = "rcdog.control.command.v1";
inline constexpr char kStatusTopic[] = "rcdog.status.v1";
inline constexpr uint8_t kSchemaVersion = 1;

enum class ControlSource : uint8_t
{
  NONE = 0,
  SBUS = 1,
  USB = 2,
};

enum class RobotMode : uint8_t
{
  MOTOR_CHECK = 0,
  LOW_WHEEL = 1,
  LOW_WHEEL_REVERSE = 2,
  STAND_HOLD = 3,
  STAND_WHEEL = 4,
  STAND_HOLD_ALT = 5,
  GAIT_ONLY = 6,
  GAIT_WHEEL = 7,
  OBSTACLE = 8,
};

enum class EntryState : uint8_t
{
  INACTIVE = 0,
  ENTERING,
  WAIT_LOWER,
  WAIT_MECHANICAL,
  WAIT_GAIT_STOP,
  WAIT_STAND,
  WAIT_OBSTACLE,
  ACTIVE,
  BLOCKED,
};

enum class BlockReason : uint8_t
{
  NONE = 0,
  LOWER,
  MECHANICAL_PERMIT,
  GAIT_STOP,
  STAND,
  WHEEL_FAULT,
  WHEEL_STOP,
  LEG_FAULT,
  OBSTACLE,
  INPUT_TIMEOUT,
  SAFETY_LATCH,
};

enum class StairProfile : uint8_t
{
  LOW = 0,
  MID = 1,
  HIGH = 2,
};

enum class ObstacleState : uint8_t
{
  DISABLED = 0,
  PRECHECK,
  READY,
  MOVING,
  COMPLETE,
  FAULT,
};

enum class ObstacleFault : uint8_t
{
  NONE = 0,
  PRECHECK,
  MOTION_START,
  MOTION_RUNTIME,
  SAFETY,
};

enum ControlFlags : uint16_t
{
  DEADMAN = 1U << 0,
  MOTION_ENABLE = 1U << 1,
  SMOOTH_STOP = 1U << 2,
  KNOWN_FLAGS = DEADMAN | MOTION_ENABLE | SMOOTH_STOP,
};

enum FaultBits : uint32_t
{
  FAULT_NONE = 0,
  FAULT_SBUS_LOST = 1U << 0,
  FAULT_USB_LOST = 1U << 1,
  FAULT_LEG_OFFLINE = 1U << 2,
  FAULT_LEG_DRIVE = 1U << 3,
  FAULT_WHEEL_OFFLINE = 1U << 4,
  FAULT_WHEEL_OVERTEMP = 1U << 5,
  FAULT_CAN_BUS_OFF = 1U << 6,
  FAULT_OBSTACLE = 1U << 7,
  FAULT_SAFETY_LATCHED = 1U << 8,
  FAULT_USB_PROTOCOL = 1U << 9,
};

#pragma pack(push, 1)
struct ControlCommandV1
{
  uint8_t schema_version;
  uint8_t mode;
  uint16_t flags;
  int16_t yaw;
  int16_t forward;
  int16_t speed;
  uint16_t reserved;
  uint32_t session_id;
  uint32_t command_counter;
  uint32_t host_time_ms;
};

struct RobotStatusV1
{
  uint8_t schema_version;
  uint8_t control_source;
  uint8_t requested_mode;
  uint8_t active_mode;
  uint8_t entry_state;
  uint8_t block_reason;
  uint8_t safety_latched;
  uint8_t obstacle_state;
  uint8_t obstacle_fault;
  uint8_t leg_online_mask;
  uint8_t wheel_online_mask;
  uint8_t reserved;
  uint32_t fault_bits;
  uint32_t last_command_counter;
  uint32_t uptime_ms;
};
#pragma pack(pop)

static_assert(sizeof(ControlCommandV1) == 24);
static_assert(sizeof(RobotStatusV1) == 24);

inline RobotStatusV1 SafeRobotStatus()
{
  RobotStatusV1 status{};
  status.schema_version = kSchemaVersion;
  status.entry_state = static_cast<uint8_t>(EntryState::BLOCKED);
  status.block_reason = static_cast<uint8_t>(BlockReason::SAFETY_LATCH);
  status.safety_latched = 1;
  status.fault_bits = FAULT_SBUS_LOST | FAULT_SAFETY_LATCHED;
  return status;
}

struct SbusSample
{
  uint16_t channel[16]{};
  int16_t normalized[16]{};
  uint32_t frame_counter = 0;
  uint32_t last_update_ms = 0;
  bool signal_lost = true;
  bool failsafe = true;
};

struct FootTarget
{
  float x_mm = 0.0F;
  float z_mm = 0.0F;
};

inline float Clamp(float value, float low, float high)
{
  return value < low ? low : (value > high ? high : value);
}

}  // namespace RCDog
