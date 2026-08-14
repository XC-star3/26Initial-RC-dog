#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Eight MW leg motors, kinematics, stand and diagonal trot controller
constructor_args:
  - stack_size: 4096
required_hardware:
  - fdcan_front
  - fdcan_rear
depends: []
=== END MANIFEST === */
// clang-format on

#include <atomic>
#include <cstdint>

#include "app_framework.hpp"
#include "can.hpp"
#include "libxr.hpp"
#include "robot_types.hpp"

class DogMotor final : public LibXR::Application
{
 public:
  enum class State : uint8_t
  {
    SAFE = 0,
    CONFIGURING,
    STANDING,
    LOWERING,
    MECHANICAL,
    GAIT,
    FOOT_MOTION,
    FAULT,
  };

  DogMotor(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
           uint32_t stack_size = 4096);

  void OnMonitor() override;
  void SafeStop();
  void RequestStand();
  void RequestLower();
  void RequestMechanicalPose(bool reverse);
  bool IsMechanicalPoseReady(bool reverse) const;
  void SetGait(float forward, float yaw, float speed, bool enabled,
               bool smooth_stop = false);
  bool StartFootMotion(const RCDog::FootTarget target[4], uint8_t leg_mask,
                       uint32_t duration_ms, float clearance_mm = 0.0F);
  bool MotionComplete() const;
  bool IsStanding() const;
  bool IsSafe() const;
  bool IsHealthy() const;
  uint8_t OnlineMask() const;
  uint32_t FaultBits() const;
  State GetState() const;

 public:
  struct MotorConfig
  {
    uint8_t leg;
    uint8_t joint;
    uint8_t bus;
    uint8_t node_id;
    float direction;
    float torque_direction;
    float ratio;
    float min_deg;
    float max_deg;
  };

 private:

  struct MotorFeedback
  {
    float encoder_turn;
    float velocity_turn_s;
    float zero_turn;
    uint32_t last_heartbeat_ms;
    uint32_t last_encoder_ms;
    uint32_t axis_error;
    uint8_t axis_state;
    uint8_t fault_flags;
    bool zero_valid;
  };

  struct Command
  {
    State requested_state = State::SAFE;
    float forward = 0.0F;
    float yaw = 0.0F;
    float speed = 0.5F;
    bool gait_enabled = false;
    bool smooth_stop = false;
    bool mechanical_reverse = false;
    RCDog::FootTarget foot_target[4]{};
    uint8_t foot_mask = 0;
    uint32_t foot_duration_ms = 0;
    float foot_clearance_mm = 0.0F;
    uint32_t generation = 0;
  };

  struct Motion
  {
    RCDog::FootTarget start[4]{};
    RCDog::FootTarget target[4]{};
    uint8_t mask = 0;
    uint32_t start_ms = 0;
    uint32_t duration_ms = 0;
    float clearance_mm = 0.0F;
    bool active = false;
    bool complete = false;
  };

  static void ThreadEntry(DogMotor* self);
  static void CanRxFront(bool in_isr, DogMotor* self,
                         const LibXR::CAN::ClassicPack& pack);
  static void CanRxRear(bool in_isr, DogMotor* self,
                        const LibXR::CAN::ClassicPack& pack);
  void Run();
  void HandleCan(const LibXR::CAN::ClassicPack& pack, uint8_t bus);
  void ControlTick(uint32_t now_ms);
  void ApplyCommand(const Command& command, uint32_t now_ms);
  void UpdateTargets(uint32_t now_ms);
  void SendMotor(uint8_t index, float target_deg, float dt_s);
  void SendIdle(uint8_t index);
  void SendQuery(uint8_t index);
  void SendSetup(uint8_t index);
  bool InverseKinematics(float x_mm, float z_mm, float& hip_deg,
                         float& knee_deg) const;
  bool ValidateFootMotion(const RCDog::FootTarget target[4], uint8_t leg_mask,
                          float clearance_mm) const;
  bool ValidateFootMotionFrom(const RCDog::FootTarget start[4],
                              const RCDog::FootTarget target[4],
                              uint8_t leg_mask, float clearance_mm) const;
  void UpdateFootTargets(uint32_t now_ms);
  uint8_t CalculateOnlineMask(uint32_t now_ms) const;
  static float Smooth(float x);

  LibXR::FDCAN& front_;
  LibXR::FDCAN& rear_;
  LibXR::CAN::Callback front_callback_;
  LibXR::CAN::Callback rear_callback_;
  LibXR::Thread thread_;
  MotorFeedback feedback_[8]{};
  float target_deg_[8]{};
  float integral_[8]{};
  float previous_error_[8]{};
  RCDog::FootTarget current_foot_[4]{};
  RCDog::FootTarget published_foot_[4]{};
  Command pending_command_{};
  Command active_command_{};
  Motion motion_{};
  std::atomic<uint32_t> command_generation_{0};
  std::atomic<uint32_t> published_state_{0};
  std::atomic<uint32_t> online_mask_{0};
  std::atomic<uint32_t> closed_loop_mask_{0};
  std::atomic<uint32_t> fault_bits_{0};
  std::atomic<bool> mechanical_ready_{false};
  std::atomic<bool> mechanical_reverse_{false};
  std::atomic<bool> motion_complete_{false};
  std::atomic<bool> reconfigure_required_{true};
  uint32_t applied_generation_ = 0;
  uint32_t setup_cursor_ = 0;
  uint8_t query_cursor_ = 0;
  uint32_t last_setup_ms_ = 0;
  uint32_t last_bus_check_ms_ = 0;
  uint32_t last_idle_ms_ = 0;
  uint32_t configuration_started_ms_ = 0;
  State last_control_state_ = State::CONFIGURING;
  bool bus_off_ = false;
  bool motion_fault_ = false;
  bool closed_loop_fault_ = false;
  bool configuration_wait_idle_ = false;
  uint32_t gait_epoch_ms_ = 0;
};
