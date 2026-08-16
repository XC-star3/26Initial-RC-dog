#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Sole mode input arbitration and production safety owner
constructor_args:
  - topics: null
  - dog_motor: null
  - wheel_motor: null
  - obstacle: null
  - stack_size: 4096
required_hardware: []
depends:
  - RobotTopics
  - DogMotor
  - WheelMotor
  - ObstacleController
=== END MANIFEST === */
// clang-format on

#include <cstdint>

#include "DogMotor.hpp"
#include "ObstacleController.hpp"
#include "RobotTopics.hpp"
#include "WheelMotor.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "robot_types.hpp"

class RobotControl final : public LibXR::Application
{
 public:
  RobotControl(LibXR::HardwareContainer&, LibXR::ApplicationManager& app,
               RobotTopics& topics, DogMotor& dog_motor, WheelMotor& wheel_motor,
               ObstacleController& obstacle,
               uint32_t stack_size = 4096);
  void OnMonitor() override;

 private:
  struct Input
  {
    RCDog::ControlSource source = RCDog::ControlSource::NONE;
    RCDog::RobotMode mode = RCDog::RobotMode::MOTOR_CHECK;
    uint8_t requested_mode = 0;
    float yaw = 0.0F;
    float forward = 0.0F;
    float speed = 0.0F;
    uint16_t flags = 0;
    bool valid = false;
  };

  static void ThreadEntry(RobotControl* self);
  void Run();
  void Tick(uint32_t now_ms);
  Input SelectInput(const RCDog::SbusSample& sbus, uint32_t now_ms);
  void UpdateSafety(const RCDog::SbusSample& sbus, uint32_t now_ms);
  void Execute(const Input& input, const RCDog::SbusSample& sbus,
               uint32_t now_ms);
  void StopAll(bool safety);
  void PublishStatus(const Input& input, const RCDog::SbusSample& sbus,
                     uint32_t now_ms);
  void RevokeUsb();
  static bool ValidCommand(const RCDog::ControlCommandV1& command);
  static bool CounterForward(uint32_t value, uint32_t previous);
  float LowWheelForward(float requested, uint32_t now_ms);

  DogMotor& dog_;
  WheelMotor& wheel_;
  ObstacleController& obstacle_;
  RCDog::LatestTopicValue<RCDog::SbusSample> sbus_;
  RCDog::LatestTopicValue<RCDog::ControlCommandV1> control_command_;
  LibXR::Topic status_topic_;
  LibXR::Thread thread_;
  RCDog::RobotStatusV1 status_{};
  RCDog::ControlCommandV1 usb_command_{};
  uint32_t usb_rx_ms_ = 0;
  uint32_t usb_generation_ = 0;
  uint32_t usb_session_ = 0;
  uint32_t blocked_session_ = 0;
  uint32_t last_usb_counter_ = 0;
  uint32_t last_accepted_usb_counter_ = 0;
  uint32_t semantic_protocol_errors_ = 0;
  uint8_t safe_zero_count_ = 0;
  uint32_t release_since_ms_ = 0;
  uint32_t obstacle_stop_since_ms_ = 0;
  uint32_t last_step_frame_ = 0;
  uint8_t step_stable_frames_ = 0;
  bool step_input_initialized_ = false;
  bool step_input_high_ = false;
  bool step_candidate_high_ = false;
  bool usb_active_ = false;
  bool usb_timeout_latched_ = false;
  bool safety_latched_ = true;
  bool obstacle_started_ = false;
  bool obstacle_profile_locked_ = false;
  bool low_wheel_reverse_rearm_ = false;
  int8_t low_wheel_direction_ = 0;
  uint32_t low_wheel_neutral_since_ms_ = 0;
};
