#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: LOW MID HIGH single-level stair state machine
constructor_args:
  - dog_motor: null
  - wheel_motor: null
required_hardware: []
depends:
  - DogMotor
  - WheelMotor
=== END MANIFEST === */
// clang-format on

#include <atomic>
#include <cstdint>

#include "DogMotor.hpp"
#include "WheelMotor.hpp"
#include "app_framework.hpp"
#include "robot_types.hpp"

class ObstacleController final : public LibXR::Application
{
 public:
  ObstacleController(LibXR::HardwareContainer&, LibXR::ApplicationManager& app,
                     DogMotor& dog_motor, WheelMotor& wheel_motor);
  void OnMonitor() override;

  void SetRequested(bool requested);
  void SelectProfile(RCDog::StairProfile profile);
  bool RequestStep();
  void SafetyAbort();
  void Tick(uint32_t now_ms);
  RCDog::ObstacleState State() const;
  RCDog::ObstacleFault Fault() const;
  RCDog::StairProfile Profile() const;
  uint8_t Phase() const;
  bool CanExit() const;

 private:
  struct Segment
  {
    RCDog::FootTarget target[4];
    uint8_t mask;
    uint32_t duration_ms;
    float clearance_mm;
    uint32_t dwell_ms;
  };

  void BuildSequence();
  void Fail(RCDog::ObstacleFault fault);
  float LandingHeight() const;

  DogMotor& dog_;
  WheelMotor& wheel_;
  Segment sequence_[28]{};
  uint8_t sequence_count_ = 0;
  uint8_t phase_ = 0;
  uint32_t dwell_started_ms_ = 0;
  bool segment_started_ = false;
  bool requested_ = false;
  bool step_pending_ = false;
  RCDog::StairProfile profile_ = RCDog::StairProfile::MID;
  std::atomic<uint32_t> state_{static_cast<uint32_t>(RCDog::ObstacleState::DISABLED)};
  std::atomic<uint32_t> fault_{static_cast<uint32_t>(RCDog::ObstacleFault::NONE)};
};
