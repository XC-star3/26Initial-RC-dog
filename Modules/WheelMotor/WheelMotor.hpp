#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Four C620 M3508 wheel motors with PI, ramps and thermal protection
constructor_args:
  - stack_size: 3072
required_hardware:
  - fdcan_wheel
depends: []
=== END MANIFEST === */
// clang-format on

#include <atomic>
#include <cstdint>

#include "app_framework.hpp"
#include "can.hpp"
#include "libxr.hpp"
#include "robot_types.hpp"

class WheelMotor final : public LibXR::Application
{
 public:
  enum class Mode : uint8_t
  {
    OFF = 0,
    HOLD,
    DRIVE,
  };

  WheelMotor(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
             uint32_t stack_size = 3072);
  void OnMonitor() override;

  void SetMotion(float forward, float yaw, float max_rpm);
  void SetTargets(const float rpm[4], const float scale[4]);
  void SetMode(Mode mode);
  void StopAndLock();
  bool TryClearLock();
  bool IsStopped() const;
  bool IsHealthy() const;
  uint8_t OnlineMask() const;
  uint32_t FaultBits() const;

 private:
  struct Feedback
  {
    int16_t speed_rpm = 0;
    uint8_t temperature_c = 0;
    uint32_t last_update_ms = 0;
  };

  struct Command
  {
    float target_rpm[4]{};
    float scale[4]{1.0F, 1.0F, 1.0F, 1.0F};
    Mode mode = Mode::OFF;
    bool locked = true;
  };

  static void ThreadEntry(WheelMotor* self);
  static void CanRx(bool in_isr, WheelMotor* self,
                    const LibXR::CAN::ClassicPack& pack);
  void Run();
  void HandleCan(const LibXR::CAN::ClassicPack& pack);
  void Tick(uint32_t now_ms);
  void SendCurrents(const int16_t currents[4]);
  uint8_t CalculateOnlineMask(uint32_t now_ms) const;

  LibXR::FDCAN& can_;
  LibXR::CAN::Callback can_callback_;
  LibXR::Thread thread_;
  Feedback feedback_[4]{};
  Command command_{};
  float ramped_rad_s_[4]{};
  float integral_[4]{};
  float peak_budget_ms_[4]{};
  bool overtemp_latched_[4]{};
  std::atomic<uint32_t> online_mask_{0};
  std::atomic<uint32_t> fault_bits_{0};
  uint32_t last_command_ms_ = 0;
  uint32_t last_bus_check_ms_ = 0;
  bool bus_off_ = false;
};
