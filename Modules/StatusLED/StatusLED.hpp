#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Board RGB system-state indicator over CubeMX SPI6
constructor_args:
  - robot_control: null
  - stack_size: 1536
required_hardware:
  - rgb_spi
depends:
  - RobotControl
=== END MANIFEST === */
// clang-format on

#include <cstdint>

#include "RobotControl.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "spi.hpp"

class StatusLED final : public LibXR::Application
{
 public:
  StatusLED(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
            RobotControl& robot_control, uint32_t stack_size = 1536);
  void OnMonitor() override;

 private:
  static void ThreadEntry(StatusLED* self);
  void Run();
  void Write(uint8_t red, uint8_t green, uint8_t blue,
             LibXR::Semaphore& semaphore);
  static void Encode(uint8_t* output, uint8_t value);

  LibXR::SPI& spi_;
  RobotControl& control_;
  LibXR::Thread thread_;
};
