#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Board RGB system-state indicator over CubeMX SPI6
constructor_args:
  - topics: null
  - stack_size: 1536
required_hardware:
  - rgb_spi
depends:
  - RobotTopics
=== END MANIFEST === */
// clang-format on

#include <cstdint>

#include "RobotTopics.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "spi.hpp"

class StatusLED final : public LibXR::Application
{
 public:
  StatusLED(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
            RobotTopics& topics, uint32_t stack_size = 1536);
  void OnMonitor() override;

 private:
  static void ThreadEntry(StatusLED* self);
  void Run();
  void Write(uint8_t red, uint8_t green, uint8_t blue,
             LibXR::Semaphore& semaphore);
  static void Encode(uint8_t* output, uint8_t value);

  LibXR::SPI& spi_;
  RCDog::LatestTopicValue<RCDog::RobotStatusV1> status_;
  LibXR::Thread thread_;
};
