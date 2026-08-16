#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Event-driven UART5 SBUS receiver and strongly typed topic publisher
constructor_args:
  - topics: null
  - stack_size: 2048
required_hardware:
  - sbus_uart
depends:
  - RobotTopics
=== END MANIFEST === */
// clang-format on

#include <cstdint>

#include "RobotTopics.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "robot_types.hpp"
#include "uart.hpp"

class SbusReceiver final : public LibXR::Application
{
 public:
  SbusReceiver(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
               RobotTopics& topics, uint32_t stack_size = 2048);
  void OnMonitor() override;

 private:
  static void ThreadEntry(SbusReceiver* self);
  void Run();
  void Feed(uint8_t byte);
  void ParseFrame();
  static int16_t Normalize(uint16_t value);

  LibXR::UART& uart_;
  LibXR::Topic sbus_topic_;
  LibXR::Thread thread_;
  uint8_t stream_[25]{};
  uint8_t stream_size_ = 0;
  uint32_t generation_ = 0;
};
