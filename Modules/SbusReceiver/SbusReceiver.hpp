#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Event-driven UART5 SBUS receiver and strong typed snapshot publisher
constructor_args:
  - stack_size: 2048
required_hardware:
  - sbus_uart
depends: []
=== END MANIFEST === */
// clang-format on

#include <atomic>
#include <cstdint>

#include "app_framework.hpp"
#include "libxr.hpp"
#include "robot_types.hpp"
#include "uart.hpp"

class SbusReceiver final : public LibXR::Application
{
 public:
  SbusReceiver(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
               uint32_t stack_size = 2048);
  void OnMonitor() override;
  RCDog::SbusSample Snapshot() const;
  bool IsFresh(uint32_t now_ms, uint32_t timeout_ms = 250) const;
  static uint8_t Switch3(const RCDog::SbusSample& sample, uint8_t channel);

 private:
  static void ThreadEntry(SbusReceiver* self);
  void Run();
  void Feed(uint8_t byte);
  void ParseFrame();
  static int16_t Normalize(uint16_t value);

  LibXR::UART& uart_;
  LibXR::Thread thread_;
  uint8_t stream_[25]{};
  uint8_t stream_size_ = 0;
  RCDog::SbusSample sample_{};
  std::atomic<uint32_t> generation_{0};
  std::atomic<uint32_t> parse_errors_{0};
};
