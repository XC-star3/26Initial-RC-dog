#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Strict XRUSB Topic packet parser and status transmitter
constructor_args:
  - stack_size: 3072
required_hardware:
  - usb_cdc
depends: []
=== END MANIFEST === */
// clang-format on

#include <atomic>
#include <cstdint>

#include "app_framework.hpp"
#include "libxr.hpp"
#include "robot_types.hpp"
#include "uart.hpp"

class HostLink final : public LibXR::Application
{
 public:
  HostLink(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
           uint32_t stack_size = 3072);
  void OnMonitor() override;

  bool ReadCommand(RCDog::ControlCommandV1& command, uint32_t& received_ms,
                   uint32_t& generation) const;
  void SetStatus(const RCDog::RobotStatusV1& status);
  uint32_t ProtocolErrors() const;

 private:
  enum class ParseState : uint8_t
  {
    SYNC = 0,
    HEADER,
    PAYLOAD,
  };

  static void ThreadEntry(HostLink* self);
  void Run();
  void Feed(uint8_t byte, uint32_t now_ms);
  void ResetParser(uint8_t possible_prefix = 0);
  bool ValidateAndPublish(uint32_t now_ms);
  void SendStatus(uint32_t now_ms);

  LibXR::UART& uart_;
  LibXR::Thread thread_;
  uint8_t packet_[sizeof(LibXR::Topic::PackedDataHeader) +
                  sizeof(RCDog::ControlCommandV1) + 1]{};
  uint8_t packet_size_ = 0;
  ParseState parse_state_ = ParseState::SYNC;
  uint32_t partial_started_ms_ = 0;
  RCDog::ControlCommandV1 command_{};
  RCDog::RobotStatusV1 status_{};
  uint32_t received_ms_ = 0;
  std::atomic<uint32_t> command_generation_{0};
  std::atomic<uint32_t> status_generation_{0};
  std::atomic<uint32_t> protocol_errors_{0};
};
