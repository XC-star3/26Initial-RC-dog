#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: XRUSB Topic Server bridge with read-only diagnostics
constructor_args:
  - topics: null
  - dog_motor: null
  - wheel_motor: null
  - stack_size: 3072
required_hardware:
  - usb_cdc
depends:
  - RobotTopics
  - DogMotor
  - WheelMotor
=== END MANIFEST === */
// clang-format on

#include <cstddef>
#include <cstdint>

#include "DogMotor.hpp"
#include "RobotTopics.hpp"
#include "WheelMotor.hpp"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "robot_types.hpp"
#include "uart.hpp"

class HostLink final : public LibXR::Application
{
 public:
  HostLink(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
           RobotTopics& topics, DogMotor& dog_motor, WheelMotor& wheel_motor,
           uint32_t stack_size = 3072);
  void OnMonitor() override;

 private:
  static void ThreadEntry(HostLink* self);
  void Run();
  void SendStatus(LibXR::Semaphore& semaphore);
  void ObserveDiagnosticByte(uint8_t byte, LibXR::Semaphore& semaphore);
  void SendSystemDiagnostic(LibXR::Semaphore& semaphore);
  void SendSbusDiagnostic(LibXR::Semaphore& semaphore);
  void WriteText(const char* text, std::size_t size,
                 LibXR::Semaphore& semaphore);

  LibXR::UART& uart_;
  DogMotor& dog_;
  WheelMotor& wheel_;
  LibXR::Topic control_topic_;
  LibXR::Topic status_topic_;
  LibXR::Topic::Server server_;
  RCDog::LatestTopicValue<RCDog::RobotStatusV1> status_;
  RCDog::LatestTopicValue<RCDog::SbusSample> sbus_;
  LibXR::Thread thread_;
  uint8_t observed_header_[16]{};
  uint8_t observed_header_size_ = 0;
  uint32_t observed_payload_bytes_ = 0;
};
