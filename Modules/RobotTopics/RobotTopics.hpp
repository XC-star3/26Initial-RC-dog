#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: Strongly typed RC-dog LibXR topic registry
constructor_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include <cstdint>

#include "FreeRTOS.h"
#include "app_framework.hpp"
#include "libxr.hpp"
#include "robot_types.hpp"
#include "task.h"

namespace RCDog
{

template <LibXR::TopicPayload Data>
class LatestTopicValue
{
 public:
  explicit LatestTopicValue(LibXR::Topic topic, const Data& initial = Data{})
      : value_(initial),
        callback_(LibXR::Topic::Callback::Create(
            [](bool in_isr, LatestTopicValue* self,
               LibXR::MicrosecondTimestamp, Data& value)
            {
              ASSERT(!in_isr);
              self->Store(value);
            },
            this))
  {
    topic.RegisterCallback(callback_);
  }

  LatestTopicValue(const LatestTopicValue&) = delete;
  LatestTopicValue& operator=(const LatestTopicValue&) = delete;
  LatestTopicValue(LatestTopicValue&&) = delete;
  LatestTopicValue& operator=(LatestTopicValue&&) = delete;

  bool ReadNext(Data& value, uint32_t& received_ms, uint32_t& generation) const
  {
    taskENTER_CRITICAL();
    const uint32_t current_generation = generation_;
    if (current_generation == generation)
    {
      taskEXIT_CRITICAL();
      return false;
    }
    value = value_;
    received_ms = received_ms_;
    taskEXIT_CRITICAL();
    generation = current_generation;
    return true;
  }

  Data Snapshot() const
  {
    Data value{};
    taskENTER_CRITICAL();
    value = value_;
    taskEXIT_CRITICAL();
    return value;
  }

 private:
  void Store(const Data& value)
  {
    taskENTER_CRITICAL();
    value_ = value;
    received_ms_ = LibXR::Thread::GetTime();
    ++generation_;
    taskEXIT_CRITICAL();
  }

  Data value_{};
  uint32_t received_ms_ = 0;
  uint32_t generation_ = 0;
  LibXR::Topic::Callback callback_;
};

}  // namespace RCDog

class RobotTopics final : public LibXR::Application
{
 public:
  RobotTopics(LibXR::HardwareContainer&, LibXR::ApplicationManager& app);
  void OnMonitor() override;

  LibXR::Topic Sbus() const { return sbus_; }
  LibXR::Topic ControlCommand() const { return control_command_; }
  LibXR::Topic Status() const { return status_; }

 private:
  LibXR::Topic::Domain domain_;
  LibXR::Topic sbus_;
  LibXR::Topic control_command_;
  LibXR::Topic status_;
};
