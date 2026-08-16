#include "HostLink.hpp"

#include <cstddef>
#include <cstdio>

#include "crc.hpp"

namespace
{
constexpr uint32_t kStatusPeriodMs = 100;
constexpr std::size_t kServerBufferSize = 64;
constexpr std::size_t kDiagnosticBufferSize = 384;

static_assert(sizeof(LibXR::Topic::PackedData<RCDog::RobotStatusV1>) ==
              LibXR::Topic::PACK_BASE_SIZE + sizeof(RCDog::RobotStatusV1));
}

HostLink::HostLink(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                   RobotTopics& topics, DogMotor& dog_motor,
                   WheelMotor& wheel_motor, uint32_t stack_size)
    : uart_(*hw.FindOrExit<LibXR::UART>({"usb_cdc"})),
      dog_(dog_motor),
      wheel_(wheel_motor),
      control_topic_(topics.ControlCommand()),
      status_topic_(topics.Status()),
      server_(kServerBufferSize),
      status_(status_topic_, RCDog::SafeRobotStatus()),
      sbus_(topics.Sbus())
{
  server_.Register(control_topic_);
  app.Register(*this);
  thread_.Create(this, ThreadEntry, "host_link", stack_size,
                 LibXR::Thread::Priority::MEDIUM);
}

void HostLink::OnMonitor() {}

void HostLink::ThreadEntry(HostLink* self) { self->Run(); }

void HostLink::Run()
{
  uint8_t byte = 0;
  uint32_t last_status_ms = 0;
  LibXR::Semaphore semaphore;
  while (true)
  {
    LibXR::ReadOperation operation(semaphore, 10);
    if (uart_.Read({&byte, 1}, operation) == LibXR::ErrorCode::OK)
    {
      (void)server_.ParseData(LibXR::ConstRawData(&byte, 1));
      ObserveDiagnosticByte(byte, semaphore);
    }

    const uint32_t now = LibXR::Thread::GetTime();
    if (now - last_status_ms >= kStatusPeriodMs)
    {
      SendStatus(semaphore);
      last_status_ms = now;
    }
  }
}

void HostLink::SendStatus(LibXR::Semaphore& semaphore)
{
  const RCDog::RobotStatusV1 status = status_.Snapshot();
  LibXR::Topic::PackedData<RCDog::RobotStatusV1> packet{};
  if (status_topic_.PackData(status, packet) != LibXR::ErrorCode::OK)
  {
    return;
  }
  LibXR::WriteOperation operation(semaphore, 20);
  (void)uart_.Write(LibXR::ConstRawData(packet), operation);
}

void HostLink::ObserveDiagnosticByte(uint8_t byte,
                                     LibXR::Semaphore& semaphore)
{
  if (observed_payload_bytes_ != 0)
  {
    --observed_payload_bytes_;
    return;
  }

  if (observed_header_size_ != 0)
  {
    observed_header_[observed_header_size_++] = byte;
    if (observed_header_size_ < sizeof(observed_header_))
    {
      return;
    }

    const bool valid_header =
        LibXR::CRC8::Verify(observed_header_, sizeof(observed_header_)) &&
        observed_header_[14] == LibXR::Topic::PACKET_VERSION;
    const uint32_t payload_size = static_cast<uint32_t>(observed_header_[1]) |
        static_cast<uint32_t>(observed_header_[2]) << 8U |
        static_cast<uint32_t>(observed_header_[3]) << 16U;
    observed_header_size_ = 0;
    if (valid_header &&
        payload_size + LibXR::Topic::PACK_BASE_SIZE <= kServerBufferSize)
    {
      observed_payload_bytes_ = payload_size + 1U;
    }
    return;
  }

  if (byte == LibXR::Topic::PACKET_PREFIX)
  {
    observed_header_[0] = byte;
    observed_header_size_ = 1;
    return;
  }

  if (byte == 'p')
  {
    SendSystemDiagnostic(semaphore);
  }
  else if (byte == 'Y' || byte == 'y')
  {
    SendSbusDiagnostic(semaphore);
  }
}

void HostLink::SendSystemDiagnostic(LibXR::Semaphore& semaphore)
{
  const RCDog::RobotStatusV1 status = status_.Snapshot();
  char text[kDiagnosticBufferSize]{};
  const int length = std::snprintf(
      text, sizeof(text),
      "\r\np STATUS schema=%u source=%u requested=%u active=%u entry=%u "
      "block=%u safety=%u faults=0x%08lX counter=%lu uptime=%lums\r\n"
      "LEG state=%u online=0x%02X healthy=%u standing=%u faults=0x%08lX\r\n"
      "WHEEL online=0x%02X healthy=%u stopped=%u faults=0x%08lX\r\n"
      "STAIR state=%u fault=%u\r\n",
      static_cast<unsigned>(status.schema_version),
      static_cast<unsigned>(status.control_source),
      static_cast<unsigned>(status.requested_mode),
      static_cast<unsigned>(status.active_mode),
      static_cast<unsigned>(status.entry_state),
      static_cast<unsigned>(status.block_reason),
      static_cast<unsigned>(status.safety_latched),
      static_cast<unsigned long>(status.fault_bits),
      static_cast<unsigned long>(status.last_command_counter),
      static_cast<unsigned long>(status.uptime_ms),
      static_cast<unsigned>(dog_.GetState()),
      static_cast<unsigned>(dog_.OnlineMask()), dog_.IsHealthy() ? 1U : 0U,
      dog_.IsStanding() ? 1U : 0U,
      static_cast<unsigned long>(dog_.FaultBits()),
      static_cast<unsigned>(wheel_.OnlineMask()),
      wheel_.IsHealthy() ? 1U : 0U, wheel_.IsStopped() ? 1U : 0U,
      static_cast<unsigned long>(wheel_.FaultBits()),
      static_cast<unsigned>(status.obstacle_state),
      static_cast<unsigned>(status.obstacle_fault));
  if (length > 0)
  {
    WriteText(text, static_cast<std::size_t>(length), semaphore);
  }
}

void HostLink::SendSbusDiagnostic(LibXR::Semaphore& semaphore)
{
  const RCDog::SbusSample sbus = sbus_.Snapshot();
  const RCDog::RobotStatusV1 status = status_.Snapshot();
  const uint32_t now = LibXR::Thread::GetTime();
  const uint32_t age = sbus.last_update_ms == 0 ? UINT32_MAX
                                                : now - sbus.last_update_ms;
  char text[kDiagnosticBufferSize]{};
  const int length = std::snprintf(
      text, sizeof(text),
      "\r\nY SBUS frame=%lu age=%lums lost=%u failsafe=%u source=%u\r\n"
      "CH1 yaw=%u/%d CH2 forward=%u/%d CH3 speed=%u/%d\r\n"
      "CH5 main=%u/%d CH8 sub=%u/%d CH9 safety=%u/%d CH10 step=%u/%d\r\n"
      "MODE requested=%u active=%u entry=%u block=%u safety_latched=%u\r\n",
      static_cast<unsigned long>(sbus.frame_counter),
      static_cast<unsigned long>(age), sbus.signal_lost ? 1U : 0U,
      sbus.failsafe ? 1U : 0U,
      static_cast<unsigned>(status.control_source),
      static_cast<unsigned>(sbus.channel[0]), static_cast<int>(sbus.normalized[0]),
      static_cast<unsigned>(sbus.channel[1]), static_cast<int>(sbus.normalized[1]),
      static_cast<unsigned>(sbus.channel[2]), static_cast<int>(sbus.normalized[2]),
      static_cast<unsigned>(sbus.channel[4]), static_cast<int>(sbus.normalized[4]),
      static_cast<unsigned>(sbus.channel[7]), static_cast<int>(sbus.normalized[7]),
      static_cast<unsigned>(sbus.channel[8]), static_cast<int>(sbus.normalized[8]),
      static_cast<unsigned>(sbus.channel[9]), static_cast<int>(sbus.normalized[9]),
      static_cast<unsigned>(status.requested_mode),
      static_cast<unsigned>(status.active_mode),
      static_cast<unsigned>(status.entry_state),
      static_cast<unsigned>(status.block_reason),
      static_cast<unsigned>(status.safety_latched));
  if (length > 0)
  {
    WriteText(text, static_cast<std::size_t>(length), semaphore);
  }
}

void HostLink::WriteText(const char* text, std::size_t size,
                         LibXR::Semaphore& semaphore)
{
  if (size >= kDiagnosticBufferSize)
  {
    size = kDiagnosticBufferSize - 1U;
  }
  LibXR::WriteOperation operation(semaphore, 50);
  (void)uart_.Write(LibXR::ConstRawData(text, size), operation);
}
