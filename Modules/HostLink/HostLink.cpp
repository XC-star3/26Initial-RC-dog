#include "HostLink.hpp"

#include <cstring>

#include "FreeRTOS.h"
#include "crc.hpp"
#include "task.h"

namespace
{
constexpr uint32_t kPartialTimeoutMs = 50;
constexpr uint32_t kStatusPeriodMs = 100;
constexpr int16_t kAxisLimit = 1000;
}

HostLink::HostLink(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                   uint32_t stack_size)
    : uart_(*hw.FindOrExit<LibXR::UART>({"usb_cdc"}))
{
  app.Register(*this);
  thread_.Create(this, ThreadEntry, "host_link", stack_size,
                 LibXR::Thread::Priority::MEDIUM);
}

void HostLink::OnMonitor() {}

bool HostLink::ReadCommand(RCDog::ControlCommandV1& command, uint32_t& received_ms,
                           uint32_t& generation) const
{
  taskENTER_CRITICAL();
  const uint32_t current = command_generation_;
  if (current == generation)
  {
    taskEXIT_CRITICAL();
    return false;
  }
  command = command_;
  received_ms = received_ms_;
  taskEXIT_CRITICAL();
  generation = current;
  return true;
}

void HostLink::SetStatus(const RCDog::RobotStatusV1& status)
{
  taskENTER_CRITICAL();
  status_ = status;
  taskEXIT_CRITICAL();
}

uint32_t HostLink::ProtocolErrors() const
{
  return protocol_errors_.load(std::memory_order_relaxed);
}

void HostLink::ThreadEntry(HostLink* self) { self->Run(); }

void HostLink::Run()
{
  uint8_t byte = 0;
  uint32_t last_status_ms = 0;
  LibXR::Semaphore semaphore;
  while (true)
  {
    LibXR::ReadOperation operation(semaphore, 10);
    const auto result = uart_.Read({&byte, 1}, operation);
    const uint32_t now = LibXR::Thread::GetTime();
    if (parse_state_ != ParseState::SYNC && now - partial_started_ms_ > kPartialTimeoutMs)
    {
      protocol_errors_.fetch_add(1, std::memory_order_relaxed);
      ResetParser(result == LibXR::ErrorCode::OK ? byte : 0);
    }
    else if (result == LibXR::ErrorCode::OK)
    {
      Feed(byte, now);
    }
    if (now - last_status_ms >= kStatusPeriodMs)
    {
      SendStatus(semaphore);
      last_status_ms = now;
    }
  }
}

void HostLink::Feed(uint8_t byte, uint32_t now_ms)
{
  if (parse_state_ == ParseState::SYNC)
  {
    if (byte == LibXR::Topic::PACKET_PREFIX)
    {
      packet_[0] = byte;
      packet_size_ = 1;
      partial_started_ms_ = now_ms;
      parse_state_ = ParseState::HEADER;
    }
    return;
  }
  packet_[packet_size_++] = byte;
  if (parse_state_ == ParseState::HEADER &&
      packet_size_ == sizeof(LibXR::Topic::PackedDataHeader))
  {
    const auto* header = reinterpret_cast<const LibXR::Topic::PackedDataHeader*>(packet_);
    const uint32_t expected_topic = LibXR::CRC32::Calculate(
        RCDog::kControlTopic, sizeof(RCDog::kControlTopic) - 1);
    if (!LibXR::CRC8::Verify(packet_, sizeof(LibXR::Topic::PackedDataHeader)) ||
        header->version != LibXR::Topic::PACKET_VERSION ||
        header->GetDataLen() != sizeof(RCDog::ControlCommandV1) ||
        header->topic_name_crc32 != expected_topic)
    {
      protocol_errors_.fetch_add(1, std::memory_order_relaxed);
      ResetParser(byte);
      return;
    }
    parse_state_ = ParseState::PAYLOAD;
  }
  if (parse_state_ == ParseState::PAYLOAD && packet_size_ == sizeof(packet_))
  {
    if (!ValidateAndPublish(now_ms))
    {
      protocol_errors_.fetch_add(1, std::memory_order_relaxed);
    }
    ResetParser();
  }
}

void HostLink::ResetParser(uint8_t possible_prefix)
{
  packet_size_ = 0;
  parse_state_ = ParseState::SYNC;
  if (possible_prefix == LibXR::Topic::PACKET_PREFIX)
  {
    packet_[0] = possible_prefix;
    packet_size_ = 1;
    partial_started_ms_ = LibXR::Thread::GetTime();
    parse_state_ = ParseState::HEADER;
  }
}

bool HostLink::ValidateAndPublish(uint32_t now_ms)
{
  if (!LibXR::CRC8::Verify(packet_, sizeof(packet_)))
  {
    return false;
  }
  RCDog::ControlCommandV1 command{};
  std::memcpy(&command, packet_ + sizeof(LibXR::Topic::PackedDataHeader),
              sizeof(command));
  if (command.schema_version != RCDog::kSchemaVersion || command.mode > 8 ||
      (command.flags & ~RCDog::KNOWN_FLAGS) != 0 || command.reserved != 0 ||
      command.yaw < -kAxisLimit || command.yaw > kAxisLimit ||
      command.forward < -kAxisLimit || command.forward > kAxisLimit ||
      command.speed < 0 || command.speed > kAxisLimit || command.session_id == 0)
  {
    return false;
  }
  taskENTER_CRITICAL();
  command_ = command;
  received_ms_ = now_ms;
  ++command_generation_;
  taskEXIT_CRITICAL();
  return true;
}

void HostLink::SendStatus(LibXR::Semaphore& semaphore)
{
  RCDog::RobotStatusV1 status{};
  taskENTER_CRITICAL();
  status = status_;
  taskEXIT_CRITICAL();
  uint8_t packet[LibXR::Topic::PACK_BASE_SIZE + sizeof(status)]{};
  const uint32_t topic_crc = LibXR::CRC32::Calculate(
      RCDog::kStatusTopic, sizeof(RCDog::kStatusTopic) - 1);
  auto* header = reinterpret_cast<LibXR::Topic::PackedDataHeader*>(packet);
  header->prefix = LibXR::Topic::PACKET_PREFIX;
  header->SetDataLen(sizeof(status));
  header->topic_name_crc32 = topic_crc;
  header->SetTimestamp(LibXR::Timebase::GetMicroseconds());
  header->version = LibXR::Topic::PACKET_VERSION;
  header->pack_header_crc8 = LibXR::CRC8::Calculate(
      header, sizeof(LibXR::Topic::PackedDataHeader) - 1);
  std::memcpy(packet + sizeof(LibXR::Topic::PackedDataHeader), &status,
              sizeof(status));
  packet[sizeof(packet) - 1] = LibXR::CRC8::Calculate(packet, sizeof(packet) - 1);
  LibXR::WriteOperation operation(semaphore, 20);
  (void)uart_.Write(LibXR::ConstRawData(packet), operation);
}
