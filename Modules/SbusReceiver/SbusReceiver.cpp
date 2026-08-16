#include "SbusReceiver.hpp"

#include <cstring>

namespace
{
constexpr uint8_t kHeader = 0x0F;
constexpr uint8_t kEnd = 0x00;
constexpr uint8_t kSignalLost = 0x04;
constexpr uint8_t kFailsafe = 0x08;
}

SbusReceiver::SbusReceiver(LibXR::HardwareContainer& hw,
                           LibXR::ApplicationManager& app, RobotTopics& topics,
                           uint32_t stack_size)
    : uart_(*hw.FindOrExit<LibXR::UART>({"sbus_uart"})),
      sbus_topic_(topics.Sbus())
{
  (void)uart_.SetConfig({100000, LibXR::UART::Parity::EVEN, 8, 2});
  app.Register(*this);
  thread_.Create(this, ThreadEntry, "sbus_rx", stack_size,
                 LibXR::Thread::Priority::HIGH);
}

void SbusReceiver::OnMonitor() {}

void SbusReceiver::ThreadEntry(SbusReceiver* self) { self->Run(); }

void SbusReceiver::Run()
{
  uint8_t byte = 0;
  LibXR::Semaphore semaphore;
  while (true)
  {
    LibXR::ReadOperation operation(semaphore, 50);
    if (uart_.Read({&byte, 1}, operation) == LibXR::ErrorCode::OK)
    {
      Feed(byte);
    }
  }
}

void SbusReceiver::Feed(uint8_t byte)
{
  if (stream_size_ == 0)
  {
    if (byte != kHeader)
    {
      return;
    }
    stream_[stream_size_++] = byte;
    return;
  }
  stream_[stream_size_++] = byte;
  if (stream_size_ < sizeof(stream_))
  {
    return;
  }
  if (stream_[0] == kHeader && stream_[24] == kEnd)
  {
    ParseFrame();
    stream_size_ = 0;
    return;
  }
  uint8_t keep = 0;
  for (uint8_t i = 1; i < sizeof(stream_); ++i)
  {
    if (stream_[i] == kHeader)
    {
      keep = sizeof(stream_) - i;
      std::memmove(stream_, stream_ + i, keep);
      break;
    }
  }
  stream_size_ = keep;
}

void SbusReceiver::ParseFrame()
{
  RCDog::SbusSample sample{};
  sample.channel[0] = ((stream_[2] << 8) | stream_[1]) & 0x07FF;
  sample.channel[1] = ((stream_[3] << 5) | (stream_[2] >> 3)) & 0x07FF;
  sample.channel[2] = ((stream_[5] << 10) | (stream_[4] << 2) | (stream_[3] >> 6)) & 0x07FF;
  sample.channel[3] = ((stream_[6] << 7) | (stream_[5] >> 1)) & 0x07FF;
  sample.channel[4] = ((stream_[7] << 4) | (stream_[6] >> 4)) & 0x07FF;
  sample.channel[5] = ((stream_[9] << 9) | (stream_[8] << 1) | (stream_[7] >> 7)) & 0x07FF;
  sample.channel[6] = ((stream_[10] << 6) | (stream_[9] >> 2)) & 0x07FF;
  sample.channel[7] = ((stream_[11] << 3) | (stream_[10] >> 5)) & 0x07FF;
  sample.channel[8] = ((stream_[13] << 8) | stream_[12]) & 0x07FF;
  sample.channel[9] = ((stream_[14] << 5) | (stream_[13] >> 3)) & 0x07FF;
  sample.channel[10] = ((stream_[16] << 10) | (stream_[15] << 2) | (stream_[14] >> 6)) & 0x07FF;
  sample.channel[11] = ((stream_[17] << 7) | (stream_[16] >> 1)) & 0x07FF;
  sample.channel[12] = ((stream_[18] << 4) | (stream_[17] >> 4)) & 0x07FF;
  sample.channel[13] = ((stream_[20] << 9) | (stream_[19] << 1) | (stream_[18] >> 7)) & 0x07FF;
  sample.channel[14] = ((stream_[21] << 6) | (stream_[20] >> 2)) & 0x07FF;
  sample.channel[15] = ((stream_[22] << 3) | (stream_[21] >> 5)) & 0x07FF;
  for (uint8_t i = 0; i < 16; ++i)
  {
    sample.normalized[i] = Normalize(sample.channel[i]);
  }
  sample.signal_lost = (stream_[23] & kSignalLost) != 0;
  sample.failsafe = (stream_[23] & kFailsafe) != 0;
  sample.last_update_ms = LibXR::Thread::GetTime();
  sample.frame_counter = ++generation_;
  sbus_topic_.Publish(sample);
}

int16_t SbusReceiver::Normalize(uint16_t value)
{
  int32_t result = (static_cast<int32_t>(value) - 1024) * 100 / 671;
  result = result < -100 ? -100 : (result > 100 ? 100 : result);
  return result > -5 && result < 5 ? 0 : static_cast<int16_t>(result);
}
