#include "StatusLED.hpp"

#include <cstddef>

namespace
{
constexpr std::size_t kResetBytes = 80;
constexpr std::size_t kEncodedChannelBytes = 8;
constexpr std::size_t kPacketSize = 2 * kResetBytes + 3 * kEncodedChannelBytes;
}

StatusLED::StatusLED(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                     RobotTopics& topics, uint32_t stack_size)
    : spi_(*hw.FindOrExit<LibXR::SPI>({"rgb_spi"})),
      status_(topics.Status(), RCDog::SafeRobotStatus())
{
  (void)spi_.SetConfig({LibXR::SPI::ClockPolarity::LOW,
                        LibXR::SPI::ClockPhase::EDGE_2,
                        LibXR::SPI::Prescaler::DIV_4, false});
  app.Register(*this);
  thread_.Create(this, ThreadEntry, "status_led", stack_size,
                 LibXR::Thread::Priority::LOW);
}

void StatusLED::OnMonitor() {}
void StatusLED::ThreadEntry(StatusLED* self) { self->Run(); }

void StatusLED::Run()
{
  LibXR::Semaphore semaphore;
  while (true)
  {
    const auto status = status_.Snapshot();
    if (status.safety_latched != 0 || status.fault_bits != 0)
    {
      Write(96, 0, 0, semaphore);
    }
    else if (status.obstacle_state ==
             static_cast<uint8_t>(RCDog::ObstacleState::MOVING))
    {
      Write(96, 48, 0, semaphore);
    }
    else if (status.entry_state == static_cast<uint8_t>(RCDog::EntryState::ACTIVE))
    {
      Write(0, 64, 16, semaphore);
    }
    else
    {
      Write(0, 16, 64, semaphore);
    }
    LibXR::Thread::Sleep(100);
  }
}

void StatusLED::Write(uint8_t red, uint8_t green, uint8_t blue,
                      LibXR::Semaphore& semaphore)
{
  uint8_t packet[kPacketSize]{};
  Encode(packet + kResetBytes, green);
  Encode(packet + kResetBytes + kEncodedChannelBytes, red);
  Encode(packet + kResetBytes + 2 * kEncodedChannelBytes, blue);
  LibXR::WriteOperation operation(semaphore, 20);
  (void)spi_.Write(LibXR::ConstRawData(packet), operation);
}

void StatusLED::Encode(uint8_t* output, uint8_t value)
{
  for (uint8_t bit = 0; bit < 8; ++bit)
  {
    output[7U - bit] = (value & (1U << bit)) != 0 ? 0x78 : 0x60;
  }
}
