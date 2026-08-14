#include "StatusLED.hpp"

#include <cstring>

StatusLED::StatusLED(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
                     RobotControl& robot_control, uint32_t stack_size)
    : spi_(*hw.FindOrExit<LibXR::SPI>({"rgb_spi"})), control_(robot_control)
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
  while (true)
  {
    const auto status = control_.Status();
    if (status.safety_latched != 0 || status.fault_bits != 0)
    {
      Write(96, 0, 0);
    }
    else if (status.obstacle_state ==
             static_cast<uint8_t>(RCDog::ObstacleState::MOVING))
    {
      Write(96, 48, 0);
    }
    else if (status.entry_state == static_cast<uint8_t>(RCDog::EntryState::ACTIVE))
    {
      Write(0, 64, 16);
    }
    else
    {
      Write(0, 16, 64);
    }
    LibXR::Thread::Sleep(100);
  }
}

void StatusLED::Write(uint8_t red, uint8_t green, uint8_t blue)
{
  uint8_t packet[184]{};
  Encode(packet + 80, green);
  Encode(packet + 88, red);
  Encode(packet + 96, blue);
  LibXR::Semaphore semaphore;
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
