#include "RobotTopics.hpp"

RobotTopics::RobotTopics(LibXR::HardwareContainer&,
                         LibXR::ApplicationManager& app)
    : domain_("rcdog"),
      sbus_(LibXR::Topic::CreateTopic<RCDog::SbusSample>(RCDog::kSbusTopic,
                                                         &domain_)),
      control_command_(LibXR::Topic::CreateTopic<RCDog::ControlCommandV1>(
          RCDog::kControlTopic, &domain_)),
      status_(LibXR::Topic::CreateTopic<RCDog::RobotStatusV1>(
          RCDog::kStatusTopic, &domain_))
{
  app.Register(*this);
}

void RobotTopics::OnMonitor() {}
