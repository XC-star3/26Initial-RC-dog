#include "app_framework.hpp"
#include "libxr.hpp"

// Module headers
#include "SbusReceiver.hpp"
#include "DogMotor.hpp"
#include "WheelMotor.hpp"
#include "ObstacleController.hpp"
#include "HostLink.hpp"
#include "RobotControl.hpp"
#include "StatusLED.hpp"

static void XRobotMain(LibXR::HardwareContainer &hw) {
  using namespace LibXR;
  ApplicationManager appmgr;

  // Auto-generated module instantiations
  static SbusReceiver SbusReceiver_0(hw, appmgr, 2048);
  static DogMotor DogMotor_0(hw, appmgr, 4096);
  static WheelMotor WheelMotor_0(hw, appmgr, 3072);
  static ObstacleController ObstacleController_0(hw, appmgr, DogMotor_0, WheelMotor_0);
  static HostLink HostLink_0(hw, appmgr, 3072);
  static RobotControl RobotControl_0(
      hw,
      appmgr,
      SbusReceiver_0,
      DogMotor_0,
      WheelMotor_0,
      ObstacleController_0,
      HostLink_0,
      4096
  );
  static StatusLED StatusLED_0(hw, appmgr, RobotControl_0, 1536);

  while (true) {
    appmgr.MonitorAll();
    Thread::Sleep(100);
  }
}