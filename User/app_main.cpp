#include "app_main.h"

#include "cdc_uart.hpp"
#include "fdcan.h"
#include "libxr.hpp"
#include "main.h"
#include "spi.h"
#include "stm32_canfd.hpp"
#include "stm32_spi.hpp"
#include "stm32_timebase.hpp"
#include "stm32_uart.hpp"
#include "stm32_usb_dev.hpp"
#include "uart.hpp"
#include "usb_otg.h"
#include "usart.h"
#include "xrobot_main.hpp"

namespace
{
using LibXR::USB::Endpoint;

constexpr auto kEnglish = LibXR::USB::DescriptorStrings::MakeLanguagePack(
    LibXR::USB::DescriptorStrings::Language::EN_US, "XRobot",
    "RC-dog XRUSB CDC", "UID-");

__attribute__((section(".dma_buffer"), aligned(32))) uint8_t sbus_rx[128];
alignas(32) uint8_t spi6_rx[184];
alignas(32) uint8_t spi6_tx[184];
alignas(32) uint8_t usb_ep0_out[64];
alignas(32) uint8_t usb_ep1_out[512];
alignas(32) uint8_t usb_ep0_in[64];
alignas(32) uint8_t usb_ep1_in[512];
alignas(32) uint8_t usb_ep2_in[64];
}  // namespace

extern "C" void app_main(void)
{
  using namespace LibXR;

  STM32Timebase timebase;
  PlatformInit(2, 1024);

  STM32CANFD fdcan_front(&hfdcan1, 16);
  STM32CANFD fdcan_rear(&hfdcan2, 16);
  STM32CANFD fdcan_wheel(&hfdcan3, 16);
  STM32UART sbus_uart(&huart5, RawData(sbus_rx), {nullptr, 0}, 16);
  STM32SPI rgb_spi(&hspi6, RawData(spi6_rx), RawData(spi6_tx), 256);

  USB::CDCUart usb_cdc(Endpoint::EPNumber::EP1, Endpoint::EPNumber::EP1,
                       Endpoint::EPNumber::EP2, 512, 512, 8);
  STM32USBDeviceOtgHS usb_device(
      &hpcd_USB_OTG_HS, 2048,
      {RawData(usb_ep0_out), RawData(usb_ep1_out)},
      {{RawData(usb_ep0_in), 512}, {RawData(usb_ep1_in), 1024},
       {RawData(usb_ep2_in), 512}},
      USB::DeviceDescriptor::PacketSize0::SIZE_64, 0x0483, 0x5740, 0x0100,
      {&kEnglish}, {{&usb_cdc}},
      ConstRawData(reinterpret_cast<const void*>(UID_BASE), 12));

  HardwareContainer hardware(
      Entry<FDCAN>{fdcan_front, {"fdcan_front"}},
      Entry<FDCAN>{fdcan_rear, {"fdcan_rear"}},
      Entry<FDCAN>{fdcan_wheel, {"fdcan_wheel"}},
      Entry<UART>{sbus_uart, {"sbus_uart"}},
      Entry<UART>{usb_cdc, {"usb_cdc"}},
      Entry<SPI>{rgb_spi, {"rgb_spi"}});

  usb_device.Init(false);
  usb_device.Start(false);
  XRobotMain(hardware);
}
