#include "usb_otg.h"

PCD_HandleTypeDef hpcd_USB_OTG_HS;

void MX_USB_OTG_HS_PCD_Init(void)
{
  hpcd_USB_OTG_HS.Instance = USB_OTG_HS;
  hpcd_USB_OTG_HS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_HS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_HS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.phy_itface = USB_OTG_EMBEDDED_PHY;
  hpcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_HS.Init.use_external_vbus = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_HS) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_PCD_MspInit(PCD_HandleTypeDef *pcd)
{
  if (pcd->Instance != USB_OTG_HS)
  {
    return;
  }
  RCC_PeriphCLKInitTypeDef clock = {0};
  GPIO_InitTypeDef gpio = {0};
  clock.PeriphClockSelection = RCC_PERIPHCLK_USB;
  clock.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_PWREx_EnableUSBVoltageDetector();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USB_OTG_HS_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(GPIOA, &gpio);
  HAL_NVIC_SetPriority(OTG_HS_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcd)
{
  if (pcd->Instance != USB_OTG_HS)
  {
    return;
  }
  __HAL_RCC_USB_OTG_HS_CLK_DISABLE();
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
  HAL_NVIC_DisableIRQ(OTG_HS_IRQn);
}
