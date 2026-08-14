#ifndef __USB_OTG_H__
#define __USB_OTG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
void MX_USB_OTG_HS_PCD_Init(void);

#ifdef __cplusplus
}
#endif

#endif
