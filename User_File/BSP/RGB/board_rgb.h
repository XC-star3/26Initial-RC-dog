//
// 板载 RGB 灯驱动（从 CtrBoard-H7_CAN 例程移植）
// 移植改动：
//   - 不再重定义 HAL_SPI_MspInit / HAL_SPI_MspDeInit（A 的 spi.c 已经实现 SPI2 版本）。
//   - BoardRgb_Init 内部先手动配 SPI6 时钟/GPIO，再调 HAL_SPI_Init，其 Msp 回调对 SPI6 不做处理即可。
//

#ifndef __BOARD_RGB_H__
#define __BOARD_RGB_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void BoardRgb_Init(void);
void BoardRgb_SetColor(uint8_t r, uint8_t g, uint8_t b);
void BoardRgb_Off(void);

#ifdef __cplusplus
}
#endif

#endif // __BOARD_RGB_H__
