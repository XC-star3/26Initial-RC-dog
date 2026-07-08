//
// 板载 RGB 灯驱动实现（从 CtrBoard-H7_CAN 例程移植）
// A 的 spi.c 已定义 HAL_SPI_MspInit 处理 SPI2，这里把 SPI6 的底层配置内联进
// BoardRgb_Init，避免重复定义全局 Msp 函数。
//

#include "board_rgb.h"

#include "stm32h7xx_hal_spi.h"

#define BOARD_RGB_SPI_INSTANCE        SPI6
#define BOARD_RGB_SPI_BAUD_PRESCALER  SPI_BAUDRATEPRESCALER_4
#define BOARD_RGB_LOW_LEVEL           0x60U
#define BOARD_RGB_HIGH_LEVEL          0x78U
#define BOARD_RGB_RESET_BYTES         80U

static SPI_HandleTypeDef s_board_rgb_spi;
static uint8_t s_board_rgb_ready = 0U;

static void BoardRgb_Spi6LowLevelInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI6;
    PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        Error_Handler();
    }

    __HAL_RCC_SPI6_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_SPI6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void BoardRgb_WriteByte(uint8_t *txbuf, uint8_t offset, uint8_t value)
{
    for (uint8_t i = 0U; i < 8U; ++i) {
        txbuf[offset + (7U - i)] =
            (((value >> i) & 0x01U) != 0U) ? BOARD_RGB_HIGH_LEVEL : BOARD_RGB_LOW_LEVEL;
    }
}

void BoardRgb_Init(void)
{
    if (s_board_rgb_ready != 0U) {
        return;
    }

    BoardRgb_Spi6LowLevelInit();

    s_board_rgb_spi.Instance = BOARD_RGB_SPI_INSTANCE;
    s_board_rgb_spi.Init.Mode = SPI_MODE_MASTER;
    s_board_rgb_spi.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
    s_board_rgb_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    s_board_rgb_spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    s_board_rgb_spi.Init.CLKPhase = SPI_PHASE_2EDGE;
    s_board_rgb_spi.Init.NSS = SPI_NSS_SOFT;
    s_board_rgb_spi.Init.BaudRatePrescaler = BOARD_RGB_SPI_BAUD_PRESCALER;
    s_board_rgb_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    s_board_rgb_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    s_board_rgb_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    s_board_rgb_spi.Init.CRCPolynomial = 0x0;
    s_board_rgb_spi.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    s_board_rgb_spi.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    s_board_rgb_spi.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    s_board_rgb_spi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    s_board_rgb_spi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    s_board_rgb_spi.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    s_board_rgb_spi.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    s_board_rgb_spi.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    s_board_rgb_spi.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    s_board_rgb_spi.Init.IOSwap = SPI_IO_SWAP_DISABLE;

    if (HAL_SPI_Init(&s_board_rgb_spi) != HAL_OK) {
        Error_Handler();
    }

    s_board_rgb_ready = 1U;
    BoardRgb_Off();
}

void BoardRgb_SetColor(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    uint8_t reset_buf[BOARD_RGB_RESET_BYTES] = {0};

    if (s_board_rgb_ready == 0U) {
        BoardRgb_Init();
    }

    BoardRgb_WriteByte(txbuf, 0U, g);
    BoardRgb_WriteByte(txbuf, 8U, r);
    BoardRgb_WriteByte(txbuf, 16U, b);

    HAL_SPI_Transmit(&s_board_rgb_spi, reset_buf, BOARD_RGB_RESET_BYTES, 0xFFFFU);
    HAL_SPI_Transmit(&s_board_rgb_spi, txbuf, sizeof(txbuf), 0xFFFFU);
    HAL_SPI_Transmit(&s_board_rgb_spi, reset_buf, BOARD_RGB_RESET_BYTES, 0xFFFFU);
}

void BoardRgb_Off(void)
{
    BoardRgb_SetColor(0U, 0U, 0U);
}
