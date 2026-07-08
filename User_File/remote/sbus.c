#include "sbus.h"

#include "usart.h"

#include <stddef.h>
#include <string.h>

#define SBUS_HEADER 0x0FU
#define SBUS_END 0x00U
#define SBUS_FLAG_SIGNAL_LOSS 0x04U
#define SBUS_FLAG_FAILSAFE 0x08U
#define SBUS_DMA_BUF_LEN 32U
#define SBUS_CENTER 1024
#define SBUS_RANGE 671
#define SBUS_DEADBAND 3

#if defined(__GNUC__)
#define SBUS_DMA_SECTION __attribute__((section(".dma_buffer"), aligned(32)))
#else
#define SBUS_DMA_SECTION
#endif

static uint8_t s_rx_dma[SBUS_DMA_BUF_LEN] SBUS_DMA_SECTION;
static uint8_t s_frame[SBUS_FRAME_LEN];
static uint8_t s_stream[SBUS_FRAME_LEN];
static uint8_t s_stream_len = 0U;
static volatile uint8_t s_frame_ready = 0U;
static SbusState s_state = {0};

static int16_t sbus_map_norm(uint16_t ch)
{
    int32_t v = ((int32_t)ch - SBUS_CENTER) * 100 / SBUS_RANGE;
    if (v > 100) {
        v = 100;
    } else if (v < -100) {
        v = -100;
    }

    if ((v > -SBUS_DEADBAND) && (v < SBUS_DEADBAND)) {
        v = 0;
    }
    return (int16_t)v;
}

static uint8_t sbus_parse_frame(SbusState *state, const uint8_t frame[SBUS_FRAME_LEN])
{
    if ((state == NULL) || (frame == NULL)) {
        return 0U;
    }

    if ((frame[0] != SBUS_HEADER) || (frame[24] != SBUS_END)) {
        return 0U;
    }

    state->flag = frame[23];
    state->ch[0]  = (uint16_t)(((frame[2] << 8)  + frame[1]) & 0x07FFU);
    state->ch[1]  = (uint16_t)(((frame[3] << 5)  + (frame[2] >> 3)) & 0x07FFU);
    state->ch[2]  = (uint16_t)(((frame[5] << 10) + (frame[4] << 2) + (frame[3] >> 6)) & 0x07FFU);
    state->ch[3]  = (uint16_t)(((frame[6] << 7)  + (frame[5] >> 1)) & 0x07FFU);
    state->ch[4]  = (uint16_t)(((frame[7] << 4)  + (frame[6] >> 4)) & 0x07FFU);
    state->ch[5]  = (uint16_t)(((frame[9] << 9)  + (frame[8] << 1) + (frame[7] >> 7)) & 0x07FFU);
    state->ch[6]  = (uint16_t)(((frame[10] << 6) + (frame[9] >> 2)) & 0x07FFU);
    state->ch[7]  = (uint16_t)(((frame[11] << 3) + (frame[10] >> 5)) & 0x07FFU);
    state->ch[8]  = (uint16_t)(((frame[13] << 8) + frame[12]) & 0x07FFU);
    state->ch[9]  = (uint16_t)(((frame[14] << 5) + (frame[13] >> 3)) & 0x07FFU);
    state->ch[10] = (uint16_t)(((frame[16] << 10) + (frame[15] << 2) + (frame[14] >> 6)) & 0x07FFU);
    state->ch[11] = (uint16_t)(((frame[17] << 7) + (frame[16] >> 1)) & 0x07FFU);
    state->ch[12] = (uint16_t)(((frame[18] << 4) + (frame[17] >> 4)) & 0x07FFU);
    state->ch[13] = (uint16_t)(((frame[20] << 9) + (frame[19] << 1) + (frame[18] >> 7)) & 0x07FFU);
    state->ch[14] = (uint16_t)(((frame[21] << 6) + (frame[20] >> 2)) & 0x07FFU);
    state->ch[15] = (uint16_t)(((frame[22] << 3) + (frame[21] >> 5)) & 0x07FFU);

    for (uint8_t i = 0U; i < SBUS_CHANNEL_COUNT; ++i) {
        state->norm[i] = sbus_map_norm(state->ch[i]);
    }

    state->signal_lost = ((state->flag & SBUS_FLAG_SIGNAL_LOSS) != 0U) ? 1U : 0U;
    state->failsafe = ((state->flag & SBUS_FLAG_FAILSAFE) != 0U) ? 1U : 0U;
    state->online = ((state->signal_lost == 0U) && (state->failsafe == 0U)) ? 1U : 0U;
    state->last_update_ms = HAL_GetTick();
    state->frame_count++;
    return 1U;
}

static void sbus_accept_stream_frame(void)
{
    memcpy(s_frame, s_stream, SBUS_FRAME_LEN);
    s_frame_ready = 1U;
    s_stream_len = 0U;
}

static void sbus_resync_stream(void)
{
    uint8_t keep = 0U;

    for (uint8_t i = 1U; i < s_stream_len; ++i) {
        if (s_stream[i] == SBUS_HEADER) {
            keep = (uint8_t)(s_stream_len - i);
            memmove(s_stream, &s_stream[i], keep);
            break;
        }
    }

    s_stream_len = keep;
}

static void sbus_feed_byte(uint8_t byte)
{
    if (s_stream_len == 0U) {
        if (byte != SBUS_HEADER) {
            s_state.parse_error_count++;
            return;
        }
        s_stream[s_stream_len++] = byte;
        return;
    }

    if (s_stream_len >= SBUS_FRAME_LEN) {
        s_state.parse_error_count++;
        s_stream_len = 0U;
        return;
    }

    s_stream[s_stream_len++] = byte;
    if (s_stream_len < SBUS_FRAME_LEN) {
        return;
    }

    if ((s_stream[0] == SBUS_HEADER) && (s_stream[SBUS_FRAME_LEN - 1U] == SBUS_END)) {
        sbus_accept_stream_frame();
        return;
    }

    s_state.parse_error_count++;
    sbus_resync_stream();
}

static void sbus_feed_bytes(const uint8_t *data, uint16_t len)
{
    if (data == NULL) {
        return;
    }
    for (uint16_t i = 0U; i < len; ++i) {
        sbus_feed_byte(data[i]);
    }
}

static void sbus_restart_rx(void)
{
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart5, s_rx_dma, SBUS_FRAME_LEN);
    if (huart5.hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }
}

void Sbus_Init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    memset(s_rx_dma, 0, sizeof(s_rx_dma));
    memset(s_frame, 0, sizeof(s_frame));
    memset(s_stream, 0, sizeof(s_stream));
    s_stream_len = 0U;
    s_frame_ready = 0U;
    sbus_restart_rx();
}

void Sbus_Process(void)
{
    uint8_t frame[SBUS_FRAME_LEN];
    uint8_t ready = 0U;

    __disable_irq();
    if (s_frame_ready != 0U) {
        memcpy(frame, s_frame, sizeof(frame));
        s_frame_ready = 0U;
        ready = 1U;
    }
    __enable_irq();

    if (ready == 0U) {
        return;
    }

    if (sbus_parse_frame(&s_state, frame) == 0U) {
        s_state.parse_error_count++;
    }
}

uint8_t Sbus_GetState(SbusState *state)
{
    if (state == NULL) {
        return 0U;
    }
    *state = s_state;
    return s_state.online;
}

uint8_t Sbus_IsFresh(uint32_t timeout_ms)
{
    if (s_state.last_update_ms == 0U) {
        return 0U;
    }
    if ((uint32_t)(HAL_GetTick() - s_state.last_update_ms) > timeout_ms) {
        return 0U;
    }
    return (s_state.online != 0U) ? 1U : 0U;
}

uint8_t Sbus_Switch3(uint8_t channel)
{
    if (channel >= SBUS_CHANNEL_COUNT) {
        return SBUS_SWITCH_LOW;
    }
    if (s_state.ch[channel] < 700U) {
        return SBUS_SWITCH_LOW;
    }
    if (s_state.ch[channel] < 1350U) {
        return SBUS_SWITCH_MID;
    }
    return SBUS_SWITCH_HIGH;
}

uint8_t Sbus_Switch2(uint8_t channel)
{
    if (channel >= SBUS_CHANNEL_COUNT) {
        return 0U;
    }
    return (s_state.ch[channel] < 1000U) ? 0U : 1U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if ((huart == NULL) || (huart->Instance != UART5)) {
        return;
    }

    SCB_InvalidateDCache_by_Addr((uint32_t *)s_rx_dma, SBUS_DMA_BUF_LEN);
    s_state.rx_event_count++;
    s_state.last_rx_size = Size;
    if ((Size == 0U) || (Size > SBUS_DMA_BUF_LEN)) {
        s_state.parse_error_count++;
    } else {
        s_state.rx_byte_count += Size;
        sbus_feed_bytes(s_rx_dma, Size);
    }
    sbus_restart_rx();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART5)) {
        return;
    }

    s_frame_ready = 0U;
    s_state.online = 0U;
    sbus_restart_rx();
}
