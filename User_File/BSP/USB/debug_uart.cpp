#include "debug_uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

#define DEBUG_UART_TX_BUFFER_SIZE 4096U
#define DEBUG_UART_TX_CHUNK_SIZE  256U
#define DEBUG_UART_CAN_RING_SIZE  64U
#define DEBUG_UART_CAN_DRAIN_MAX  16U

extern USBD_HandleTypeDef hUsbDeviceHS;

typedef struct {
    uint8_t port;
    uint32_t id;
    uint8_t len;
    uint8_t data[64];
} debug_uart_can_frame_t;

static uint8_t s_debug_uart_ready = 0U;
static debug_uart_can_frame_t s_can_ring[DEBUG_UART_CAN_RING_SIZE];
static volatile uint32_t s_can_ring_head = 0U;
static volatile uint32_t s_can_ring_tail = 0U;
static volatile uint32_t s_can_ring_drops = 0U;
static uint8_t s_tx_pending[DEBUG_UART_TX_BUFFER_SIZE];
static uint16_t s_tx_pending_len = 0U;
static volatile uint8_t s_log_verbose = 1U;
static volatile uint8_t s_can_rx_verbose = 1U;

static void DebugUart_FlushTx(void)
{
    if ((s_debug_uart_ready == 0U) || (s_tx_pending_len == 0U)) {
        return;
    }

    if ((hUsbDeviceHS.pClassData == NULL) ||
        (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)) {
        return;
    }

    uint16_t chunk_len = s_tx_pending_len;
    if (chunk_len > DEBUG_UART_TX_CHUNK_SIZE) {
        chunk_len = DEBUG_UART_TX_CHUNK_SIZE;
    }

    if (CDC_Transmit_HS(s_tx_pending, chunk_len) == USBD_OK) {
        if (s_tx_pending_len > chunk_len) {
            memmove(s_tx_pending, &s_tx_pending[chunk_len], s_tx_pending_len - chunk_len);
        }
        s_tx_pending_len = (uint16_t)(s_tx_pending_len - chunk_len);
    }
}

void DebugUart_Init(void)
{
    if (s_debug_uart_ready != 0U) {
        return;
    }

    memset(s_can_ring, 0, sizeof(s_can_ring));
    s_can_ring_head = 0U;
    s_can_ring_tail = 0U;
    s_can_ring_drops = 0U;
    s_tx_pending_len = 0U;
    s_log_verbose = 1U;
    s_can_rx_verbose = 1U;
    s_debug_uart_ready = 1U;
}

void DebugUart_WriteRaw(const uint8_t *data, uint16_t len)
{
    if ((s_debug_uart_ready == 0U) || (data == nullptr) || (len == 0U)) {
        return;
    }

    DebugUart_FlushTx();

    if ((uint32_t)s_tx_pending_len + len > DEBUG_UART_TX_BUFFER_SIZE) {
        len = (uint16_t)(DEBUG_UART_TX_BUFFER_SIZE - s_tx_pending_len);
    }
    if (len == 0U) {
        return;
    }

    memcpy(&s_tx_pending[s_tx_pending_len], data, len);
    s_tx_pending_len = (uint16_t)(s_tx_pending_len + len);
}

void DebugUart_Printf(const char *fmt, ...)
{
    char buffer[192];
    va_list args;
    int length;

    if (s_debug_uart_ready == 0U) {
        return;
    }

    va_start(args, fmt);
    length = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (length <= 0) {
        return;
    }
    if (length > ((int)sizeof(buffer) - 1)) {
        length = (int)sizeof(buffer) - 1;
    }

    DebugUart_FlushTx();

    uint16_t copy_len = (uint16_t)length;
    if ((uint32_t)s_tx_pending_len + copy_len > DEBUG_UART_TX_BUFFER_SIZE) {
        copy_len = (uint16_t)(DEBUG_UART_TX_BUFFER_SIZE - s_tx_pending_len);
    }
    if (copy_len == 0U) {
        return;
    }

    memcpy(&s_tx_pending[s_tx_pending_len], buffer, copy_len);
    s_tx_pending_len = (uint16_t)(s_tx_pending_len + copy_len);
}

void DebugUart_LogCanRx(uint8_t port, uint32_t id, const uint8_t *data, uint8_t len)
{
    if ((s_can_rx_verbose == 0U) || (data == nullptr)) {
        return;
    }

    uint8_t copy_len = (len > 64U) ? 64U : len;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t head = s_can_ring_head;
    uint32_t tail = s_can_ring_tail;
    if ((head - tail) >= DEBUG_UART_CAN_RING_SIZE) {
        s_can_ring_drops++;
        __set_PRIMASK(primask);
        return;
    }

    debug_uart_can_frame_t *slot = &s_can_ring[head % DEBUG_UART_CAN_RING_SIZE];
    slot->port = port;
    slot->id = id;
    slot->len = copy_len;
    memset(slot->data, 0, sizeof(slot->data));
    memcpy(slot->data, data, copy_len);
    __DMB();
    s_can_ring_head = head + 1U;

    __set_PRIMASK(primask);
}

void DebugUart_SetLogVerbose(uint8_t enable)
{
    s_log_verbose = (enable != 0U) ? 1U : 0U;
}

uint8_t DebugUart_GetLogVerbose(void)
{
    return s_log_verbose;
}

void DebugUart_SetCanRxVerbose(uint8_t enable)
{
    s_can_rx_verbose = (enable != 0U) ? 1U : 0U;
}

uint8_t DebugUart_GetCanRxVerbose(void)
{
    return s_can_rx_verbose;
}

uint8_t DebugUart_IsHostOpen(void)
{
    if (s_debug_uart_ready == 0U) {
        return 0U;
    }
    if ((hUsbDeviceHS.pClassData == NULL) ||
        (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED)) {
        return 0U;
    }
    return CDC_HostIsOpen_HS();
}

int DebugUart_GetByte(void)
{
    return CDC_GetByte_HS();
}

uint8_t DebugUart_WaitForHost(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout_ms) {
        if (DebugUart_IsHostOpen() != 0U) {
            return 1U;
        }
        DebugUart_Process();
        HAL_Delay(10U);
    }

    return DebugUart_IsHostOpen();
}

void DebugUart_Process(void)
{
    if (s_debug_uart_ready == 0U) {
        return;
    }

    DebugUart_FlushTx();

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t drops = s_can_ring_drops;
    s_can_ring_drops = 0U;
    __set_PRIMASK(primask);

    if ((drops != 0U) && (s_log_verbose != 0U)) {
        DebugUart_Printf("[CAN LOG OVERFLOW: %lu frames dropped]\r\n", (unsigned long)drops);
    }

    if (s_log_verbose == 0U) {
        uint32_t drop_primask = __get_PRIMASK();
        __disable_irq();
        s_can_ring_tail = s_can_ring_head;
        __set_PRIMASK(drop_primask);
        DebugUart_FlushTx();
        return;
    }

    for (uint8_t drained = 0U; drained < DEBUG_UART_CAN_DRAIN_MAX; ++drained) {
        uint32_t head = s_can_ring_head;
        uint32_t tail = s_can_ring_tail;
        if (head == tail) {
            break;
        }

        debug_uart_can_frame_t frame = s_can_ring[tail % DEBUG_UART_CAN_RING_SIZE];
        __DMB();
        s_can_ring_tail = tail + 1U;

        DebugUart_Printf("CAN RX: port=%u id=0x%03lX len=%u bytes=[",
                         frame.port, (unsigned long)frame.id, frame.len);
        uint8_t print_len = (frame.len > 16U) ? 16U : frame.len;
        for (uint8_t i = 0U; i < print_len; ++i) {
            DebugUart_Printf("%s%02X", (i == 0U) ? "" : " ", frame.data[i]);
        }
        if (frame.len > print_len) {
            DebugUart_Printf(" ...");
        }
        DebugUart_Printf("]\r\n");
    }

    DebugUart_FlushTx();
}
