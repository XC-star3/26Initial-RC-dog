#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void DebugUart_Init(void);
void DebugUart_Process(void);
void DebugUart_LogCanRx(uint8_t port, uint32_t id, const uint8_t *data, uint8_t len);
void DebugUart_Printf(const char *fmt, ...);
void DebugUart_WriteRaw(const uint8_t *data, uint16_t len);
void DebugUart_SetLogVerbose(uint8_t enable);
uint8_t DebugUart_GetLogVerbose(void);
void DebugUart_SetCanRxVerbose(uint8_t enable);
uint8_t DebugUart_GetCanRxVerbose(void);
uint8_t DebugUart_IsHostOpen(void);
uint8_t DebugUart_WaitForHost(uint32_t timeout_ms);
int DebugUart_GetByte(void);

#ifdef __cplusplus
}
#endif

#endif
