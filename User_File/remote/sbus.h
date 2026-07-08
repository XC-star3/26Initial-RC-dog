#ifndef SBUS_H
#define SBUS_H

#include "struct_typedef.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SBUS_CHANNEL_COUNT 16U
#define SBUS_FRAME_LEN 25U

typedef enum SbusSwitchState {
    SBUS_SWITCH_LOW = 0U,
    SBUS_SWITCH_MID = 1U,
    SBUS_SWITCH_HIGH = 2U,
} SbusSwitchState;

typedef struct SbusState {
    uint16_t ch[SBUS_CHANNEL_COUNT];
    int16_t norm[SBUS_CHANNEL_COUNT];
    uint8_t flag;
    uint8_t signal_lost;
    uint8_t failsafe;
    uint8_t online;
    uint32_t frame_count;
    uint32_t parse_error_count;
    uint32_t rx_event_count;
    uint32_t rx_byte_count;
    uint16_t last_rx_size;
    uint32_t last_update_ms;
} SbusState;

void Sbus_Init(void);
void Sbus_Process(void);
uint8_t Sbus_GetState(SbusState *state);
uint8_t Sbus_IsFresh(uint32_t timeout_ms);
uint8_t Sbus_Switch3(uint8_t channel);
uint8_t Sbus_Switch2(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif
