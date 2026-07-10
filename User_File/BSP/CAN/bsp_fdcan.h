#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include "fdcan.h"
#include "stm32h7xx_hal.h"

typedef void (*CAN_Callback)(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);

#define FDCAN_RX_BUFFER_BYTES 64U

#define FDCAN_RECOVERY_FAILED    0U
#define FDCAN_RECOVERY_HEALTHY   1U
#define FDCAN_RECOVERY_RESTARTED 2U

struct CAN_Manage_Object
{
    FDCAN_HandleTypeDef *CAN_Handler;
    CAN_Callback Callback_Function;
    FDCAN_RxHeaderTypeDef Rx_Header;
    uint8_t Rx_Buffer[FDCAN_RX_BUFFER_BYTES];
};

extern bool system_can[3];
extern CAN_Manage_Object CAN1_Manage_Object;
extern CAN_Manage_Object CAN2_Manage_Object;
extern CAN_Manage_Object CAN3_Manage_Object;

void bsp_can_init(FDCAN_HandleTypeDef *hfdcan, CAN_Callback Callback_Function);
void fdcan_poll_rx(FDCAN_HandleTypeDef *hfdcan);
uint32_t fdcan_rx_count(FDCAN_HandleTypeDef *hfdcan);
uint32_t fdcan_tx_free_level(FDCAN_HandleTypeDef *hfdcan);
uint8_t fdcan_abort_all_tx(FDCAN_HandleTypeDef *hfdcan);
uint8_t fdcan_recover_bus_off(FDCAN_HandleTypeDef *hfdcan);
uint8_t fdcan_dlc_to_bytes(uint32_t dlc);
uint8_t fdcan_send_std8(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t data[8]);
uint8_t fdcan_send_data_stand(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint32_t len);
uint8_t fdcan_send_data_Exten(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint32_t len);

#endif
