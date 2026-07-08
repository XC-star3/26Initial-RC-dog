#include "bsp_fdcan.h"

bool system_can[3];

CAN_Manage_Object CAN1_Manage_Object;
CAN_Manage_Object CAN2_Manage_Object;

static volatile uint32_t s_can_rx_count[2] = {0U, 0U};

static CAN_Manage_Object *can_object(FDCAN_HandleTypeDef *hfdcan, uint8_t *index)
{
    if (hfdcan == nullptr) {
        return nullptr;
    }

    if (hfdcan->Instance == FDCAN1) {
        if (index != nullptr) {
            *index = 0U;
        }
        return &CAN1_Manage_Object;
    }
    if (hfdcan->Instance == FDCAN2) {
        if (index != nullptr) {
            *index = 1U;
        }
        return &CAN2_Manage_Object;
    }
    return nullptr;
}

static void can_filter_init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef fdcan_filter = {};
    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x00U;
    fdcan_filter.FilterID2 = 0x00U;

    HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter);

    FDCAN_FilterTypeDef fdcan_ext_filter = {};
    fdcan_ext_filter.IdType = FDCAN_EXTENDED_ID;
    fdcan_ext_filter.FilterIndex = 0;
    fdcan_ext_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_ext_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_ext_filter.FilterID1 = 0x00U;
    fdcan_ext_filter.FilterID2 = 0x00U;
    HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_ext_filter);

    HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                  FDCAN_ACCEPT_IN_RX_FIFO0,
                                  FDCAN_REJECT,
                                  FDCAN_REJECT_REMOTE,
                                  FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(hfdcan, FDCAN_CFG_RX_FIFO0, 1);
}

void bsp_can_init(FDCAN_HandleTypeDef *hfdcan, CAN_Callback Callback_Function)
{
    uint8_t index = 0U;
    CAN_Manage_Object *obj = can_object(hfdcan, &index);
    if (obj == nullptr) {
        return;
    }

    obj->CAN_Handler = hfdcan;
    obj->Callback_Function = Callback_Function;

    if (system_can[index] == 0U) {
        can_filter_init(hfdcan);
        HAL_FDCAN_Start(hfdcan);
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
        system_can[index] = 1U;
    }
}

void fdcan_poll_rx(FDCAN_HandleTypeDef *hfdcan)
{
    uint8_t index = 0U;
    CAN_Manage_Object *obj = can_object(hfdcan, &index);
    if (obj == nullptr) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &obj->Rx_Header, obj->Rx_Buffer) != HAL_OK) {
            break;
        }

        s_can_rx_count[index]++;
        if (obj->Callback_Function != nullptr) {
            obj->Callback_Function(obj->Rx_Header, obj->Rx_Buffer);
        }
    }
}

uint32_t fdcan_rx_count(FDCAN_HandleTypeDef *hfdcan)
{
    uint8_t index = 0U;
    if (can_object(hfdcan, &index) == nullptr) {
        return 0U;
    }
    return s_can_rx_count[index];
}

uint8_t fdcan_dlc_to_bytes(uint32_t dlc)
{
    switch (dlc) {
    case FDCAN_DLC_BYTES_0:  return 0U;
    case FDCAN_DLC_BYTES_1:  return 1U;
    case FDCAN_DLC_BYTES_2:  return 2U;
    case FDCAN_DLC_BYTES_3:  return 3U;
    case FDCAN_DLC_BYTES_4:  return 4U;
    case FDCAN_DLC_BYTES_5:  return 5U;
    case FDCAN_DLC_BYTES_6:  return 6U;
    case FDCAN_DLC_BYTES_7:  return 7U;
    case FDCAN_DLC_BYTES_8:  return 8U;
    case FDCAN_DLC_BYTES_12: return 12U;
    case FDCAN_DLC_BYTES_16: return 16U;
    case FDCAN_DLC_BYTES_20: return 20U;
    case FDCAN_DLC_BYTES_24: return 24U;
    case FDCAN_DLC_BYTES_32: return 32U;
    case FDCAN_DLC_BYTES_48: return 48U;
    case FDCAN_DLC_BYTES_64: return 64U;
    default:                 return 0U;
    }
}

static uint32_t fdcan_bytes_to_dlc(uint32_t len)
{
    if (len <= 0U)  return FDCAN_DLC_BYTES_0;
    if (len <= 1U)  return FDCAN_DLC_BYTES_1;
    if (len <= 2U)  return FDCAN_DLC_BYTES_2;
    if (len <= 3U)  return FDCAN_DLC_BYTES_3;
    if (len <= 4U)  return FDCAN_DLC_BYTES_4;
    if (len <= 5U)  return FDCAN_DLC_BYTES_5;
    if (len <= 6U)  return FDCAN_DLC_BYTES_6;
    if (len <= 7U)  return FDCAN_DLC_BYTES_7;
    if (len <= 8U)  return FDCAN_DLC_BYTES_8;
    if (len <= 12U) return FDCAN_DLC_BYTES_12;
    if (len <= 16U) return FDCAN_DLC_BYTES_16;
    if (len <= 20U) return FDCAN_DLC_BYTES_20;
    if (len <= 24U) return FDCAN_DLC_BYTES_24;
    if (len <= 32U) return FDCAN_DLC_BYTES_32;
    if (len <= 48U) return FDCAN_DLC_BYTES_48;
    return FDCAN_DLC_BYTES_64;
}

static uint8_t fdcan_send_data(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t id,
                               uint32_t id_type,
                               uint8_t *data,
                               uint32_t len,
                               uint32_t fd_format,
                               uint32_t brs)
{
    FDCAN_TxHeaderTypeDef fdcan_tx_header = {};
    fdcan_tx_header.Identifier = id;
    fdcan_tx_header.IdType = id_type;
    fdcan_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    fdcan_tx_header.DataLength = fdcan_bytes_to_dlc(len);
    fdcan_tx_header.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
    fdcan_tx_header.BitRateSwitch = brs;
    fdcan_tx_header.FDFormat = fd_format;
    fdcan_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &fdcan_tx_header, data) == HAL_OK) ? 0U : 1U;
}

uint8_t fdcan_send_data_stand(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint32_t len)
{
    return fdcan_send_data(hfdcan, id, FDCAN_STANDARD_ID, data, len, FDCAN_CLASSIC_CAN, FDCAN_BRS_OFF);
}

uint8_t fdcan_send_std8(FDCAN_HandleTypeDef *hfdcan, uint32_t id, const uint8_t data[8])
{
    uint8_t tx[8] = {0U};
    if (data != nullptr) {
        for (uint8_t i = 0U; i < 8U; ++i) {
            tx[i] = data[i];
        }
    }

    return fdcan_send_data(hfdcan, id, FDCAN_STANDARD_ID, tx, 8U, FDCAN_CLASSIC_CAN, FDCAN_BRS_OFF);
}

uint8_t fdcan_send_data_Exten(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint32_t len)
{
    return fdcan_send_data(hfdcan, id, FDCAN_EXTENDED_ID, data, len, FDCAN_CLASSIC_CAN, FDCAN_BRS_OFF);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;

    uint8_t index = 0U;
    CAN_Manage_Object *obj = can_object(hfdcan, &index);
    if (obj == nullptr) {
        return;
    }

    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &obj->Rx_Header, obj->Rx_Buffer) == HAL_OK) {
        s_can_rx_count[index]++;
        if (obj->Callback_Function != nullptr) {
            obj->Callback_Function(obj->Rx_Header, obj->Rx_Buffer);
        }
    }
}
