#ifndef USB_FRAME_PROTOCOL_H
#define USB_FRAME_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_FRAME_SIZE                 40U
#define USB_FRAME_PAYLOAD_SIZE         28U
#define USB_FRAME_MSG_VIRTUAL_RC       0x10U
#define USB_FRAME_MSG_VIRTUAL_IMU      0x11U

#define USB_RC_FLAG_DEADMAN_HELD       (1U << 0)
#define USB_RC_FLAG_MOTION_ENABLE      (1U << 1)
#define USB_RC_FLAG_SMOOTH_STOP        (1U << 2)
#define USB_RC_FLAG_KNOWN_MASK         0x0007U
#define USB_RC_CHANNEL_VALID_MASK      0x0067U

typedef struct UsbVirtualRcSample {
    uint32_t session_id;
    uint32_t host_time_ms;
    uint8_t main_switch;
    uint8_t sub_switch;
    uint16_t command_flags;
    int16_t yaw_permille;
    int16_t forward_permille;
    int16_t speed_permille;
    uint16_t channel_valid_mask;
    uint32_t command_counter;
    uint16_t frame_seq;
    uint32_t received_ms;
    uint32_t generation;
} UsbVirtualRcSample;

typedef struct UsbFrameProtocolDiag {
    uint32_t rx_bytes;
    uint32_t valid_control_frames;
    uint32_t valid_imu_frames;
    uint32_t header_error_count;
    uint32_t crc_error_count;
    uint32_t payload_reject_count;
    uint32_t sequence_gap_count;
    uint32_t parser_timeout_count;
    uint16_t last_frame_seq;
    uint8_t last_frame_seq_valid;
    uint8_t parser_bytes;
} UsbFrameProtocolDiag;

void UsbFrameProtocol_Init(void);

/* Returns 1 when the byte belongs to a binary frame, 0 for standalone ASCII. */
uint8_t UsbFrameProtocol_FeedByte(uint8_t byte, uint32_t now_ms);
void UsbFrameProtocol_Tick(uint32_t now_ms);

uint8_t UsbFrameProtocol_GetVirtualRc(UsbVirtualRcSample *sample);
void UsbFrameProtocol_GetDiag(UsbFrameProtocolDiag *diag);
uint16_t UsbFrameProtocol_Crc16(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
