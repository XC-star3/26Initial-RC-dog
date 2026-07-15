#include "usb_frame_protocol.h"

#include <string.h>

#define USB_FRAME_MAGIC0             0xA5U
#define USB_FRAME_MAGIC1             0x5AU
#define USB_FRAME_VERSION            0x01U
#define USB_FRAME_PARSER_TIMEOUT_MS  50U

static uint8_t s_frame[USB_FRAME_SIZE];
static uint8_t s_frame_len = 0U;
static uint32_t s_frame_started_ms = 0U;
static UsbVirtualRcSample s_latest_rc = {};
static uint8_t s_have_rc = 0U;
static UsbFrameProtocolDiag s_diag = {};

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)read_u16_le(data);
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

uint16_t UsbFrameProtocol_Crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    if (data == nullptr) {
        return crc;
    }
    for (uint16_t i = 0U; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = ((crc & 0x8000U) != 0U) ?
                (uint16_t)((crc << 1) ^ 0x1021U) :
                (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint8_t permille_is_valid(int16_t value)
{
    return ((value >= -1000) && (value <= 1000)) ? 1U : 0U;
}

static uint8_t decode_virtual_rc(uint32_t now_ms)
{
    const uint8_t *payload = &s_frame[10];
    UsbVirtualRcSample sample = {};
    sample.session_id = read_u32_le(&payload[0]);
    sample.host_time_ms = read_u32_le(&payload[4]);
    sample.main_switch = payload[8];
    sample.sub_switch = payload[9];
    sample.command_flags = read_u16_le(&payload[10]);
    sample.yaw_permille = read_i16_le(&payload[12]);
    sample.forward_permille = read_i16_le(&payload[14]);
    sample.speed_permille = read_i16_le(&payload[16]);
    const int16_t legacy_reserved_axis_0 = read_i16_le(&payload[18]);
    const int16_t legacy_reserved_axis_1 = read_i16_le(&payload[20]);
    sample.channel_valid_mask = read_u16_le(&payload[22]);
    sample.command_counter = read_u32_le(&payload[24]);
    sample.frame_seq = read_u16_le(&s_frame[6]);
    sample.received_ms = now_ms;

    const uint8_t valid =
        ((sample.session_id != 0U) &&
         (sample.main_switch <= 2U) &&
         (sample.sub_switch <= 2U) &&
         ((sample.command_flags & (uint16_t)~USB_RC_FLAG_KNOWN_MASK) == 0U) &&
         (sample.channel_valid_mask == USB_RC_CHANNEL_VALID_MASK) &&
         (permille_is_valid(sample.yaw_permille) != 0U) &&
         (permille_is_valid(sample.forward_permille) != 0U) &&
         (permille_is_valid(sample.speed_permille) != 0U) &&
         (permille_is_valid(legacy_reserved_axis_0) != 0U) &&
         (permille_is_valid(legacy_reserved_axis_1) != 0U)) ? 1U : 0U;
    if (valid == 0U) {
        s_diag.payload_reject_count++;
        return 0U;
    }

    sample.generation = s_latest_rc.generation + 1U;
    s_latest_rc = sample;
    s_have_rc = 1U;
    s_diag.valid_control_frames++;
    return 1U;
}

static void update_sequence_diag(uint16_t seq)
{
    if (s_diag.last_frame_seq_valid != 0U) {
        const uint16_t expected = (uint16_t)(s_diag.last_frame_seq + 1U);
        if (seq != expected) {
            s_diag.sequence_gap_count++;
        }
    }
    s_diag.last_frame_seq = seq;
    s_diag.last_frame_seq_valid = 1U;
}

static void process_complete_frame(uint32_t now_ms)
{
    if ((s_frame[2] != USB_FRAME_VERSION) ||
        ((s_frame[3] != USB_FRAME_MSG_VIRTUAL_RC) &&
         (s_frame[3] != USB_FRAME_MSG_VIRTUAL_IMU)) ||
        (s_frame[4] != 0U) || (s_frame[5] != 0U) ||
        (read_u16_le(&s_frame[8]) != USB_FRAME_PAYLOAD_SIZE)) {
        s_diag.header_error_count++;
        return;
    }

    const uint16_t expected_crc = read_u16_le(&s_frame[38]);
    const uint16_t actual_crc = UsbFrameProtocol_Crc16(&s_frame[2], 36U);
    if (expected_crc != actual_crc) {
        s_diag.crc_error_count++;
        return;
    }

    update_sequence_diag(read_u16_le(&s_frame[6]));
    if (s_frame[3] == USB_FRAME_MSG_VIRTUAL_RC) {
        (void)decode_virtual_rc(now_ms);
    } else {
        s_diag.valid_imu_frames++;
    }
}

static void resync_after_invalid_frame(uint32_t now_ms)
{
    uint8_t next = USB_FRAME_SIZE;
    for (uint8_t i = 1U; i + 1U < USB_FRAME_SIZE; ++i) {
        if ((s_frame[i] == USB_FRAME_MAGIC0) &&
            (s_frame[i + 1U] == USB_FRAME_MAGIC1)) {
            next = i;
            break;
        }
    }
    if (next < USB_FRAME_SIZE) {
        s_frame_len = (uint8_t)(USB_FRAME_SIZE - next);
        memmove(s_frame, &s_frame[next], s_frame_len);
        s_frame_started_ms = now_ms;
    } else if (s_frame[USB_FRAME_SIZE - 1U] == USB_FRAME_MAGIC0) {
        s_frame[0] = USB_FRAME_MAGIC0;
        s_frame_len = 1U;
        s_frame_started_ms = now_ms;
    } else {
        s_frame_len = 0U;
    }
}

void UsbFrameProtocol_Init(void)
{
    memset(s_frame, 0, sizeof(s_frame));
    memset(&s_latest_rc, 0, sizeof(s_latest_rc));
    memset(&s_diag, 0, sizeof(s_diag));
    s_frame_len = 0U;
    s_frame_started_ms = 0U;
    s_have_rc = 0U;
}

uint8_t UsbFrameProtocol_FeedByte(uint8_t byte, uint32_t now_ms)
{
    s_diag.rx_bytes++;
    if (s_frame_len == 0U) {
        if (byte != USB_FRAME_MAGIC0) {
            return 0U;
        }
        s_frame[0] = byte;
        s_frame_len = 1U;
        s_frame_started_ms = now_ms;
        s_diag.parser_bytes = s_frame_len;
        return 1U;
    }

    if (s_frame_len == 1U) {
        if (byte == USB_FRAME_MAGIC1) {
            s_frame[1] = byte;
            s_frame_len = 2U;
            s_diag.parser_bytes = s_frame_len;
            return 1U;
        }
        if (byte == USB_FRAME_MAGIC0) {
            s_frame_started_ms = now_ms;
            return 1U;
        }
        s_frame_len = 0U;
        s_diag.parser_bytes = 0U;
        return 0U;
    }

    s_frame[s_frame_len++] = byte;
    if (s_frame_len < USB_FRAME_SIZE) {
        s_diag.parser_bytes = s_frame_len;
        return 1U;
    }

    const uint32_t errors_before = s_diag.header_error_count + s_diag.crc_error_count;
    process_complete_frame(now_ms);
    const uint32_t errors_after = s_diag.header_error_count + s_diag.crc_error_count;
    if (errors_after != errors_before) {
        resync_after_invalid_frame(now_ms);
    } else {
        s_frame_len = 0U;
    }
    s_diag.parser_bytes = s_frame_len;
    return 1U;
}

void UsbFrameProtocol_Tick(uint32_t now_ms)
{
    if ((s_frame_len != 0U) &&
        ((uint32_t)(now_ms - s_frame_started_ms) > USB_FRAME_PARSER_TIMEOUT_MS)) {
        s_frame_len = 0U;
        s_diag.parser_bytes = 0U;
        s_diag.parser_timeout_count++;
    }
}

uint8_t UsbFrameProtocol_GetVirtualRc(UsbVirtualRcSample *sample)
{
    if ((sample == nullptr) || (s_have_rc == 0U)) {
        return 0U;
    }
    *sample = s_latest_rc;
    return 1U;
}

void UsbFrameProtocol_GetDiag(UsbFrameProtocolDiag *diag)
{
    if (diag != nullptr) {
        *diag = s_diag;
    }
}
