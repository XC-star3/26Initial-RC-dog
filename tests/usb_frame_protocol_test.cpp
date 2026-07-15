#include "usb_frame_protocol.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8);
}

static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
    data[2] = (uint8_t)((value >> 16) & 0xFFU);
    data[3] = (uint8_t)(value >> 24);
}

static void test_control_vector(void)
{
    uint8_t frame[USB_FRAME_SIZE] = {
        0xA5, 0x5A, 0x01, 0x10, 0x00, 0x00, 0x34, 0x12, 0x1C, 0x00,
        0x44, 0x33, 0x22, 0x11, 0x04, 0x03, 0x02, 0x01,
        0x02, 0x00, 0x03, 0x00, 0x06, 0xFF, 0xF4, 0x01,
        0xE0, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x67, 0x00,
        0x0A, 0x00, 0x00, 0x00, 0xCF, 0x96,
    };

    assert(UsbFrameProtocol_Crc16(&frame[2], 36U) == 0x96CFU);
    assert(UsbFrameProtocol_FeedByte((uint8_t)'p', 1U) == 0U);
    assert(UsbFrameProtocol_FeedByte((uint8_t)'Y', 1U) == 0U);
    for (uint8_t i = 0U; i < USB_FRAME_SIZE; ++i) {
        assert(UsbFrameProtocol_FeedByte(frame[i], (uint32_t)(10U + i)) == 1U);
    }

    UsbVirtualRcSample sample = {};
    assert(UsbFrameProtocol_GetVirtualRc(&sample) == 1U);
    assert(sample.session_id == 0x11223344U);
    assert(sample.host_time_ms == 0x01020304U);
    assert(sample.main_switch == 2U);
    assert(sample.sub_switch == 0U);
    assert(sample.command_flags == 0x0003U);
    assert(sample.yaw_permille == -250);
    assert(sample.forward_permille == 500);
    assert(sample.speed_permille == -800);
    assert(sample.channel_valid_mask == 0x0067U);
    assert(sample.command_counter == 10U);
    assert(sample.frame_seq == 0x1234U);
}

static void test_crc_reject_and_imu_ignore(void)
{
    uint8_t bad[USB_FRAME_SIZE] = {};
    bad[0] = 0xA5U;
    bad[1] = 0x5AU;
    bad[2] = 0x01U;
    bad[3] = USB_FRAME_MSG_VIRTUAL_RC;
    bad[8] = USB_FRAME_PAYLOAD_SIZE;
    for (uint8_t i = 0U; i < USB_FRAME_SIZE; ++i) {
        (void)UsbFrameProtocol_FeedByte(bad[i], 100U);
    }

    UsbFrameProtocolDiag before = {};
    UsbFrameProtocol_GetDiag(&before);
    assert(before.crc_error_count == 1U);

    uint8_t imu[USB_FRAME_SIZE] = {};
    imu[0] = 0xA5U;
    imu[1] = 0x5AU;
    imu[2] = 0x01U;
    imu[3] = USB_FRAME_MSG_VIRTUAL_IMU;
    imu[6] = 0x35U;
    imu[7] = 0x12U;
    imu[8] = USB_FRAME_PAYLOAD_SIZE;
    write_u16_le(&imu[38], UsbFrameProtocol_Crc16(&imu[2], 36U));
    for (uint8_t i = 0U; i < USB_FRAME_SIZE; ++i) {
        assert(UsbFrameProtocol_FeedByte(imu[i], 200U) == 1U);
    }

    UsbFrameProtocolDiag after = {};
    UsbFrameProtocol_GetDiag(&after);
    assert(after.valid_imu_frames == 1U);
    assert(after.valid_control_frames == 1U);
}

static void test_timeout_and_resync(void)
{
    UsbFrameProtocol_Init();
    assert(UsbFrameProtocol_FeedByte(0xA5U, 10U) == 1U);
    assert(UsbFrameProtocol_FeedByte(0x5AU, 10U) == 1U);
    UsbFrameProtocol_Tick(61U);
    assert(UsbFrameProtocol_FeedByte((uint8_t)'p', 62U) == 0U);

    uint8_t valid[USB_FRAME_SIZE] = {};
    valid[0] = 0xA5U;
    valid[1] = 0x5AU;
    valid[2] = 0x01U;
    valid[3] = USB_FRAME_MSG_VIRTUAL_RC;
    valid[8] = USB_FRAME_PAYLOAD_SIZE;
    write_u32_le(&valid[10], 0x12345678U);
    write_u16_le(&valid[26], (uint16_t)(int16_t)-1000);
    write_u16_le(&valid[32], USB_RC_CHANNEL_VALID_MASK);
    write_u16_le(&valid[38], UsbFrameProtocol_Crc16(&valid[2], 36U));

    uint8_t damaged[USB_FRAME_SIZE] = {};
    damaged[0] = 0xA5U;
    damaged[1] = 0x5AU;
    memcpy(&damaged[20], valid, 20U);
    for (uint8_t i = 0U; i < USB_FRAME_SIZE; ++i) {
        assert(UsbFrameProtocol_FeedByte(damaged[i], 100U) == 1U);
    }
    for (uint8_t i = 20U; i < USB_FRAME_SIZE; ++i) {
        assert(UsbFrameProtocol_FeedByte(valid[i], 101U) == 1U);
    }

    UsbVirtualRcSample sample = {};
    UsbFrameProtocolDiag diag = {};
    assert(UsbFrameProtocol_GetVirtualRc(&sample) == 1U);
    assert(sample.session_id == 0x12345678U);
    UsbFrameProtocol_GetDiag(&diag);
    assert(diag.parser_timeout_count == 1U);
    assert(diag.header_error_count == 1U);
    assert(diag.valid_control_frames == 1U);
}

static void test_legacy_reserved_axes_are_ignored(void)
{
    UsbFrameProtocol_Init();
    uint8_t frame[USB_FRAME_SIZE] = {};
    frame[0] = 0xA5U;
    frame[1] = 0x5AU;
    frame[2] = 0x01U;
    frame[3] = USB_FRAME_MSG_VIRTUAL_RC;
    frame[8] = USB_FRAME_PAYLOAD_SIZE;
    write_u32_le(&frame[10], 1U);
    write_u16_le(&frame[26], (uint16_t)(int16_t)-1000);
    write_u16_le(&frame[28], (uint16_t)(int16_t)1000);
    write_u16_le(&frame[30], (uint16_t)(int16_t)-1000);
    write_u16_le(&frame[32], USB_RC_CHANNEL_VALID_MASK);
    write_u16_le(&frame[38], UsbFrameProtocol_Crc16(&frame[2], 36U));

    for (uint8_t i = 0U; i < USB_FRAME_SIZE; ++i) {
        assert(UsbFrameProtocol_FeedByte(frame[i], 300U) == 1U);
    }

    UsbVirtualRcSample sample = {};
    UsbFrameProtocolDiag diag = {};
    assert(UsbFrameProtocol_GetVirtualRc(&sample) == 1U);
    assert(sample.session_id == 1U);
    UsbFrameProtocol_GetDiag(&diag);
    assert(diag.payload_reject_count == 0U);
    assert(diag.valid_control_frames == 1U);
}

int main(void)
{
    UsbFrameProtocol_Init();
    test_control_vector();
    test_crc_reject_and_imu_ignore();
    test_timeout_and_resync();
    test_legacy_reserved_axes_are_ignored();
    puts("usb_frame_protocol_test: PASS");
    return 0;
}
