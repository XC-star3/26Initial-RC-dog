#include "motor_DM.h"

static constexpr fp32 kPi = 3.14159265358979323846f;
static constexpr fp32 kDegToRad = kPi / 180.0f;
static constexpr fp32 kRadToDeg = 180.0f / kPi;

static fp32 clamp_fp32(fp32 value, fp32 min_value, fp32 max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static uint16_t float_to_uint(fp32 value, fp32 min_value, fp32 max_value, uint8_t bits)
{
    value = clamp_fp32(value, min_value, max_value);
    const fp32 span = max_value - min_value;
    const uint32_t max_int = (1UL << bits) - 1UL;
    return (uint16_t)((value - min_value) * (fp32)max_int / span);
}

static fp32 uint_to_float(uint16_t value, fp32 min_value, fp32 max_value, uint8_t bits)
{
    const fp32 span = max_value - min_value;
    const uint32_t max_int = (1UL << bits) - 1UL;
    return ((fp32)value * span / (fp32)max_int) + min_value;
}

void Class_Motor_DM::Init(FDCAN_HandleTypeDef *hfdcan,
                          uint16_t id,
                          const motor_DM_Model &motor_model,
                          const Enum_Motor_DM_Mode &motor_control_mode)
{
    FDcan = hfdcan;
    CAN_id = id;
    model = motor_model;
    control_mode = motor_control_mode;

    switch (model) {
    case MOTOR_DM_J10010L:
        Pos_Max = 12.57f;
        W_Max = 50.0f;
        T_Max = 100.0f;
        Kp_Max = 500.0f;
        Kd_Max = 5.0f;
        Kt = 1.2f;
        break;
    case MOTOR_DM_J4310:
    default:
        Pos_Max = kPi;
        W_Max = 30.0f;
        T_Max = 10.0f;
        Kp_Max = 500.0f;
        Kd_Max = 5.0f;
        Kt = 1.2f;
        break;
    }
    torque_send_count = 0U;
}

void Class_Motor_DM::enable()
{
    if (FDcan == nullptr) {
        return;
    }

    uint8_t tx[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU};
    (void)fdcan_send_data_stand(FDcan, CAN_id, tx, 8U);
    Motor_DM_Status = Motor_DM_Status_ENABLE;
    torque_send_count = 0U;
}

void Class_Motor_DM::lose()
{
    (void)probe_disable();
}

uint8_t Class_Motor_DM::probe_disable()
{
    Motor_DM_Status = Motor_DM_Status_DISABLE;
    if (FDcan == nullptr) {
        return 0U;
    }

    uint8_t tx[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFDU};
    return (fdcan_send_data_stand(FDcan, CAN_id, tx, 8U) == 0U) ? 1U : 0U;
}

void Class_Motor_DM::zero()
{
    if (FDcan == nullptr) {
        return;
    }

    uint8_t tx[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFEU};
    (void)fdcan_send_data_stand(FDcan, CAN_id, tx, 8U);
}

void Class_Motor_DM::can_send()
{
    if ((FDcan == nullptr) || (Motor_DM_Status != Motor_DM_Status_ENABLE)) {
        return;
    }

    if (control_mode == Motor_DM_Angle_control) {
        Pos = Angle * kDegToRad;
    }

    const uint16_t pos = float_to_uint(Pos, -Pos_Max, Pos_Max, 16U);
    const uint16_t vel = float_to_uint(W, -W_Max, W_Max, 12U);
    const uint16_t tor = float_to_uint(T, -T_Max, T_Max, 12U);
    const uint16_t kp = float_to_uint(Kp, 0.0f, Kp_Max, 12U);
    const uint16_t kd = float_to_uint(Kd, 0.0f, Kd_Max, 12U);

    uint8_t tx[8] = {};
    tx[0] = (uint8_t)(pos >> 8);
    tx[1] = (uint8_t)pos;
    tx[2] = (uint8_t)(vel >> 4);
    tx[3] = (uint8_t)(((vel & 0x0FU) << 4) | ((kp >> 8) & 0x0FU));
    tx[4] = (uint8_t)kp;
    tx[5] = (uint8_t)(kd >> 4);
    tx[6] = (uint8_t)(((kd & 0x0FU) << 4) | ((tor >> 8) & 0x0FU));
    tx[7] = (uint8_t)tor;

    (void)fdcan_send_data_stand(FDcan, CAN_id, tx, 8U);
}

void Class_Motor_DM::can_send_torque_only(fp32 torque_nm, uint8_t invert)
{
    if (FDcan == nullptr) {
        return;
    }

    if (Motor_DM_Status != Motor_DM_Status_ENABLE) {
        lose();
        return;
    }

    torque_send_count++;
    if ((torque_send_count % 100U) == 0U) {
        enable();
        return;
    }

    const fp32 command_torque = (invert != 0U) ? -torque_nm : torque_nm;
    const uint16_t pos = float_to_uint(0.0f, -Pos_Max, Pos_Max, 16U);
    const uint16_t vel = float_to_uint(0.0f, -W_Max, W_Max, 12U);
    const uint16_t tor = float_to_uint(command_torque, -T_Max, T_Max, 12U);
    const uint16_t kp = float_to_uint(0.0f, 0.0f, Kp_Max, 12U);
    const uint16_t kd = float_to_uint(0.0f, 0.0f, Kd_Max, 12U);

    uint8_t tx[8] = {};
    tx[0] = (uint8_t)(pos >> 8);
    tx[1] = (uint8_t)pos;
    tx[2] = (uint8_t)(vel >> 4);
    tx[3] = (uint8_t)(((vel & 0x0FU) << 4) | ((kp >> 8) & 0x0FU));
    tx[4] = (uint8_t)kp;
    tx[5] = (uint8_t)(kd >> 4);
    tx[6] = (uint8_t)(((kd & 0x0FU) << 4) | ((tor >> 8) & 0x0FU));
    tx[7] = (uint8_t)tor;

    (void)fdcan_send_data_stand(FDcan, CAN_id, tx, 8U);
}

void Class_Motor_DM::can_recv(uint8_t *data)
{
    if (data == nullptr) {
        return;
    }

    recv.MError = (uint8_t)(data[0] >> 4);
    recv.mode = (uint8_t)(data[0] & 0x0FU);
    recv.Now_Pos = uint_to_float((uint16_t)((data[1] << 8) | data[2]), -Pos_Max, Pos_Max, 16U);
    recv.Now_W = uint_to_float((uint16_t)((data[3] << 4) | (data[4] >> 4)), -W_Max, W_Max, 12U);
    recv.Now_T = uint_to_float((uint16_t)(((data[4] & 0x0FU) << 8) | data[5]), -T_Max, T_Max, 12U);
    recv.mos_Temperature = (fp32)((int8_t)data[6]);
    recv.motor_Temperature = (fp32)((int8_t)data[7]);
    recv.Now_Angle = recv.Now_Pos * kRadToDeg;
    Motor_DM_Status = Motor_DM_Status_ENABLE;
}
