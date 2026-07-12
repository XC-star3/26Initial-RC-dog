#include "motor_LZ.h"

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

void Class_Motor_LZ::Init(FDCAN_HandleTypeDef *hfdcan,
                          uint16_t id,
                          const motor_LZ_Model &motor_model,
                          const Enum_Motor_LZ_Mode &motor_control_mode)
{
    FDcan = hfdcan;
    CAN_id = id;
    model = motor_model;
    control_mode = motor_control_mode;

    Master_id = MOTOR_LZ_DEFAULT_MASTER_ID;

    switch (model) {
    case MOTOR_LZ_EL05:
        Pos_Max = 12.57f;
        W_Max = 50.0f;
        T_Max = 6.0f;
        break;
    case MOTOR_LZ_02:
    default:
        Pos_Max = 12.57f;
        W_Max = 44.0f;
        T_Max = 17.0f;
        break;
    }
}

uint32_t Class_Motor_LZ::make_ext_id(uint32_t id_data) const
{
    return (((uint32_t)mode) << 24) | ((id_data & 0xFFFFU) << 8) | (CAN_id & 0xFFU);
}

void Class_Motor_LZ::enable()
{
    if (FDcan == nullptr) {
        return;
    }

    uint8_t tx[8] = {};
    mode = CANCOM_MOTOR_IN;
    Motor_LZ_Status = Motor_LZ_Status_ENABLE;
    (void)fdcan_send_data_Exten(FDcan, make_ext_id(Master_id), tx, 8U);
}

void Class_Motor_LZ::lose()
{
    if (FDcan == nullptr) {
        return;
    }

    uint8_t tx[8] = {};
    mode = CANCOM_MOTOR_STOP;
    Motor_LZ_Status = Motor_LZ_Status_DISABLE;
    (void)fdcan_send_data_Exten(FDcan, make_ext_id(Master_id), tx, 8U);
}

uint8_t Class_Motor_LZ::probe_device_id()
{
    if (FDcan == nullptr) {
        return 0U;
    }

    uint8_t tx[8] = {};
    const uint32_t ext_id = ((uint32_t)Master_id << 8) | (CAN_id & 0xFFU);
    return (fdcan_send_data_Exten(FDcan, ext_id, tx, 8U) == 0U) ? 1U : 0U;
}

void Class_Motor_LZ::active_recv(uint8_t enable)
{
    if (FDcan == nullptr) {
        return;
    }

    uint8_t tx[8] = {1U, 2U, 3U, 4U, 5U, 6U, 0U, 0U};
    tx[6] = (enable != 0U) ? 1U : 0U;
    mode = CANCOM_MODE_ACTIVE_RECV;
    (void)fdcan_send_data_Exten(FDcan, make_ext_id(Master_id), tx, 8U);
}

void Class_Motor_LZ::zero()
{
    if (FDcan == nullptr) {
        return;
    }

    uint8_t tx[8] = {1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    lose();
    mode = CANCOM_MOTOR_ZERO;
    (void)fdcan_send_data_Exten(FDcan, make_ext_id(Master_id), tx, 8U);
    enable();
}

void Class_Motor_LZ::motor_set_CAN_ID(uint8_t set_id)
{
    if (FDcan == nullptr) {
        return;
    }

    uint8_t tx[8] = {1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    lose();
    mode = CANCOM_MOTOR_ID;
    (void)fdcan_send_data_Exten(FDcan, make_ext_id(((uint32_t)Master_id << 8) | set_id), tx, 8U);
    CAN_id = set_id;
}

void Class_Motor_LZ::set_master_id(uint8_t master_id)
{
    Master_id = master_id;
}

void Class_Motor_LZ::set_run_mode(uint8_t run_mode)
{
    if (FDcan == nullptr) {
        return;
    }

    uint8_t tx[8] = {};
    tx[0] = 0x05U;
    tx[1] = 0x70U;
    tx[4] = run_mode;
    mode = CANCOM_EL05_WRITE_PARAM;
    (void)fdcan_send_data_Exten(FDcan, make_ext_id(Master_id), tx, 8U);
}

void Class_Motor_LZ::can_recv(uint32_t ext_id, uint8_t *data)
{
    if (data == nullptr) {
        return;
    }

    recv.Now_Pos = uint_to_float((uint16_t)((data[0] << 8) | data[1]), -Pos_Max, Pos_Max, 16U);
    recv.Now_Angle = recv.Now_Pos * kRadToDeg;
    recv.Now_W = uint_to_float((uint16_t)((data[2] << 8) | data[3]), -W_Max, W_Max, 16U);
    recv.Now_T = uint_to_float((uint16_t)((data[4] << 8) | data[5]), -T_Max, T_Max, 16U);
    recv.MError = (uint8_t)((ext_id >> 16) & 0x3FU);
    recv.mode = (uint8_t)((ext_id >> 22) & 0x03U);
    recv.Now_Temperature = (fp32)((data[6] << 8) | data[7]) * 0.1f;
    Motor_LZ_Status = Motor_LZ_Status_ENABLE;
}

void Class_Motor_LZ::can_send()
{
    if ((FDcan == nullptr) || (Motor_LZ_Status != Motor_LZ_Status_ENABLE)) {
        return;
    }

    mode = CANCOM_MOTOR_CTRL;
    if (control_mode == Motor_LZ_Angle_control) {
        Pos = Angle * kDegToRad;
    }

    Pos = clamp_fp32(Pos, -Pos_Max, Pos_Max);
    W = clamp_fp32(W, -W_Max, W_Max);
    T = clamp_fp32(T, -T_Max, T_Max);
    Kp = clamp_fp32(Kp, 0.0f, 500.0f);
    Kd = clamp_fp32(Kd, 0.0f, 5.0f);

    const uint16_t pos = float_to_uint(Pos, -Pos_Max, Pos_Max, 16U);
    const uint16_t vel = float_to_uint(W, -W_Max, W_Max, 16U);
    const uint16_t tor = float_to_uint(T, -T_Max, T_Max, 16U);
    const uint16_t kp = float_to_uint(Kp, 0.0f, 500.0f, 16U);
    const uint16_t kd = float_to_uint(Kd, 0.0f, 5.0f, 16U);

    uint8_t tx[8] = {};
    tx[0] = (uint8_t)(pos >> 8);
    tx[1] = (uint8_t)pos;
    tx[2] = (uint8_t)(vel >> 8);
    tx[3] = (uint8_t)vel;
    tx[4] = (uint8_t)(kp >> 8);
    tx[5] = (uint8_t)kp;
    tx[6] = (uint8_t)(kd >> 8);
    tx[7] = (uint8_t)kd;

    (void)fdcan_send_data_Exten(FDcan, make_ext_id(tor), tx, 8U);
}
