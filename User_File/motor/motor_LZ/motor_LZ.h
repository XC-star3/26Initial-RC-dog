#ifndef MOTOR_LZ_H
#define MOTOR_LZ_H

#include "bsp_fdcan.h"
#include "struct_typedef.h"

#include <stdint.h>

#define MOTOR_LZ_DEFAULT_MASTER_ID 0xFDU
#define MOTOR_LZ_EL05_RUN_MODE_MIT 0U

enum Enum_Motor_LZ_Status {
    Motor_LZ_Status_DISABLE = 0,
    Motor_LZ_Status_ENABLE,
};

enum Enum_Motor_LZ_Mode {
    Motor_LZ_Pos_control = 0,
    Motor_LZ_Angle_control,
    Motor_LZ_T_control,
    Motor_LZ_W_control,
};

enum motor_LZ_Model {
    MOTOR_LZ_00 = 0,
    MOTOR_LZ_01,
    MOTOR_LZ_02,
    MOTOR_LZ_03,
    MOTOR_LZ_04,
    MOTOR_LZ_05,
    MOTOR_LZ_EL05 = MOTOR_LZ_05,
};

enum canComMode {
    CANCOM_ANNOUNCE_DEVID = 0,
    CANCOM_MOTOR_CTRL,
    CANCOM_MOTOR_FEEDBACK,
    CANCOM_MOTOR_IN,
    CANCOM_MOTOR_RESET,
    CANCOM_MOTOR_STOP = CANCOM_MOTOR_RESET,
    CANCOM_MOTOR_CALI,
    CANCOM_MOTOR_ZERO,
    CANCOM_MOTOR_ID,
    CANCOM_PARA_WRITE,
    CANCOM_PARA_READ,
    CANCOM_CALI_ING,
    CANCOM_CALI_RST,
    CANCOM_PARA_STR_INFO,
    CANCOM_MOTOR_BRAKE,
    CANCOM_FAULT_WARN,
    CANCOM_MODE_TOTAL,
    CANCOM_MODE_BD,
    CANCOM_EL05_WRITE_PARAM = 0x12,
    CANCOM_MODE_ACTIVE_RECV = 0x18,
    CANCOM_MODE_AGREEMENT,
};

struct Struct_recv_motor_Lz {
    uint8_t MError;
    uint8_t mode;
    fp32 Now_Pos;
    fp32 Now_Angle;
    fp32 Now_W;
    fp32 Now_T;
    fp32 Now_Temperature;
};

struct Struct_send_motor_Lz {
    fp32 Angle;
    fp32 Pos;
    fp32 T;
    fp32 W;
    fp32 Kp;
    fp32 Kd;
    fp32 T_ff;
};

struct motor_lz_control {
    Struct_send_motor_Lz send;
    Struct_recv_motor_Lz recv;
};

class Class_Motor_LZ {
public:
    void Init(FDCAN_HandleTypeDef *hfdcan,
              uint16_t id,
              const motor_LZ_Model &model,
              const Enum_Motor_LZ_Mode &control_mode);

    void enable();
    void lose();
    uint8_t probe_device_id();
    void active_recv(uint8_t enable);
    void zero();
    void motor_set_CAN_ID(uint8_t set_id);
    void set_master_id(uint8_t master_id);
    void set_run_mode(uint8_t run_mode);
    void can_send();
    void can_recv(uint32_t ext_id, uint8_t *data);

    uint16_t Get_CAN_ID() const { return CAN_id; }
    uint8_t Get_Master_ID() const { return Master_id; }
    FDCAN_HandleTypeDef *Get_CAN() const { return FDcan; }
    Enum_Motor_LZ_Status Get_Status() const { return Motor_LZ_Status; }
    void Set_Status(const Enum_Motor_LZ_Status &status) { Motor_LZ_Status = status; }
    void Set_Control_Mode(const Enum_Motor_LZ_Mode &mode) { control_mode = mode; }

    void Set_Pos(const fp32 &pos) { Pos = pos; }
    fp32 Get_Pos() const { return Pos; }
    void Set_Angle(const fp32 &angle) { Angle = angle; }
    fp32 Get_Angle() const { return Angle; }
    void Set_W(const fp32 &w) { W = w; }
    fp32 Get_W() const { return W; }
    void Set_T(const fp32 &t) { T = t; }
    fp32 Get_T() const { return T; }
    void Set_Kd(const fp32 &kd) { Kd = kd; }
    fp32 Get_Kd() const { return Kd; }
    void Set_Kp(const fp32 &kp) { Kp = kp; }
    fp32 Get_Kp() const { return Kp; }

    fp32 Get_Now_Angle() const { return recv.Now_Angle; }
    fp32 Get_Now_Pos() const { return recv.Now_Pos; }
    fp32 Get_Now_W() const { return recv.Now_W; }
    fp32 Get_Now_T() const { return recv.Now_T; }
    fp32 Get_Now_Temperature() const { return recv.Now_Temperature; }
    uint8_t Get_MError() const { return recv.MError; }
    uint8_t Get_mode() const { return recv.mode; }

private:
    uint32_t make_ext_id(uint32_t id_data) const;

    uint16_t CAN_id = 0U;
    uint8_t Master_id = MOTOR_LZ_DEFAULT_MASTER_ID;
    motor_LZ_Model model = MOTOR_LZ_EL05;
    FDCAN_HandleTypeDef *FDcan = nullptr;
    fp32 Pos_Max = 12.57f;
    fp32 W_Max = 44.0f;
    fp32 T_Max = 17.0f;
    Enum_Motor_LZ_Status Motor_LZ_Status = Motor_LZ_Status_DISABLE;
    Struct_recv_motor_Lz recv = {};
    canComMode mode = CANCOM_MOTOR_FEEDBACK;
    Enum_Motor_LZ_Mode control_mode = Motor_LZ_Pos_control;
    fp32 Angle = 0.0f;
    fp32 Pos = 0.0f;
    fp32 T = 0.0f;
    fp32 W = 0.0f;
    fp32 Kp = 0.0f;
    fp32 Kd = 0.0f;
};

#endif
