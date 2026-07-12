#ifndef MOTOR_DM_H
#define MOTOR_DM_H

#include "bsp_fdcan.h"
#include "struct_typedef.h"

#include <stdint.h>

enum Enum_Motor_DM_Status {
    Motor_DM_Status_DISABLE = 0,
    Motor_DM_Status_ENABLE,
};

enum Enum_Motor_DM_Mode {
    Motor_DM_Pos_control = 0,
    Motor_DM_Angle_control,
    Motor_DM_T_control,
    Motor_DM_W_control,
};

enum motor_DM_Model {
    MOTOR_DM_J10010L = 0,
    MOTOR_DM_J4310 = 1,
};

struct Struct_recv_motor_DM {
    uint8_t MError;
    uint8_t mode;
    fp32 Now_Pos;
    fp32 Now_Angle;
    fp32 Now_W;
    fp32 Now_T;
    fp32 motor_Temperature;
    fp32 mos_Temperature;
};

struct Struct_send_motor_DM {
    fp32 Angle;
    fp32 Pos;
    fp32 T;
    fp32 W;
    fp32 Kp;
    fp32 Kd;
    fp32 T_ff;
};

struct motor_DM_control {
    Struct_send_motor_DM send;
    Struct_recv_motor_DM recv;
};

class Class_Motor_DM {
public:
    void Init(FDCAN_HandleTypeDef *hfdcan,
              uint16_t id,
              const motor_DM_Model &model,
              const Enum_Motor_DM_Mode &control_mode);

    void enable();
    void lose();
    uint8_t probe_disable();
    void zero();
    void can_send();
    void can_send_torque_only(fp32 torque_nm, uint8_t invert);
    void can_recv(uint8_t *data);

    uint16_t Get_CAN_ID() const { return CAN_id; }
    FDCAN_HandleTypeDef *Get_CAN() const { return FDcan; }
    Enum_Motor_DM_Status Get_Status() const { return Motor_DM_Status; }
    void Set_Status(const Enum_Motor_DM_Status &status) { Motor_DM_Status = status; }
    void Set_Control_Mode(const Enum_Motor_DM_Mode &mode) { control_mode = mode; }

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
    fp32 Get_motor_Temperature() const { return recv.motor_Temperature; }
    fp32 Get_mos_Temperature() const { return recv.mos_Temperature; }
    uint8_t Get_MError() const { return recv.MError; }
    uint8_t Get_mode() const { return recv.mode; }
    fp32 Get_T_Max() const { return T_Max; }
    fp32 Get_Kt() const { return Kt; }

private:
    uint16_t CAN_id = 0U;
    motor_DM_Model model = MOTOR_DM_J4310;
    FDCAN_HandleTypeDef *FDcan = nullptr;
    fp32 Pos_Max = 12.57f;
    fp32 W_Max = 30.0f;
    fp32 T_Max = 30.0f;
    fp32 Kp_Max = 500.0f;
    fp32 Kd_Max = 5.0f;
    fp32 Kt = 1.2f;
    uint32_t torque_send_count = 0U;
    Enum_Motor_DM_Status Motor_DM_Status = Motor_DM_Status_DISABLE;
    Struct_recv_motor_DM recv = {};
    Enum_Motor_DM_Mode control_mode = Motor_DM_Pos_control;
    fp32 Angle = 0.0f;
    fp32 Pos = 0.0f;
    fp32 T = 0.0f;
    fp32 W = 0.0f;
    fp32 Kp = 0.0f;
    fp32 Kd = 0.0f;
};

#endif
