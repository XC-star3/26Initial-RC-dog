#include "MWMotor.h"

/* 所有总线挂的所有电机访问信息 */
MW_MOTOR_ACCESS_INFO motors[MAX_BUS_NUM][MAX_MOTOR_NUM_PER_BUS];

uint8_t MWRegisterMotor(MW_MOTOR_ACCESS_INFO motor) {
    if(motor.busId >= MAX_BUS_NUM || motor.nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return MW_ERROR_OUT_OF_RANGE;
    motors[motor.busId][motor.nodeId] = motor;
    return MW_ERROR_SUCCESS;
}

void MWEstop(uint8_t busId, uint8_t nodeId) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    sender(busId, ((uint16_t)nodeId << 5) | MW_ESTOP_CMD, txBuff, sizeof(txBuff));
}

uint32_t MWGetMotorError(uint8_t busId, uint8_t nodeId, MW_ERROR_TYPE errorType) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return 0;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return 0;

    uint8_t txBuff[8] = { 0 };
    txBuff[0] = errorType;
    sender(busId, ((uint16_t)nodeId << 5) | MW_GET_ERROR_CMD, txBuff, sizeof(txBuff));
    
    uint32_t error = motors[busId][nodeId].motorData->error;
    return error;
}

void MWRxTxSdo(uint8_t busId, uint8_t nodeId, MW_ENDPOINT endpointData) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    txBuff[0] = endpointData.opcode;
    *(uint16_t *)&txBuff[1] = endpointData.id;
    memcpy(&txBuff[4], &endpointData.data, 4);
    motors[busId][nodeId].motorData->EndpointData.type = endpointData.type;
    sender(busId, ((uint16_t)nodeId << 5) | ((endpointData.opcode == MW_ENDPOINT_OPCODE_READ) ? MW_RXSDO_CMD : MW_TXSDO_CMD), txBuff, 8);
}

void MWSetAxisNodeID(uint8_t busId, uint8_t nodeId, uint8_t newNodeId, MW_MOTOR_ACCESS_INFO *motorInfo) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    txBuff[0] = newNodeId;
    sender(busId, ((uint16_t)nodeId << 5) | MW_SET_AXIS_NODE_ID_CMD, txBuff, sizeof(txBuff));
    memcpy(&motors[busId][newNodeId], motorInfo, sizeof(motors[busId][newNodeId]));
}

void MWSetAxisState(uint8_t busId, uint8_t nodeId, MW_MOTER_STATE state) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint32_t stateValue = (uint32_t)state;
    uint8_t txBuff[8] = { 0 };
    memcpy(txBuff, &stateValue, sizeof(stateValue));
    sender(busId, ((uint16_t)nodeId << 5) | MW_SET_AXIS_STATE_CMD, txBuff, sizeof(txBuff));
}

MW_MIT_CTRL MWMitControl(uint8_t busId, uint8_t nodeId, MW_MIT_CTRL *mit) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return (MW_MIT_CTRL){ -1 };
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return (MW_MIT_CTRL){ -1 };

    uint8_t txBuff[8] = { 0 };
    int16_t pos_int = (int16_t)((mit->pos + 12.5f) * 65535.0f / 25.0f);
    int16_t vel_int = (int16_t)((mit->vel + 65.0f) * 4095.0f / 130.0f);
    int16_t kp_int = (int16_t)(mit->kp * 4095.0f / 500.0f);
    int16_t kd_int = (int16_t)(mit->kd * 4095.0f / 5.0f);
    int16_t t_int = (int16_t)((mit->torque + 50.0f) * 4095.0f / 100.0f);
    txBuff[0] = pos_int >> 8;
    txBuff[1] = pos_int & 0xFF;
    txBuff[2] = vel_int >> 4;
    txBuff[3] = ((vel_int & 0xF) << 4) + (kp_int >> 8);
    txBuff[4] = kp_int & 0xFF;
    txBuff[5] = kd_int >> 4;
    txBuff[6] = ((kd_int & 0xF) << 4) + (t_int >> 8);
    txBuff[7] = t_int & 0xFF;
    sender(busId, ((uint16_t)nodeId << 5) | MW_MIT_CONTROL_CMD, txBuff, sizeof(txBuff));
    
    return motors[busId][nodeId].motorData->motorMIT;
}

MW_ENCODER_ESTIMATES MWGetEncoderEstimates(uint8_t busId, uint8_t nodeId) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS)
        return (MW_ENCODER_ESTIMATES){ -1 };
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return (MW_ENCODER_ESTIMATES){ -1 }; 

    uint8_t txBuff[8] = { 0 };
    sender(busId, ((uint16_t)nodeId << 5) | MW_GET_ENCODER_ESTIMATES_CMD, txBuff, sizeof(txBuff));
    
    return motors[busId][nodeId].motorData->encoderEstimates;
}

MW_ENCODER_COUNT MWGetEncoderCount(uint8_t busId, uint8_t nodeId) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS)
        return (MW_ENCODER_COUNT) { -1 };
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return (MW_ENCODER_COUNT) { -1 };

    uint8_t txBuff[8] = { 0 };
    sender(busId, ((uint16_t)nodeId << 5) | MW_GET_ENCODER_COUNT_CMD, (uint8_t*)txBuff, sizeof(txBuff));
    
    return motors[busId][nodeId].motorData->encoderCount;
}

void MWSetControllerMode(uint8_t busId, uint8_t nodeId, MW_CONTROL_MODE ctrlMode, MW_INPUT_MODE inputMode) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint32_t ctrlModeValue = (uint32_t)ctrlMode;
    uint32_t inputModeValue = (uint32_t)inputMode;
    uint8_t txBuff[8] = { 0 };
    memcpy(&txBuff[0], &ctrlModeValue, sizeof(ctrlModeValue));
    memcpy(&txBuff[4], &inputModeValue, sizeof(inputModeValue));
    sender(busId, ((uint16_t)nodeId << 5) | MW_SET_CONTROLLER_MODE_CMD, (uint8_t*)txBuff, sizeof(txBuff));
}

void MWPosControl(uint8_t busId, uint8_t nodeId, float inputPos, int16_t velFF, int16_t torqueFF) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    memcpy(&txBuff[0], (uint8_t*)&inputPos, 4);
    memcpy(&txBuff[4], (uint8_t*)&velFF, 2);
    memcpy(&txBuff[6], (uint8_t*)&torqueFF, 2);
    sender(busId, ((uint16_t)nodeId << 5) | MW_SET_INPUT_POS_CMD, (uint8_t*)txBuff, sizeof(txBuff));
}

void MWVelControl(uint8_t busId, uint8_t nodeId, float inputVel, float torqueFF) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    memcpy(&txBuff[0], (uint8_t*)&inputVel, 4);
    memcpy(&txBuff[4], (uint8_t*)&torqueFF, 4);
    sender(busId, ((uint16_t)nodeId << 5) | MW_SET_INPUT_VEL_CMD, (uint8_t*)txBuff, sizeof(txBuff)); 
}

void MWTorqueControl(uint8_t busId, uint8_t nodeId, float inputTorque) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    memcpy(&txBuff[0], (uint8_t*)&inputTorque, 4);
    sender(busId, ((uint16_t)nodeId << 5) | MW_SET_INPUT_TORQUE_CMD, (uint8_t*)txBuff, sizeof(txBuff)); 
}

void MWSetLimits(uint8_t busId, uint8_t nodeId, float velLim, float currLim) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    memcpy(&txBuff[0], (uint8_t*)&velLim, 4);
    memcpy(&txBuff[4], (uint8_t*)&currLim, 4);
    sender(busId, ((uint16_t)nodeId << 5) | MW_SET_LIMITS_CMD, (uint8_t*)txBuff, sizeof(txBuff));  
}

void MWStartAnticogging(uint8_t busId, uint8_t nodeId) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    sender(busId, ((uint16_t)nodeId << 5) | MW_START_ANTICOGGING_CMD, (uint8_t*)txBuff, sizeof(txBuff)); 
}

void MWSetTrajVelLimit(uint8_t busId, uint8_t nodeId, float velLim) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    memcpy(&txBuff[0], (uint8_t*)&velLim, 4);
    sender(busId, ((uint16_t)nodeId << 5) | MW_SET_TRAJ_VEL_LIMIT_CMD, (uint8_t*)txBuff, sizeof(txBuff));
}

void MWSetTrajAccelLimits(uint8_t busId, uint8_t nodeId, float accLim, float decLim) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    memcpy(&txBuff[0], (uint8_t*)&accLim, 4);
    memcpy(&txBuff[4], (uint8_t*)&decLim, 4);
    sender(busId, ((uint16_t)nodeId << 5) | MW_SET_TRAJ_ACCEL_LIMIT_CMD, (uint8_t*)txBuff, sizeof(txBuff));  
}

void MWSetTrajInertia(uint8_t busId, uint8_t nodeId, float inertia) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    memcpy(&txBuff[0], (uint8_t*)&inertia, 4);
    sender(busId, ((uint16_t)nodeId << 5) | MW_SET_TRAJ_INERTIA_CMD, (uint8_t*)txBuff, sizeof(txBuff));    
}

MW_IQ MWGetIq(uint8_t busId, uint8_t nodeId) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return (MW_IQ){ -1 };
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return (MW_IQ){ -1 };

    uint8_t txBuff[8] = { 0 };
    sender(busId, ((uint16_t)nodeId << 5) | MW_GET_IQ_CMD, (uint8_t*)txBuff, sizeof(txBuff));
    
    return motors[busId][nodeId].motorData->iq;
}

void MWReboot(uint8_t busId, uint8_t nodeId) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    sender(busId, ((uint16_t)nodeId << 5) | MW_REBOOT_CMD, (uint8_t*)txBuff, sizeof(txBuff));
}

MW_BUS_VOLTAGE_CURRENT MWGetBusVoltageCurrent(uint8_t busId, uint8_t nodeId) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return (MW_BUS_VOLTAGE_CURRENT){ -1 };
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return (MW_BUS_VOLTAGE_CURRENT){ -1 };

    uint8_t txBuff[8] = { 0 };
    sender(busId, ((uint16_t)nodeId << 5) | MW_GET_BUS_VOLTAGE_CURRENT_CMD, (uint8_t*)txBuff, sizeof(txBuff));
    
    return motors[busId][nodeId].motorData->busVoltCurrent;
}

void MWClearErrors(uint8_t busId, uint8_t nodeId) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    sender(busId, ((uint16_t)nodeId << 5) | MW_CLEAR_ERRORS_CMD, (uint8_t*)txBuff, sizeof(txBuff));
}

void MWSetLinearCount(uint8_t busId, uint8_t nodeId, int32_t linearCount) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    memcpy(&txBuff[0], (uint8_t*)&linearCount, 4);
    sender(busId, ((uint16_t)nodeId << 5) | MW_SET_LINEAR_COUNT_CMD, (uint8_t*)txBuff, sizeof(txBuff));
}

void MWSetPosGain(uint8_t busId, uint8_t nodeId, float posGain) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    memcpy(&txBuff[0], (uint8_t*)&posGain, 4);
    sender(busId, ((uint16_t)nodeId << 5) | MW_SET_POS_GAIN_CMD, (uint8_t*)txBuff, sizeof(txBuff));
}

void MWSetVelGain(uint8_t busId, uint8_t nodeId, float velGain, float velIntGain) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    memcpy(&txBuff[0], (uint8_t*)&velGain, 4);
    memcpy(&txBuff[4], (uint8_t*)&velIntGain, 4);
    sender(busId, ((uint16_t)nodeId << 5) | MW_SET_VEL_GAIN_CMD, (uint8_t*)txBuff, sizeof(txBuff));
}

MW_TORQUES MWGetTorques(uint8_t busId, uint8_t nodeId) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return (MW_TORQUES){ -1 };
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return (MW_TORQUES){ -1 };

    uint8_t txBuff[8] = { 0 };
    sender(busId, ((uint16_t)nodeId << 5) | MW_GET_TORQUES_CMD, (uint8_t*)txBuff, sizeof(txBuff));
    
    return motors[busId][nodeId].motorData->torques;
}

MW_POWERS MWGetPowers(uint8_t busId, uint8_t nodeId) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return (MW_POWERS){ -1 };
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return (MW_POWERS){ -1 };

    uint8_t txBuff[8] = { 0 };
    sender(busId, ((uint16_t)nodeId << 5) | MW_GET_POWERS_CMD, (uint8_t*)txBuff, sizeof(txBuff));
    
    return motors[busId][nodeId].motorData->powers;
}

void MWDisableCAN(uint8_t busId, uint8_t nodeId) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    sender(busId, ((uint16_t)nodeId << 5) | MW_DISABLE_CAN_CMD, (uint8_t*)txBuff, sizeof(txBuff));
}

void MWSaveConfigeration(uint8_t busId, uint8_t nodeId) {
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) 
        return;
    MotorSender sender = motors[busId][nodeId].sender;
    if (sender == 0) return;

    uint8_t txBuff[8] = { 0 };
    sender(busId, ((uint16_t)nodeId << 5) | MW_SAVE_CONFIGURATION_CMD, (uint8_t*)txBuff, sizeof(txBuff));
}

/**
 * @brief 电机心跳数据获取
 * 
 * @param motorData 指向MW_MOTOR_DATA结构体的指针，用于存储电机的相关数据
 * @param data CAN接收数据
 * @note cmd_id: 0x001
 */
void MWHeartbeatRec(MW_MOTOR_DATA *motorData, uint8_t *data) {
    memcpy(&(motorData->heartBeat.ErrorStatus.axisError), &data[0], sizeof(motorData->heartBeat.ErrorStatus.axisError));
    motorData->heartBeat.currentState = data[4];
    if (FIRMWARE_VERSION_NUMBER <= 511) {
        motorData->heartBeat.ErrorStatus.motorErrorFlag = data[5];
        motorData->heartBeat.ErrorStatus.encoderErrorFlag = data[6];
        motorData->heartBeat.ErrorStatus.controllerErrorFlag = data[7] & 0x1;
        motorData->heartBeat.trajectoryDoneFlag = (data[7] & 0x8) >> 3;
    }
    else if (FIRMWARE_VERSION_NUMBER == 512) {
        motorData->heartBeat.ErrorStatus.motorErrorFlag = data[5] & 0x1;
        motorData->heartBeat.ErrorStatus.encoderErrorFlag = (data[5] & 0x2) >> 1;
        motorData->heartBeat.ErrorStatus.controllerErrorFlag = (data[5] & 0x4) >> 2;
        motorData->heartBeat.trajectoryDoneFlag = (data[5] & 0x80) >> 7;        
    }
    else if (FIRMWARE_VERSION_NUMBER >= 513) {
        motorData->heartBeat.ErrorStatus.motorErrorFlag = data[5] & 0x1;
        motorData->heartBeat.ErrorStatus.encoderErrorFlag = (data[5] & 0x2) >> 1;
        motorData->heartBeat.ErrorStatus.controllerErrorFlag = (data[5] & 0x4) >> 2;
        motorData->heartBeat.ErrorStatus.ErrorFlag = (data[5] & 0x8) >> 3;
        motorData->heartBeat.trajectoryDoneFlag = (data[5] & 0x80) >> 7;    
        motorData->heartBeat.life = data[7];        
    }
}

/**
 * @brief 电机异常数据接收
 * 
 * @param motorData 指向MW_MOTOR_DATA结构体的指针，用于存储电机的相关数据
 * @param data CAN接收数据
 * @note cmd_id: 0x003
 */
void MWGetMotorErrorRec(MW_MOTOR_DATA *motorData, uint8_t *data) {
    memcpy(&(motorData->error), &data[0], sizeof(motorData->error));
}

/**
 * @brief 电机Endpoint_ID数据接收
 * 
 * @param motorData 指向MW_MOTOR_DATA结构体的指针，用于存储电机的相关数据
 * @param data CAN接收数据
 * @note cmd_id: 0x004
 */
void MWRxSdoRec(MW_MOTOR_DATA *motorData, uint8_t *data) {
    motorData->EndpointData.opcode = ((data[0] == 0) ? MW_ENDPOINT_OPCODE_READ : MW_ENDPOINT_OPCODE_WRITE);
    motorData->EndpointData.id = *(uint16_t *)&data[1];
    memcpy(&motorData->EndpointData.data, &data[4], 4);
}

/**
 * @brief 电机MIT控制模式接收
 * 
 * @param motorData 指向MW_MOTOR_DATA结构体的指针，用于存储电机的相关数据
 * @param data CAN接收数据
 * @note cmd_id: 0x008
 */
uint8_t MWMitControlRec(MW_MOTOR_DATA *motorData, uint8_t *data) {
    motorData->motorMIT.pos = ((float)(data[1] << 8 | data[2]) * 25.0f / 65535) - 12.5f;
    motorData->motorMIT.vel = ((float)(data[3] << 4 | data[4] >> 4) * 130.0f / 4095.0f) - 65.0f;
    motorData->motorMIT.torque = ((float)(((data[4] & 0xF) << 8) | data[5]) * 100.0f / 4095.0f) - 50.0f;
    return data[0];
}

/**
 * @brief 电机编码器位置速度数据接收
 * 
 * @param motorData 指向MW_MOTOR_DATA结构体的指针，用于存储电机的相关数据
 * @param data CAN接收数据
 * @note cmd_id: 0x009
 */
void MWGetEncoderEstimatesRec(MW_MOTOR_DATA *motorData, uint8_t *data) {
    memcpy(&(motorData->encoderEstimates.encoderPosEstimate), &data[0], sizeof(motorData->encoderEstimates.encoderPosEstimate));
    memcpy(&(motorData->encoderEstimates.encoderVelEstimate), &data[4], sizeof(motorData->encoderEstimates.encoderVelEstimate));
}

/**
 * @brief 电机编码器CPR数据接收
 * 
 * @param motorData 指向MW_MOTOR_DATA结构体的指针，用于存储电机的相关数据
 * @param data CAN接收数据
 * @note cmd_id: 0x00A
 */
void MWGetEncoderCountRec(MW_MOTOR_DATA *motorData, uint8_t *data) {
    memcpy(&(motorData->encoderCount.shadowCount), &data[0], sizeof(motorData->encoderCount.shadowCount));
    memcpy(&(motorData->encoderCount.countInCPR), &data[4], sizeof(motorData->encoderCount.countInCPR));
}

/**
 * @brief 电机电流数据接收
 * 
 * @param motorData 指向MW_MOTOR_DATA结构体的指针，用于存储电机的相关数据
 * @param data CAN接收数据
 * @note cmd_id: 0x014
 */
void MWGetIqRec(MW_MOTOR_DATA *motorData, uint8_t *data) {
    memcpy(&(motorData->iq.iqSetpoint), &data[0], sizeof(motorData->iq.iqSetpoint));
    memcpy(&(motorData->iq.iqMeasured), &data[4], sizeof(motorData->iq.iqMeasured));
}

/**
 * @brief 电机电源电压电源电流数据接收
 * 
 * @param motorData 指向MW_MOTOR_DATA结构体的指针，用于存储电机的相关数据
 * @param data CAN接收数据
 * @note cmd_id: 0x017
 */
void MWGetBusVoltageCurrentRec(MW_MOTOR_DATA *motorData, uint8_t *data) {
    memcpy(&(motorData->busVoltCurrent.busVoltage), &data[0], sizeof(motorData->busVoltCurrent.busVoltage));
    memcpy(&(motorData->busVoltCurrent.busCurrent), &data[4], sizeof(motorData->busVoltCurrent.busCurrent));    
}

/**
 * @brief 电机力矩数据接收
 * 
 * @param motorData 指向MW_MOTOR_DATA结构体的指针，用于存储电机的相关数据
 * @param data CAN接收数据
 * @note cmd_id: 0x01C
 */
void MWGetTorquesRec(MW_MOTOR_DATA *motorData, uint8_t *data) {
    memcpy(&(motorData->torques.torqueSetpoint), &data[0], sizeof(motorData->torques.torqueSetpoint));
    memcpy(&(motorData->torques.torque), &data[4], sizeof(motorData->torques.torque));        
}

/**
 * @brief 电机功率和机械功率数据接收
 *
 * @param motorData 指向MW_MOTOR_DATA结构体的指针，用于存储电机的相关数据
 * @param data CAN接收数据
 * @note cmd_id: 0x01D
 */
void MWGetPowersRec(MW_MOTOR_DATA *motorData, uint8_t *data) {
    memcpy(&(motorData->powers.electricalPower), &data[0], sizeof(motorData->powers.electricalPower));
    memcpy(&(motorData->powers.mechanicalPower), &data[4], sizeof(motorData->powers.mechanicalPower));     
}

void MWReceiver(uint8_t busId, uint32_t canId, uint8_t *data) {
    uint8_t nodeId = canId >> 5;
    if (busId >= MAX_BUS_NUM || nodeId >= MAX_MOTOR_NUM_PER_BUS) return;
    MW_MOTOR_ACCESS_INFO *motor = &(motors[busId][nodeId]);
    if (motor->motorData == 0) return;

    MW_CMD_ID cmdId = (MW_CMD_ID)(canId & 0x1F);
    switch (cmdId) {
        case MW_HEARTBEAT_CMD:
            MWHeartbeatRec(motor->motorData, data);
            break;
        case MW_GET_ERROR_CMD:
            MWGetMotorErrorRec(motor->motorData, data);
            break;
        case MW_RXSDO_CMD:
            MWRxSdoRec(motor->motorData, data);
            break;
        case MW_MIT_CONTROL_CMD:
            MWMitControlRec(motor->motorData, data);
            break;
        case MW_GET_ENCODER_ESTIMATES_CMD:
            MWGetEncoderEstimatesRec(motor->motorData, data);
            break;
        case MW_GET_ENCODER_COUNT_CMD:
            MWGetEncoderCountRec(motor->motorData, data);
            break;
        case MW_GET_TORQUES_CMD:
            MWGetTorquesRec(motor->motorData, data);
            break;
        case MW_GET_IQ_CMD:
            MWGetIqRec(motor->motorData, data);
            break;
        case MW_GET_BUS_VOLTAGE_CURRENT_CMD:
            MWGetBusVoltageCurrentRec(motor->motorData, data);
            break;
        case MW_GET_POWERS_CMD:
            MWGetPowersRec(motor->motorData, data);
            break;
        default:
            break;
    }
    if (motor->notifier != 0) {
        motor->notifier(busId, nodeId, cmdId);
    }
} 
