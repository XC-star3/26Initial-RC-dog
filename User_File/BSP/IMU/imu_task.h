#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef IMU_BODY_X_FROM
#define IMU_BODY_X_FROM 0
#endif
#ifndef IMU_BODY_Y_FROM
#define IMU_BODY_Y_FROM 1
#endif
#ifndef IMU_BODY_Z_FROM
#define IMU_BODY_Z_FROM 2
#endif
#ifndef IMU_BODY_X_SIGN
#define IMU_BODY_X_SIGN 1.0f
#endif
#ifndef IMU_BODY_Y_SIGN
#define IMU_BODY_Y_SIGN 1.0f
#endif
#ifndef IMU_BODY_Z_SIGN
#define IMU_BODY_Z_SIGN 1.0f
#endif

#define IMU_TASK_PERIOD_MS      2U
#define IMU_CALIBRATE_MS        1000U
#define IMU_CALIBRATE_SAMPLES   (IMU_CALIBRATE_MS / IMU_TASK_PERIOD_MS)
#define IMU_LOCAL_GRAVITY_MPS2  9.7948f

#ifndef IMU_Calibration_ENABLE
#define IMU_Calibration_ENABLE  0U
#endif

typedef struct {
    uint8_t initialized;
    uint8_t valid;
    uint8_t calibrated;
    uint8_t init_error;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gyro_dps[3];
    float accel_mps2[3];
    float gyro_bias_dps[3];
    float temp_c;
    float z_comp_mm[4];
    uint32_t update_count;
    uint32_t tick_ms;
    uint32_t last_ok_ms;
} ImuTask_Status;

typedef struct {
    float Temperature;
    float Heat_Target;
    uint16_t Heat_Compare;
    float Offsets_Gyro_X;
    float Offsets_Gyro_Y;
    float Offsets_Gyro_Z;
    uint8_t Offsets_Init;
    uint32_t Sample_Count;
    float Gyro_Dps[3];
} ImuCalibrationWatch_T;

extern ImuCalibrationWatch_T ImuCalibrationWatch;

void ImuTask_Init(void);
void ImuTask_Tick(void);
uint8_t ImuTask_GetStatus(ImuTask_Status *status);
void ImuTask_PrintStatus(void);

#ifdef __cplusplus
}
#endif

#endif
