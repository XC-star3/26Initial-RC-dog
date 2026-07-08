#include "imu_task.h"

#include "debug_uart.h"
#include "motor_task.h"
#include "tim.h"

extern "C" {
#include "BMI088driver.h"
}

#include "stm32h7xx_hal.h"

#include <math.h>
#include <string.h>

#define IMU_RAD_TO_DEG         57.29577951308232f
#define IMU_COMP_ALPHA         0.98f
#define IMU_INIT_RETRY_MS      1000U
#define IMU_HEAT_TARGET_C      40.0f
#define IMU_HEAT_MAX_COMPARE   2000U
#define IMU_HEAT_KP            600.0f
#define IMU_HEAT_KI            2.0f

extern "C" {
ImuCalibrationWatch_T ImuCalibrationWatch = {};
}

static ImuTask_Status s_imu_status = {};
static float s_gyro_bias_rad_s[3] = {};
static float s_gyro_bias_accum_rad_s[3] = {};
static uint16_t s_calib_count = 0U;
static uint32_t s_last_tick_ms = 0U;
static uint32_t s_last_init_try_ms = 0U;

#if IMU_Calibration_ENABLE
static float s_heat_integral = 0.0f;
#endif

#if !IMU_Calibration_ENABLE
static const float s_fixed_gyro_bias_rad_s[3] = {
    0.0f,
    0.0f,
    0.0f,
};
#endif

static uint8_t imu_axis_index(uint8_t index)
{
    return (index < 3U) ? index : 0U;
}

static float imu_map_axis(const float raw[3], uint8_t index, float sign)
{
    return raw[imu_axis_index(index)] * sign;
}

static void imu_map_raw(const float raw[3], float body[3])
{
    body[0] = imu_map_axis(raw, IMU_BODY_X_FROM, IMU_BODY_X_SIGN);
    body[1] = imu_map_axis(raw, IMU_BODY_Y_FROM, IMU_BODY_Y_SIGN);
    body[2] = imu_map_axis(raw, IMU_BODY_Z_FROM, IMU_BODY_Z_SIGN);
}

static void imu_accel_angles(const float accel_mps2[3], float *roll_deg, float *pitch_deg)
{
    const float ax = accel_mps2[0];
    const float ay = accel_mps2[1];
    const float az = accel_mps2[2];
    if (roll_deg != nullptr) {
        *roll_deg = atan2f(ay, az) * IMU_RAD_TO_DEG;
    }
    if (pitch_deg != nullptr) {
        *pitch_deg = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * IMU_RAD_TO_DEG;
    }
}

static void imu_set_watch_offsets(const float bias_rad_s[3])
{
    ImuCalibrationWatch.Offsets_Gyro_X = bias_rad_s[0];
    ImuCalibrationWatch.Offsets_Gyro_Y = bias_rad_s[1];
    ImuCalibrationWatch.Offsets_Gyro_Z = bias_rad_s[2];
}

static void imu_set_watch_gyro_dps(const float gyro_rad_s[3])
{
    for (uint8_t i = 0U; i < 3U; ++i) {
        ImuCalibrationWatch.Gyro_Dps[i] = gyro_rad_s[i] * IMU_RAD_TO_DEG;
    }
}

static void imu_apply_gyro_bias(const float bias_rad_s[3])
{
    for (uint8_t i = 0U; i < 3U; ++i) {
        s_gyro_bias_rad_s[i] = bias_rad_s[i];
        s_imu_status.gyro_bias_dps[i] = s_gyro_bias_rad_s[i] * IMU_RAD_TO_DEG;
    }
    imu_set_watch_offsets(s_gyro_bias_rad_s);
}

static void imu_init_attitude_from_accel(const float accel_body_g[3])
{
    for (uint8_t i = 0U; i < 3U; ++i) {
        s_imu_status.accel_mps2[i] = accel_body_g[i] * IMU_LOCAL_GRAVITY_MPS2;
    }
    imu_accel_angles(s_imu_status.accel_mps2, &s_imu_status.roll_deg, &s_imu_status.pitch_deg);
    s_imu_status.yaw_deg = 0.0f;
}

#if IMU_Calibration_ENABLE
static void imu_heat_set_compare(uint16_t compare)
{
    if (compare > IMU_HEAT_MAX_COMPARE) {
        compare = IMU_HEAT_MAX_COMPARE;
    }
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, compare);
    ImuCalibrationWatch.Heat_Compare = compare;
}

static void imu_heat_init(void)
{
    s_heat_integral = 0.0f;
    ImuCalibrationWatch.Heat_Target = IMU_HEAT_TARGET_C;
    ImuCalibrationWatch.Heat_Compare = 0U;
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    imu_heat_set_compare(0U);
}

static void imu_heat_control(float temp_c)
{
    const float error = IMU_HEAT_TARGET_C - temp_c;
    uint16_t compare = 0U;

    if (error > 0.0f) {
        s_heat_integral += error;
        if (s_heat_integral > (float)IMU_HEAT_MAX_COMPARE) {
            s_heat_integral = (float)IMU_HEAT_MAX_COMPARE;
        }

        float output = (IMU_HEAT_KP * error) + (IMU_HEAT_KI * s_heat_integral);
        if (output > (float)IMU_HEAT_MAX_COMPARE) {
            output = (float)IMU_HEAT_MAX_COMPARE;
        }
        compare = (uint16_t)output;
    } else {
        s_heat_integral = 0.0f;
    }

    imu_heat_set_compare(compare);
}
#endif

static void imu_publish(uint8_t valid)
{
    Dog_Imu_Sample sample = {};
    sample.valid = valid;
    sample.calibrated = s_imu_status.calibrated;
    sample.init_error = s_imu_status.init_error;
    sample.roll_deg = s_imu_status.roll_deg;
    sample.pitch_deg = s_imu_status.pitch_deg;
    sample.yaw_deg = s_imu_status.yaw_deg;
    sample.temp_c = s_imu_status.temp_c;
    sample.tick_ms = s_imu_status.tick_ms;
    memcpy(sample.gyro_dps, s_imu_status.gyro_dps, sizeof(sample.gyro_dps));
    memcpy(sample.accel_mps2, s_imu_status.accel_mps2, sizeof(sample.accel_mps2));
    DogImu_Update(&sample);
}

void ImuTask_Init(void)
{
    memset(&s_imu_status, 0, sizeof(s_imu_status));
    memset(s_gyro_bias_rad_s, 0, sizeof(s_gyro_bias_rad_s));
    memset(s_gyro_bias_accum_rad_s, 0, sizeof(s_gyro_bias_accum_rad_s));
    memset(&ImuCalibrationWatch, 0, sizeof(ImuCalibrationWatch));
    s_calib_count = 0U;
    s_last_tick_ms = HAL_GetTick();
    s_last_init_try_ms = s_last_tick_ms;
    ImuCalibrationWatch.Heat_Target = IMU_HEAT_TARGET_C;
    ImuCalibrationWatch.Heat_Compare = 0U;

#if !IMU_Calibration_ENABLE
    imu_apply_gyro_bias(s_fixed_gyro_bias_rad_s);
    ImuCalibrationWatch.Offsets_Init = 1U;
    ImuCalibrationWatch.Sample_Count = IMU_CALIBRATE_SAMPLES;
    s_calib_count = IMU_CALIBRATE_SAMPLES;
#else
    imu_heat_init();
#endif

    s_imu_status.init_error = BMI088_init();
    s_imu_status.initialized = (s_imu_status.init_error == BMI088_NO_ERROR) ? 1U : 0U;
    s_imu_status.tick_ms = s_last_tick_ms;
    imu_publish(0U);
}

void ImuTask_Tick(void)
{
    const uint32_t now = HAL_GetTick();
    if (s_imu_status.initialized == 0U) {
        if ((uint32_t)(now - s_last_init_try_ms) >= IMU_INIT_RETRY_MS) {
            s_last_init_try_ms = now;
            s_imu_status.init_error = BMI088_init();
            s_imu_status.initialized = (s_imu_status.init_error == BMI088_NO_ERROR) ? 1U : 0U;
        }
        s_imu_status.tick_ms = now;
        imu_publish(0U);
        return;
    }

    float gyro_raw_rad_s[3] = {};
    float accel_raw_g[3] = {};
    float temp_c = 0.0f;
    BMI088_read(gyro_raw_rad_s, accel_raw_g, &temp_c);

    float gyro_body_rad_s[3] = {};
    float accel_body_g[3] = {};
    imu_map_raw(gyro_raw_rad_s, gyro_body_rad_s);
    imu_map_raw(accel_raw_g, accel_body_g);
    ImuCalibrationWatch.Temperature = temp_c;
    imu_set_watch_gyro_dps(gyro_body_rad_s);

#if IMU_Calibration_ENABLE
    imu_heat_control(temp_c);

    if (s_calib_count < IMU_CALIBRATE_SAMPLES) {
        for (uint8_t i = 0U; i < 3U; ++i) {
            s_gyro_bias_accum_rad_s[i] += gyro_body_rad_s[i];
        }
        s_calib_count++;
        s_imu_status.tick_ms = now;
        s_imu_status.temp_c = temp_c;
        ImuCalibrationWatch.Sample_Count = s_calib_count;

        const float running_inv = 1.0f / (float)s_calib_count;
        float running_bias_rad_s[3] = {};
        for (uint8_t i = 0U; i < 3U; ++i) {
            running_bias_rad_s[i] = s_gyro_bias_accum_rad_s[i] * running_inv;
        }
        imu_set_watch_offsets(running_bias_rad_s);

        if (s_calib_count >= IMU_CALIBRATE_SAMPLES) {
            const float inv = 1.0f / (float)IMU_CALIBRATE_SAMPLES;
            float calibrated_bias_rad_s[3] = {};
            for (uint8_t i = 0U; i < 3U; ++i) {
                calibrated_bias_rad_s[i] = s_gyro_bias_accum_rad_s[i] * inv;
            }
            imu_apply_gyro_bias(calibrated_bias_rad_s);
            imu_init_attitude_from_accel(accel_body_g);
            s_imu_status.calibrated = 1U;
            ImuCalibrationWatch.Offsets_Init = 1U;
            s_last_tick_ms = now;
        }
        imu_publish(0U);
        return;
    }
#else
    if (s_imu_status.calibrated == 0U) {
        imu_apply_gyro_bias(s_fixed_gyro_bias_rad_s);
        imu_init_attitude_from_accel(accel_body_g);
        s_imu_status.calibrated = 1U;
        ImuCalibrationWatch.Offsets_Init = 1U;
        ImuCalibrationWatch.Sample_Count = IMU_CALIBRATE_SAMPLES;
        s_last_tick_ms = now;
    }
#endif

    float dt_s = (float)(now - s_last_tick_ms) * 0.001f;
    if ((dt_s <= 0.0f) || (dt_s > 0.1f)) {
        dt_s = (float)IMU_TASK_PERIOD_MS * 0.001f;
    }
    s_last_tick_ms = now;

    for (uint8_t i = 0U; i < 3U; ++i) {
        const float gyro_rad_s = gyro_body_rad_s[i] - s_gyro_bias_rad_s[i];
        s_imu_status.gyro_dps[i] = gyro_rad_s * IMU_RAD_TO_DEG;
        s_imu_status.accel_mps2[i] = accel_body_g[i] * IMU_LOCAL_GRAVITY_MPS2;
    }
    for (uint8_t i = 0U; i < 3U; ++i) {
        ImuCalibrationWatch.Gyro_Dps[i] = s_imu_status.gyro_dps[i];
    }

    float accel_roll = 0.0f;
    float accel_pitch = 0.0f;
    imu_accel_angles(s_imu_status.accel_mps2, &accel_roll, &accel_pitch);

    s_imu_status.roll_deg = (IMU_COMP_ALPHA * (s_imu_status.roll_deg + (s_imu_status.gyro_dps[0] * dt_s))) +
                            ((1.0f - IMU_COMP_ALPHA) * accel_roll);
    s_imu_status.pitch_deg = (IMU_COMP_ALPHA * (s_imu_status.pitch_deg + (s_imu_status.gyro_dps[1] * dt_s))) +
                             ((1.0f - IMU_COMP_ALPHA) * accel_pitch);
    s_imu_status.yaw_deg += s_imu_status.gyro_dps[2] * dt_s;
    if (s_imu_status.yaw_deg > 180.0f) {
        s_imu_status.yaw_deg -= 360.0f;
    } else if (s_imu_status.yaw_deg < -180.0f) {
        s_imu_status.yaw_deg += 360.0f;
    }

    s_imu_status.temp_c = temp_c;
    s_imu_status.valid = 1U;
    s_imu_status.tick_ms = now;
    s_imu_status.last_ok_ms = now;
    s_imu_status.update_count++;
    dog_imu_balance_get_leg_z_offsets(s_imu_status.z_comp_mm);
    imu_publish(1U);
}

uint8_t ImuTask_GetStatus(ImuTask_Status *status)
{
    if (status == nullptr) {
        return 0U;
    }
    *status = s_imu_status;
    dog_imu_balance_get_leg_z_offsets(status->z_comp_mm);
    return 1U;
}

void ImuTask_PrintStatus(void)
{
    ImuTask_Status st = {};
    (void)ImuTask_GetStatus(&st);
    const uint8_t balance_enabled = dog_imu_balance_is_enabled();
    const uint8_t balance_active = dog_imu_balance_is_active();
    const uint32_t now = HAL_GetTick();
    const uint32_t age = (st.tick_ms == 0U) ? 0xFFFFFFFFU : (uint32_t)(now - st.tick_ms);

    DebugUart_Printf("IMU BMI088 init=0x%02X initialized=%u valid=%u calibrated=%u age=%lums updates=%lu balance=%u active=%u\r\n",
                     (unsigned)st.init_error,
                     (unsigned)st.initialized,
                     (unsigned)st.valid,
                     (unsigned)st.calibrated,
                     (unsigned long)age,
                     (unsigned long)st.update_count,
                     (unsigned)balance_enabled,
                     (unsigned)balance_active);
    DebugUart_Printf("  rpy=(%ld.%02ld,%ld.%02ld,%ld.%02ld)deg temp=%ld.%01ldC\r\n",
                     (long)st.roll_deg, (long)(fabsf(st.roll_deg * 100.0f)) % 100L,
                     (long)st.pitch_deg, (long)(fabsf(st.pitch_deg * 100.0f)) % 100L,
                     (long)st.yaw_deg, (long)(fabsf(st.yaw_deg * 100.0f)) % 100L,
                     (long)st.temp_c, (long)(fabsf(st.temp_c * 10.0f)) % 10L);
    DebugUart_Printf("  gyro_dps=(%ld,%ld,%ld) accel_mps2=(%ld,%ld,%ld) zcomp LF/RF/LB/RB=(%ld,%ld,%ld,%ld)mm\r\n",
                     (long)st.gyro_dps[0],
                     (long)st.gyro_dps[1],
                     (long)st.gyro_dps[2],
                     (long)st.accel_mps2[0],
                     (long)st.accel_mps2[1],
                     (long)st.accel_mps2[2],
                     (long)st.z_comp_mm[0],
                     (long)st.z_comp_mm[1],
                     (long)st.z_comp_mm[2],
                     (long)st.z_comp_mm[3]);
}
