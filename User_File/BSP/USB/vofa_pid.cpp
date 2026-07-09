#include "vofa_pid.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "debug_uart.h"
#include "motor_task.h"

#define VOFA_JUSTFLOAT_TAIL_0 0x00U
#define VOFA_JUSTFLOAT_TAIL_1 0x00U
#define VOFA_JUSTFLOAT_TAIL_2 0x80U
#define VOFA_JUSTFLOAT_TAIL_3 0x7FU
#define VOFA_RX_LINE_MAX        96U
#define VOFA_CHANNEL_COUNT      5U

static const uint8_t s_vofa_tail[4U] = {
    VOFA_JUSTFLOAT_TAIL_0,
    VOFA_JUSTFLOAT_TAIL_1,
    VOFA_JUSTFLOAT_TAIL_2,
    VOFA_JUSTFLOAT_TAIL_3,
};

static uint8_t s_vofa_enabled = 0U;
static uint8_t s_vofa_motor_index = 0U;
static char s_vofa_rx_line[VOFA_RX_LINE_MAX];
static uint16_t s_vofa_rx_len = 0U;

static void vofa_trim_line(char *line)
{
    if (line == nullptr) {
        return;
    }

    char *start = line;
    while ((*start != '\0') && isspace((unsigned char)*start)) {
        ++start;
    }

    if (start != line) {
        memmove(line, start, strlen(start) + 1U);
    }

    size_t len = strlen(line);
    while ((len > 0U) && isspace((unsigned char)line[len - 1U])) {
        line[len - 1U] = '\0';
        --len;
    }

    if ((len > 0U) && (line[0U] == '#')) {
        memmove(line, &line[1U], len);
    }
}

static Dog_Mit_Ang_Pid *vofa_active_ang_pid(void)
{
    return (Dog_Mit_Ang_Pid *)dog_mit_active_ang_pid();
}

static void vofa_apply_ang_param(Dog_Mit_Ang_Pid *pid, const char *field, float value)
{
    if (pid == nullptr) {
        return;
    }

    if (strcmp(field, "Kp") == 0) {
        if (value < 0.0f) {
            value = 0.0f;
        }
        pid->kp_a_per_deg = value;
        dog_mit_reset_integrators();
        return;
    }

    if (strcmp(field, "Ki") == 0) {
        if (value < 0.0f) {
            value = 0.0f;
        }
        pid->ki_a_per_deg_s = value;
        dog_mit_reset_integrators();
        return;
    }

    if (strcmp(field, "Kd") == 0) {
        if (value < 0.0f) {
            value = 0.0f;
        }
        pid->kd_a_per_dps = value;
        dog_mit_reset_integrators();
        return;
    }

    if ((strcmp(field, "Ol") == 0) || (strcmp(field, "Sl") == 0) || (strcmp(field, "Wl") == 0)) {
        if (value < 0.1f) {
            value = 0.1f;
        }
        pid->output_limit_a = value;
        return;
    }
}

static void vofa_apply_param(const char *name, float value)
{
    if ((name == nullptr) || (!isfinite(value))) {
        return;
    }

    if ((strncmp(name, "Skp", 3) == 0) || (strncmp(name, "skp", 3) == 0)) {
        vofa_apply_ang_param(&g_dog_mit_stand_pid, "Kp", value);
        return;
    }
    if ((strncmp(name, "Ski", 3) == 0) || (strncmp(name, "ski", 3) == 0)) {
        vofa_apply_ang_param(&g_dog_mit_stand_pid, "Ki", value);
        return;
    }
    if ((strncmp(name, "Skd", 3) == 0) || (strncmp(name, "skd", 3) == 0)) {
        vofa_apply_ang_param(&g_dog_mit_stand_pid, "Kd", value);
        return;
    }
    if ((strncmp(name, "Sl", 2) == 0) || (strncmp(name, "sl", 2) == 0)) {
        vofa_apply_ang_param(&g_dog_mit_stand_pid, "Sl", value);
        return;
    }

    if ((strncmp(name, "Wkp", 3) == 0) || (strncmp(name, "wkp", 3) == 0)) {
        vofa_apply_ang_param(&g_dog_mit_swing_pid, "Kp", value);
        return;
    }
    if ((strncmp(name, "Wki", 3) == 0) || (strncmp(name, "wki", 3) == 0)) {
        vofa_apply_ang_param(&g_dog_mit_swing_pid, "Ki", value);
        return;
    }
    if ((strncmp(name, "Wkd", 3) == 0) || (strncmp(name, "wkd", 3) == 0)) {
        vofa_apply_ang_param(&g_dog_mit_swing_pid, "Kd", value);
        return;
    }
    if ((strncmp(name, "Wl", 2) == 0) || (strncmp(name, "wl", 2) == 0)) {
        vofa_apply_ang_param(&g_dog_mit_swing_pid, "Wl", value);
        return;
    }

    if ((strcmp(name, "Kp") == 0) || (strcmp(name, "kp") == 0)) {
        vofa_apply_ang_param(vofa_active_ang_pid(), "Kp", value);
        return;
    }
    if ((strcmp(name, "Ki") == 0) || (strcmp(name, "ki") == 0)) {
        vofa_apply_ang_param(vofa_active_ang_pid(), "Ki", value);
        return;
    }
    if ((strcmp(name, "Kd") == 0) || (strcmp(name, "kd") == 0)) {
        vofa_apply_ang_param(vofa_active_ang_pid(), "Kd", value);
        return;
    }
    if ((strcmp(name, "Ol") == 0) || (strcmp(name, "ol") == 0)) {
        vofa_apply_ang_param(vofa_active_ang_pid(), "Ol", value);
        return;
    }

    if ((strcmp(name, "Il") == 0) || (strcmp(name, "il") == 0) ||
        (strcmp(name, "Ilim") == 0) || (strcmp(name, "ilim") == 0)) {
        if (value < 0.1f) {
            value = 0.1f;
        }
        g_dog_mit_motor_limits.current_limit_a = value;
        return;
    }

    if ((strcmp(name, "Motor") == 0) || (strcmp(name, "motor") == 0) ||
        (strcmp(name, "M") == 0) || (strcmp(name, "m") == 0)) {
        int motor = (int)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
        if ((motor >= 0) && (motor < (int)DOG_MOTOR_COUNT)) {
            s_vofa_motor_index = (uint8_t)motor;
        }
    }
}

static void vofa_parse_line(char *line)
{
    if (line == nullptr) {
        return;
    }

    vofa_trim_line(line);
    if (line[0U] == '\0') {
        return;
    }

    char *colon = strchr(line, ':');
    if (colon == nullptr) {
        return;
    }

    *colon = '\0';
    const char *name = line;
    const char *value_text = colon + 1;
    float value = strtof(value_text, nullptr);
    vofa_apply_param(name, value);
}

void VofaPid_Init(void)
{
    s_vofa_enabled = 0U;
    s_vofa_motor_index = 0U;
    s_vofa_rx_len = 0U;
    memset(s_vofa_rx_line, 0, sizeof(s_vofa_rx_line));
}

void VofaPid_SetEnabled(uint8_t enable)
{
    s_vofa_enabled = (enable != 0U) ? 1U : 0U;
    s_vofa_rx_len = 0U;
    s_vofa_rx_line[0U] = '\0';
}

uint8_t VofaPid_IsEnabled(void)
{
    return s_vofa_enabled;
}

void VofaPid_SetMotorIndex(uint8_t motor_index)
{
    if (motor_index < DOG_MOTOR_COUNT) {
        s_vofa_motor_index = motor_index;
    }
}

uint8_t VofaPid_GetMotorIndex(void)
{
    return s_vofa_motor_index;
}

void VofaPid_CycleMotorIndex(void)
{
    s_vofa_motor_index = (uint8_t)((s_vofa_motor_index + 1U) % DOG_MOTOR_COUNT);
}

void VofaPid_SendTelemetry(const Dog_Mit_Pid_Telemetry *telemetry)
{
    if ((s_vofa_enabled == 0U) || (telemetry == nullptr)) {
        return;
    }

    float channels[VOFA_CHANNEL_COUNT] = {
        telemetry->target_deg,
        telemetry->user_deg,
        telemetry->err_deg,
        telemetry->cmd_a,
        telemetry->p_a,
    };

    uint8_t frame[(VOFA_CHANNEL_COUNT * sizeof(float)) + sizeof(s_vofa_tail)];
    memcpy(frame, channels, sizeof(channels));
    memcpy(&frame[sizeof(channels)], s_vofa_tail, sizeof(s_vofa_tail));
    DebugUart_WriteRaw(frame, (uint16_t)sizeof(frame));
}

uint8_t VofaPid_FeedRxByte(uint8_t byte)
{
    if (s_vofa_enabled == 0U) {
        return 0U;
    }

    if (s_vofa_rx_len > 0U) {
        if ((byte == '\r') || (byte == '\n')) {
            s_vofa_rx_line[s_vofa_rx_len] = '\0';
            vofa_parse_line(s_vofa_rx_line);
            s_vofa_rx_len = 0U;
            return 1U;
        }

        if (s_vofa_rx_len >= (VOFA_RX_LINE_MAX - 1U)) {
            s_vofa_rx_len = 0U;
            return 1U;
        }

        s_vofa_rx_line[s_vofa_rx_len++] = (char)byte;
        s_vofa_rx_line[s_vofa_rx_len] = '\0';
        return 1U;
    }

    if ((byte == '#') || (byte == 'K') || (byte == 'k') || (byte == 'I') || (byte == 'i') ||
        (byte == 'D') || (byte == 'd') || (byte == 'O') || (byte == 'o') ||
        (byte == 'S') || (byte == 's') || (byte == 'W') || (byte == 'w') ||
        (byte == 'M') || (byte == 'm')) {
        s_vofa_rx_line[0U] = (char)byte;
        s_vofa_rx_len = 1U;
        s_vofa_rx_line[1U] = '\0';
        return 1U;
    }

    return 0U;
}
