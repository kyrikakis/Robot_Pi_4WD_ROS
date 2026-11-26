// Motors: MCPWM H-bridge drive, PCNT quadrature encoders, per-wheel speed PID.
//
// Kept equal to the stock firmware so the ROS side (velocity watchdog gains, EKF)
// sees the same robot: 25 kHz PWM with a 400-tick period, a 200-tick dead zone
// added to any non-zero output, an incremental PID on encoder counts per 10 ms,
// and wheel speed = v -/+ w * half_track.
//
// Differences: a wheel whose target is zero brakes and its PID resets (the stock
// PID hunted around zero through the dead zone), and the motors stop when
// cmd_vel is silent for CONFIG_RAINBOW_CMD_TIMEOUT_MS.

#include <math.h>
#include <string.h>

#include "bdc_motor.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "rainbow.h"

#define PWM_RESOLUTION_HZ 10000000
#define PWM_FREQ_HZ 25000
#define PWM_PERIOD_TICKS (PWM_RESOLUTION_HZ / PWM_FREQ_HZ)  // 400
#define PWM_DEAD_ZONE 200
#define PID_OUTPUT_MAX (PWM_PERIOD_TICKS - PWM_DEAD_ZONE)  // 200
#define PID_PERIOD_MS 10
#define PCNT_LIMIT 1000

typedef struct {
    int pwm_a;     // GPIO driven for forward
    int pwm_b;
    int group;     // MCPWM group (3 timers each: M1-M3 in 0, M4 in 1)
    int enc_a;
    int enc_b;
} wheel_pins_t;

// M1 and M2 (left side) are wired mirrored: their B pin drives forward.
static const wheel_pins_t PINS[RAINBOW_NUM_WHEELS] = {
    {.pwm_a = 5, .pwm_b = 4, .group = 0, .enc_a = 6, .enc_b = 7},
    {.pwm_a = 16, .pwm_b = 15, .group = 0, .enc_a = 47, .enc_b = 48},
    {.pwm_a = 9, .pwm_b = 10, .group = 0, .enc_a = 11, .enc_b = 12},
    {.pwm_a = 13, .pwm_b = 14, .group = 1, .enc_a = 1, .enc_b = 2},
};

typedef struct {
    float e1, e2, out;
} pid_state_t;

static bdc_motor_handle_t s_motor[RAINBOW_NUM_WHEELS];
static pcnt_unit_handle_t s_pcnt[RAINBOW_NUM_WHEELS];
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static float s_target_counts[RAINBOW_NUM_WHEELS];  // counts per PID period
static int64_t s_last_command_us;

static float s_kp, s_ki, s_kd;
static float s_m_per_count;
static float s_half_track;
static float s_max_speed;

float motors_metres_per_count(void) { return s_m_per_count; }
float motors_half_track(void) { return s_half_track; }

static void pcnt_init_one(int i)
{
    pcnt_unit_config_t unit_cfg = {
        .high_limit = PCNT_LIMIT,
        .low_limit = -PCNT_LIMIT,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_pcnt[i]));
    pcnt_glitch_filter_config_t filter = {.max_glitch_ns = 1000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_pcnt[i], &filter));

    pcnt_chan_config_t a_cfg = {.edge_gpio_num = PINS[i].enc_a, .level_gpio_num = PINS[i].enc_b};
    pcnt_chan_config_t b_cfg = {.edge_gpio_num = PINS[i].enc_b, .level_gpio_num = PINS[i].enc_a};
    pcnt_channel_handle_t a, b;
    ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt[i], &a_cfg, &a));
    ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt[i], &b_cfg, &b));
    // Full quadrature (x4).
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_pcnt[i], PCNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_pcnt[i], -PCNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(s_pcnt[i]));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_pcnt[i]));
    ESP_ERROR_CHECK(pcnt_unit_start(s_pcnt[i]));
}

static void motor_init_one(int i)
{
    bdc_motor_config_t cfg = {
        .pwm_freq_hz = PWM_FREQ_HZ,
        .pwma_gpio_num = PINS[i].pwm_a,
        .pwmb_gpio_num = PINS[i].pwm_b,
    };
    bdc_motor_mcpwm_config_t mcpwm_cfg = {
        .group_id = PINS[i].group,
        .resolution_hz = PWM_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&cfg, &mcpwm_cfg, &s_motor[i]));
    ESP_ERROR_CHECK(bdc_motor_enable(s_motor[i]));
    ESP_ERROR_CHECK(bdc_motor_brake(s_motor[i]));
}

// output in PWM ticks before the dead zone, signed.
static void motor_drive(int i, int output)
{
    if (output == 0) {
        bdc_motor_brake(s_motor[i]);
        return;
    }
    int ticks = abs(output) + PWM_DEAD_ZONE;
    if (ticks > PWM_PERIOD_TICKS) ticks = PWM_PERIOD_TICKS;
    if (output > 0) {
        bdc_motor_forward(s_motor[i]);
    } else {
        bdc_motor_reverse(s_motor[i]);
    }
    bdc_motor_set_speed(s_motor[i], (uint32_t)ticks);
}

// Incremental PID, as Espressif's pid_ctrl (PID_CAL_TYPE_INCREMENTAL) used by the stock firmware.
static float pid_step(pid_state_t *p, float error)
{
    p->out += s_kp * (error - p->e1) + s_ki * error + s_kd * (error - 2.0f * p->e1 + p->e2);
    if (p->out > PID_OUTPUT_MAX) p->out = PID_OUTPUT_MAX;
    if (p->out < -PID_OUTPUT_MAX) p->out = -PID_OUTPUT_MAX;
    p->e2 = p->e1;
    p->e1 = error;
    return p->out;
}

static int read_count(int i)
{
    int c = 0;
    pcnt_unit_get_count(s_pcnt[i], &c);
    return c;
}

static void motor_task(void *arg)
{
    pid_state_t pid[RAINBOW_NUM_WHEELS] = {0};
    int last[RAINBOW_NUM_WHEELS];
    for (int i = 0; i < RAINBOW_NUM_WHEELS; i++) last[i] = read_count(i);

    TickType_t wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PID_PERIOD_MS));

        float target[RAINBOW_NUM_WHEELS];
        int64_t now = esp_timer_get_time();
        taskENTER_CRITICAL(&s_lock);
        if (now - s_last_command_us > (int64_t)CONFIG_RAINBOW_CMD_TIMEOUT_MS * 1000) {
            memset(s_target_counts, 0, sizeof(s_target_counts));
        }
        memcpy(target, s_target_counts, sizeof(target));
        taskEXIT_CRITICAL(&s_lock);

        for (int i = 0; i < RAINBOW_NUM_WHEELS; i++) {
            int c = read_count(i);
            float measured = (float)(c - last[i]);
            last[i] = c;
            if (target[i] == 0.0f) {
                memset(&pid[i], 0, sizeof(pid[i]));
                motor_drive(i, 0);
            } else {
                motor_drive(i, (int)pid_step(&pid[i], target[i] - measured));
            }
        }
    }
}

static float clamp_speed(float v)
{
    if (v > s_max_speed) return s_max_speed;
    if (v < -s_max_speed) return -s_max_speed;
    return v;
}

void motors_command(float v, float w)
{
    float left = clamp_speed(v - w * s_half_track);
    float right = clamp_speed(v + w * s_half_track);
    float per_period = s_m_per_count * (1000.0f / PID_PERIOD_MS);  // m/s per count/period
    taskENTER_CRITICAL(&s_lock);
    s_target_counts[0] = s_target_counts[1] = left / per_period;
    s_target_counts[2] = s_target_counts[3] = right / per_period;
    s_last_command_us = esp_timer_get_time();
    taskEXIT_CRITICAL(&s_lock);
}

void motors_stop(void)
{
    taskENTER_CRITICAL(&s_lock);
    memset(s_target_counts, 0, sizeof(s_target_counts));
    taskEXIT_CRITICAL(&s_lock);
}

void motors_get_counts(int64_t counts[RAINBOW_NUM_WHEELS])
{
    for (int i = 0; i < RAINBOW_NUM_WHEELS; i++) counts[i] = read_count(i);
}

void motors_init(void)
{
    s_kp = CONFIG_RAINBOW_PID_KP_X100 / 100.0f;
    s_ki = CONFIG_RAINBOW_PID_KI_X100 / 100.0f;
    s_kd = CONFIG_RAINBOW_PID_KD_X100 / 100.0f;
    s_m_per_count = CONFIG_RAINBOW_WHEEL_CIRCUMFERENCE_UM / 1e6f / CONFIG_RAINBOW_ENCODER_COUNTS_PER_REV;
    s_half_track = CONFIG_RAINBOW_HALF_TRACK_MM / 1000.0f;
    s_max_speed = CONFIG_RAINBOW_MAX_WHEEL_SPEED_MM_S / 1000.0f;
    for (int i = 0; i < RAINBOW_NUM_WHEELS; i++) {
        pcnt_init_one(i);
        motor_init_one(i);
    }
    xTaskCreatePinnedToCore(motor_task, "motors", 4096, NULL, 10, NULL, 1);
}
