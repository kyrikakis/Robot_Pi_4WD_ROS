// Motors: MCPWM H-bridge drive, PCNT quadrature encoders, per-wheel speed PID.
//
// Kept equal to the stock firmware so the ROS side (velocity watchdog gains, EKF)
// sees the same robot: 25 kHz PWM with a 400-tick period, a 200-tick dead zone
// added to any non-zero output, an incremental PID on encoder counts per 10 ms,
// and wheel speed = v -/+ w * half_track.
//
// Differences: a wheel whose target is zero brakes and its PID resets (the stock
// PID hunted around zero through the dead zone), the motors stop when cmd_vel is
// silent for CONFIG_RAINBOW_CMD_TIMEOUT_MS, and a runaway guard stops them all
// when a wheel at full power reads the opposite way to its target (a reversed
// encoder makes the PID push harder the faster the wheel turns).

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
#define RUNAWAY_PERIODS 30  // 0.3 s at full power the wrong way
#define RUNAWAY_BEEP_MS 500

typedef struct {
    int pwm_a;     // GPIO driven for forward
    int pwm_b;
    int group;     // MCPWM group (3 timers each: M1-M3 in 0, M4 in 1)
    int enc_a;
    int enc_b;
    int enc_sign;  // +1, or -1 where the encoder counts backward for a forward wheel
} wheel_pins_t;

// M1 and M2 (left side) are wired mirrored: their B pin drives forward.
// M3 and M4 (right side) have their encoders mirrored instead: turned forward by
// hand they count down (2026-11-27; the right PID ran away to full power).
static const wheel_pins_t PINS[RAINBOW_NUM_WHEELS] = {
    {.pwm_a = 5, .pwm_b = 4, .group = 0, .enc_a = 6, .enc_b = 7, .enc_sign = 1},
    {.pwm_a = 16, .pwm_b = 15, .group = 0, .enc_a = 47, .enc_b = 48, .enc_sign = 1},
    {.pwm_a = 9, .pwm_b = 10, .group = 0, .enc_a = 11, .enc_b = 12, .enc_sign = -1},
    {.pwm_a = 13, .pwm_b = 14, .group = 1, .enc_a = 1, .enc_b = 2, .enc_sign = -1},
};

typedef struct {
    float e1, e2, out;
} pid_state_t;

static bdc_motor_handle_t s_motor[RAINBOW_NUM_WHEELS];
static pcnt_unit_handle_t s_pcnt[RAINBOW_NUM_WHEELS];
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static float s_target_counts[RAINBOW_NUM_WHEELS];  // counts per PID period
static int64_t s_last_command_us;
static bool s_fault;  // runaway: all wheels stopped until a zero command

static volatile float s_gain[MOTOR_NUM_GAINS];  // kp, ki, kd, kf
static float s_m_per_count;
static float s_half_track;
static float s_max_speed;

float motors_metres_per_count(void) { return s_m_per_count; }
float motors_half_track(void) { return s_half_track; }
float motors_get_gain(motor_gain_t g) { return s_gain[g]; }
void motors_set_gain(motor_gain_t g, float value) { s_gain[g] = value; }  // one aligned float: atomic

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
// Plus feedforward kf * target; the PID's part is clamped so the sum stays within
// the output range (no windup past it).
static float pid_step(pid_state_t *p, float target, float error)
{
    float kp = s_gain[MOTOR_GAIN_KP], ki = s_gain[MOTOR_GAIN_KI], kd = s_gain[MOTOR_GAIN_KD];
    float ff = s_gain[MOTOR_GAIN_KF] * target;
    if (ff > PID_OUTPUT_MAX) ff = PID_OUTPUT_MAX;
    if (ff < -PID_OUTPUT_MAX) ff = -PID_OUTPUT_MAX;
    p->out += kp * (error - p->e1) + ki * error + kd * (error - 2.0f * p->e1 + p->e2);
    if (p->out > PID_OUTPUT_MAX - ff) p->out = PID_OUTPUT_MAX - ff;
    if (p->out < -PID_OUTPUT_MAX - ff) p->out = -PID_OUTPUT_MAX - ff;
    p->e2 = p->e1;
    p->e1 = error;
    return p->out + ff;
}

static int read_count(int i)
{
    int c = 0;
    pcnt_unit_get_count(s_pcnt[i], &c);
    return c * PINS[i].enc_sign;
}

static void motor_task(void *arg)
{
    pid_state_t pid[RAINBOW_NUM_WHEELS] = {0};
    int wrong_way[RAINBOW_NUM_WHEELS] = {0};
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
        if (s_fault) memset(s_target_counts, 0, sizeof(s_target_counts));
        memcpy(target, s_target_counts, sizeof(target));
        taskEXIT_CRITICAL(&s_lock);

        for (int i = 0; i < RAINBOW_NUM_WHEELS; i++) {
            int c = read_count(i);
            float measured = (float)(c - last[i]);
            last[i] = c;
            if (target[i] == 0.0f) {
                memset(&pid[i], 0, sizeof(pid[i]));
                wrong_way[i] = 0;
                motor_drive(i, 0);
                continue;
            }
            float out = pid_step(&pid[i], target[i], target[i] - measured);
            bool saturated = fabsf(out) >= PID_OUTPUT_MAX;
            if (saturated && measured * target[i] < 0.0f) {
                wrong_way[i]++;
            } else {
                wrong_way[i] = 0;
            }
            if (wrong_way[i] >= RUNAWAY_PERIODS) {
                beep_command(RUNAWAY_BEEP_MS);  // no console: the buzzer says it
                taskENTER_CRITICAL(&s_lock);
                s_fault = true;
                memset(s_target_counts, 0, sizeof(s_target_counts));
                taskEXIT_CRITICAL(&s_lock);
                for (int j = 0; j < RAINBOW_NUM_WHEELS; j++) {
                    memset(&pid[j], 0, sizeof(pid[j]));
                    wrong_way[j] = 0;
                    motor_drive(j, 0);
                }
                break;
            }
            motor_drive(i, (int)out);
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
    if (left == 0.0f && right == 0.0f) s_fault = false;
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
    s_gain[MOTOR_GAIN_KP] = CONFIG_RAINBOW_PID_KP_X100 / 100.0f;
    s_gain[MOTOR_GAIN_KI] = CONFIG_RAINBOW_PID_KI_X100 / 100.0f;
    s_gain[MOTOR_GAIN_KD] = CONFIG_RAINBOW_PID_KD_X100 / 100.0f;
    s_gain[MOTOR_GAIN_KF] = CONFIG_RAINBOW_PID_KF_X100 / 100.0f;
    s_m_per_count = CONFIG_RAINBOW_WHEEL_CIRCUMFERENCE_UM / 1e6f / CONFIG_RAINBOW_ENCODER_COUNTS_PER_REV;
    s_half_track = CONFIG_RAINBOW_HALF_TRACK_MM / 1000.0f;
    s_max_speed = CONFIG_RAINBOW_MAX_WHEEL_SPEED_MM_S / 1000.0f;
    for (int i = 0; i < RAINBOW_NUM_WHEELS; i++) {
        pcnt_init_one(i);
        motor_init_one(i);
    }
    xTaskCreatePinnedToCore(motor_task, "motors", 4096, NULL, 10, NULL, 1);
}
