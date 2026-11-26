// Servos: S1 (pan, GPIO8) and S2 (tilt, GPIO21), 50 Hz LEDC PWM.
//
// Angle to pulse as the stock firmware: -90..90 deg -> 500..2500 us, limits
// S1 [-90, 90], S2 [-90, 20]; the pulse takes fractions of a degree (14-bit
// duty: 1.22 us, ~0.11 deg). Unlike the stock firmware, no pulse is sent until
// the first command, so a reset does not swing the head to a default pose (the
// head node resends its pose every second).
//
// Two ways to command them:
// - servos_goal (topic servo_goal): each servo moves to its angle with limited
//   speed and acceleration (servo_motion.c), stepped once per 20 ms servo frame,
//   so every frame the servo reads gets one even step. Streaming positions from
//   the Pi instead beat against the frames (some got two steps, some none), and
//   whole degrees made the servos step visibly at every degree.
// - servos_set (servo_s1 / servo_s2, whole degrees, the stock contract): jump
//   there at the servo's own speed, cancelling any move.
// Both take effect at the next frame (<= 20 ms).

#include "driver/ledc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

#include "rainbow.h"
#include "servo_motion.h"

#define SERVO_FREQ_HZ 50
#define SERVO_PERIOD_US 20000
#define SERVO_RES_BITS 14  // the ESP32-S3's LEDC maximum
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

static const int GPIO[2] = {8, 21};
static const float MIN_DEG[2] = {-90.0f, -90.0f};
static const float MAX_DEG[2] = {90.0f, 20.0f};
static const int OFFSET[2] = {CONFIG_RAINBOW_SERVO_S1_OFFSET, CONFIG_RAINBOW_SERVO_S2_OFFSET};

static servo_motion_t s_motion[2];
static uint32_t s_duty[2];  // last duty written, 0 = no pulse yet
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static esp_timer_handle_t s_timer;

static uint32_t angle_to_duty(int i, float angle_deg)
{
    float us = (angle_deg + (float)OFFSET[i] + 90.0f) * (SERVO_MAX_US - SERVO_MIN_US) / 180.0f + SERVO_MIN_US;
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    return (uint32_t)(us * (float)(1u << SERVO_RES_BITS) / SERVO_PERIOD_US + 0.5f);
}

// Write the motion's position to the pulse (only for servos that were commanded).
// Only the frame timer calls it: a new command shows within one frame (20 ms).
static void output(void)
{
    float pos[2];
    bool known[2];
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < 2; i++) {
        pos[i] = s_motion[i].pos;
        known[i] = s_motion[i].known;
    }
    portEXIT_CRITICAL(&s_mux);
    for (int i = 0; i < 2; i++) {
        if (!known[i]) continue;
        uint32_t duty = angle_to_duty(i, pos[i]);
        if (duty == s_duty[i]) continue;
        s_duty[i] = duty;
        // Takes effect at the start of the next PWM period
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
    }
}

// Every servo frame (20 ms): one motion step. The timer and the LEDC run from
// the same crystal, so the step keeps its phase to the pulses.
static void on_frame(void *arg)
{
    const float dt = SERVO_PERIOD_US * 1e-6f;
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < 2; i++) servo_motion_step(&s_motion[i], dt);
    portEXIT_CRITICAL(&s_mux);
    output();
}

void servos_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)SERVO_RES_BITS,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t ch = {
            .gpio_num = GPIO[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
        servo_motion_init(&s_motion[i], MIN_DEG[i], MAX_DEG[i]);
    }
    const esp_timer_create_args_t args = {.callback = on_frame, .name = "servos"};
    ESP_ERROR_CHECK(esp_timer_create(&args, &s_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_timer, SERVO_PERIOD_US));
}

void servos_set(servo_id_t servo, int angle_deg)
{
    int i = (int)servo;
    if (i < 0 || i > 1) return;
    portENTER_CRITICAL(&s_mux);
    servo_motion_set(&s_motion[i], (float)angle_deg, 0.0f, 0.0f);  // jump
    portEXIT_CRITICAL(&s_mux);
}

void servos_goal(const float angle_deg[2], const float speed[2], const float accel[2])
{
    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < 2; i++) servo_motion_set(&s_motion[i], angle_deg[i], speed[i], accel[i]);
    portEXIT_CRITICAL(&s_mux);
}
