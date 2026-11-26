// Servos: S1 (pan, GPIO8) and S2 (tilt, GPIO21), 50 Hz LEDC PWM.
//
// Angle to pulse as the stock firmware: -90..90 deg -> 500..2500 us, in whole
// degrees. Limits S1 [-90, 90], S2 [-90, 20]. Unlike the stock firmware, no pulse
// is sent until the first command, so a reset does not swing the head to a
// default pose (the head node resends its pose every second).

#include "driver/ledc.h"
#include "sdkconfig.h"

#include "rainbow.h"

#define SERVO_FREQ_HZ 50
#define SERVO_PERIOD_US 20000
#define SERVO_RES LEDC_TIMER_14_BIT
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

static const int GPIO[2] = {8, 21};
static const int MIN_DEG[2] = {-90, -90};
static const int MAX_DEG[2] = {90, 20};
static const int OFFSET[2] = {CONFIG_RAINBOW_SERVO_S1_OFFSET, CONFIG_RAINBOW_SERVO_S2_OFFSET};

void servos_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_RES,
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
    }
}

void servos_set(servo_id_t servo, int angle_deg)
{
    int i = (int)servo;
    if (i < 0 || i > 1) return;
    if (angle_deg < MIN_DEG[i]) angle_deg = MIN_DEG[i];
    if (angle_deg > MAX_DEG[i]) angle_deg = MAX_DEG[i];
    int a = angle_deg + OFFSET[i];
    int us = (a + 90) * (SERVO_MAX_US - SERVO_MIN_US) / 180 + SERVO_MIN_US;
    uint32_t duty = (uint32_t)us * (1u << 14) / SERVO_PERIOD_US;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
}
