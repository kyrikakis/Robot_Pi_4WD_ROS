// Buzzer on GPIO46 (active high): 0 off, 1 on, >= 10 on for that many ms.

#include "driver/gpio.h"
#include "esp_timer.h"

#include "rainbow.h"

#define BEEP_GPIO 46

static esp_timer_handle_t s_off_timer;

static void off_cb(void *arg) { gpio_set_level(BEEP_GPIO, 0); }

void beep_init(void)
{
    gpio_reset_pin(BEEP_GPIO);
    gpio_set_direction(BEEP_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BEEP_GPIO, 0);
    esp_timer_create_args_t args = {.callback = off_cb, .name = "beep_off"};
    ESP_ERROR_CHECK(esp_timer_create(&args, &s_off_timer));
}

void beep_command(uint16_t value)
{
    esp_timer_stop(s_off_timer);
    if (value == 1) {
        gpio_set_level(BEEP_GPIO, 1);
    } else if (value >= 10) {
        gpio_set_level(BEEP_GPIO, 1);
        esp_timer_start_once(s_off_timer, (uint64_t)value * 1000);
    } else {
        gpio_set_level(BEEP_GPIO, 0);
    }
}
