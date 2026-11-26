// Battery voltage: GPIO3 (ADC1 channel 2) behind a x4.03 divider, as the stock firmware.
// Sampled at 10 Hz, averaged over the last second.

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "rainbow.h"

#define BAT_CHANNEL ADC_CHANNEL_2
#define BAT_DIVIDER 4.03f
#define BAT_SAMPLES 10

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static volatile float s_volts;

static void battery_task(void *arg)
{
    int history[BAT_SAMPLES] = {0};
    int n = 0, next = 0;
    while (1) {
        int raw = 0, mv = 0;
        if (adc_oneshot_read(s_adc, BAT_CHANNEL, &raw) == ESP_OK &&
            adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) {
            history[next] = mv;
            next = (next + 1) % BAT_SAMPLES;
            if (n < BAT_SAMPLES) n++;
            int sum = 0;
            for (int i = 0; i < n; i++) sum += history[i];
            s_volts = sum / (float)n / 1000.0f * BAT_DIVIDER;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

float battery_volts(void) { return s_volts; }

void battery_init(void)
{
    adc_oneshot_unit_init_cfg_t unit = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &s_adc));
    adc_oneshot_chan_cfg_t ch = {.bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, BAT_CHANNEL, &ch));
    adc_cali_curve_fitting_config_t cali = {
        .unit_id = ADC_UNIT_1,
        .chan = BAT_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali, &s_cali));
    xTaskCreatePinnedToCore(battery_task, "battery", 3072, NULL, 2, NULL, 1);
}
