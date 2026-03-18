#include "comon.h"

#include "driver/ledc.h"

#define TAG "POWER"

esp_err_t power_contr_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = PWM_MODE,
        .timer_num        = PWM_TIMER,
        .duty_resolution  = PWM_RESOLUTION,
        .freq_hz          = PWM_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK)
    {
        return ret;
    }

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = PWM_MODE,
        .channel        = PWM_CHANNEL,
        .timer_sel      = PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = FAN_GPIO,
        .duty           = 0,
        .hpoint         = 0,
        .flags.output_invert = 1
    };
    return ledc_channel_config(&ledc_channel);
}

esp_err_t power_contr_set(uint16_t value)
{
    uint32_t duty_raw = (uint32_t)(value * 81.92);
    if (duty_raw > 8192)
    {
        duty_raw = 8192;
    }
    //ESP_LOGI(TAG,"DUT RAW: %ld", duty_raw);
    esp_err_t ret = ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty_raw);
    if(ret != ESP_OK)
    {
        return ret;
    }

    ret = ledc_update_duty(PWM_MODE, PWM_CHANNEL);

    return ret;
}