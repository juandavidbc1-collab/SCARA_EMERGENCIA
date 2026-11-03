#pragma once
#include "driver/gpio.h"
typedef enum { LEDC_LOW_SPEED_MODE=0 } ledc_mode_t;
typedef enum { LEDC_TIMER_0=0 } ledc_timer_t;
typedef enum { LEDC_TIMER_10_BIT=10 } ledc_timer_bit_t;
typedef enum { LEDC_AUTO_CLK=0 } ledc_clk_cfg_t;
typedef enum { LEDC_CHANNEL_0=0, LEDC_CHANNEL_1, LEDC_CHANNEL_2 } ledc_channel_t;
typedef struct { ledc_mode_t speed_mode; ledc_timer_bit_t duty_resolution; ledc_timer_t timer_num; uint32_t freq_hz; ledc_clk_cfg_t clk_cfg; } ledc_timer_config_t;
typedef struct { int gpio_num; ledc_mode_t speed_mode; ledc_channel_t channel; ledc_timer_t timer_sel; uint32_t duty; int hpoint; } ledc_channel_config_t;
esp_err_t ledc_timer_config(const ledc_timer_config_t *c);
esp_err_t ledc_channel_config(const ledc_channel_config_t *c);
esp_err_t ledc_set_duty(ledc_mode_t m, ledc_channel_t c, uint32_t d);
esp_err_t ledc_update_duty(ledc_mode_t m, ledc_channel_t c);
