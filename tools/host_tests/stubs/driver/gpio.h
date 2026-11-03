#pragma once
#include <stdint.h>
#include "esp_err.h"
typedef enum { GPIO_NUM_NC=-1, GPIO_NUM_0=0, GPIO_NUM_1,GPIO_NUM_2,GPIO_NUM_3,GPIO_NUM_4,GPIO_NUM_5,GPIO_NUM_6,GPIO_NUM_7,GPIO_NUM_8,GPIO_NUM_9,GPIO_NUM_10,GPIO_NUM_11,GPIO_NUM_12,GPIO_NUM_13,GPIO_NUM_14,GPIO_NUM_15,GPIO_NUM_16,GPIO_NUM_17,GPIO_NUM_18,GPIO_NUM_19,GPIO_NUM_20,GPIO_NUM_21,GPIO_NUM_22,GPIO_NUM_23,GPIO_NUM_24,GPIO_NUM_25,GPIO_NUM_26,GPIO_NUM_27,GPIO_NUM_28,GPIO_NUM_29,GPIO_NUM_30,GPIO_NUM_31,GPIO_NUM_32,GPIO_NUM_33,GPIO_NUM_34,GPIO_NUM_35,GPIO_NUM_36,GPIO_NUM_37,GPIO_NUM_38,GPIO_NUM_39 } gpio_num_t;
typedef enum { GPIO_MODE_INPUT=1, GPIO_MODE_OUTPUT=2 } gpio_mode_t;
typedef enum { GPIO_PULLUP_DISABLE, GPIO_PULLUP_ENABLE } gpio_pullup_t;
typedef enum { GPIO_PULLDOWN_DISABLE, GPIO_PULLDOWN_ENABLE } gpio_pulldown_t;
typedef struct { uint64_t pin_bit_mask; gpio_mode_t mode; gpio_pullup_t pull_up_en; gpio_pulldown_t pull_down_en; int intr_type; } gpio_config_t;
esp_err_t gpio_config(const gpio_config_t *c);
esp_err_t gpio_set_level(gpio_num_t g, uint32_t l);
int gpio_get_level(gpio_num_t g);
