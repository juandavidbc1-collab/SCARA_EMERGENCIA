#pragma once
#include <stddef.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#define UART_NUM_0 0
#define UART_PIN_NO_CHANGE (-1)
typedef enum { UART_DATA_8_BITS } uart_word_length_t;
typedef enum { UART_PARITY_DISABLE } uart_parity_t;
typedef enum { UART_STOP_BITS_1 } uart_stop_bits_t;
typedef enum { UART_HW_FLOWCTRL_DISABLE } uart_hw_flowcontrol_t;
typedef struct { int baud_rate; uart_word_length_t data_bits; uart_parity_t parity; uart_stop_bits_t stop_bits; uart_hw_flowcontrol_t flow_ctrl; } uart_config_t;
esp_err_t uart_param_config(int, const uart_config_t *);
esp_err_t uart_set_pin(int, int, int, int, int);
esp_err_t uart_driver_install(int, int, int, int, void*, int);
esp_err_t uart_driver_delete(int);
int uart_write_bytes(int, const char *, size_t);
int uart_read_bytes(int, void*, uint32_t, TickType_t);
