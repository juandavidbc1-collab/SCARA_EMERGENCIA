#pragma once
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#define UART_NUM_0 0
esp_err_t uart_driver_install(int, int, int, int, void*, int);
int uart_read_bytes(int, void*, uint32_t, TickType_t);
