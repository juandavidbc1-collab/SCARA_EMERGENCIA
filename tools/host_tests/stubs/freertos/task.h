#pragma once
#include "freertos/FreeRTOS.h"
typedef void (*TaskFunction_t)(void*);
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *prev, TickType_t inc);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char*, uint32_t, void*, int, TaskHandle_t*, int);
