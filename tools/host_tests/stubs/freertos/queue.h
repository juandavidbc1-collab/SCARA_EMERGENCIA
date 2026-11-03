#pragma once
#include "freertos/FreeRTOS.h"
QueueHandle_t xQueueCreate(int, int);
BaseType_t xQueueSend(QueueHandle_t, const void*, TickType_t);
BaseType_t xQueueReceive(QueueHandle_t, void*, TickType_t);
