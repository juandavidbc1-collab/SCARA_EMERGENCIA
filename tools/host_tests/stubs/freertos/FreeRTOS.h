#pragma once
#include <stdint.h>
typedef uint32_t TickType_t; typedef int BaseType_t; typedef void *QueueHandle_t; typedef void *TaskHandle_t;
typedef struct { int x; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0}
#define portENTER_CRITICAL(m) (void)(m)
#define portEXIT_CRITICAL(m) (void)(m)
#define pdMS_TO_TICKS(x) ((TickType_t)(x))
#define pdTRUE 1
#define configMAX_PRIORITIES 25
