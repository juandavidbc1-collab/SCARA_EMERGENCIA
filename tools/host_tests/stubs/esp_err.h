#pragma once
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERROR_CHECK(x) do { esp_err_t _e = (x); (void)_e; } while (0)
