#include "uros_serial_transport.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

#define UART_BUFFER_SIZE 1024

bool scara_serial_open(struct uxrCustomTransport *transport)
{
    size_t *port = (size_t *)transport->args;
    uart_config_t cfg = {
        .baud_rate = CONFIG_SCARA_UROS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    if (uart_param_config(*port, &cfg) != ESP_OK) return false;
    /* pines por defecto de UART0 (TX = GPIO1, RX = GPIO3: los del puente USB) */
    if (uart_set_pin(*port, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return false;
    if (uart_driver_install(*port, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0) != ESP_OK) return false;
    return true;
}

bool scara_serial_close(struct uxrCustomTransport *transport)
{
    size_t *port = (size_t *)transport->args;
    return uart_driver_delete(*port) == ESP_OK;
}

size_t scara_serial_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err)
{
    size_t *port = (size_t *)transport->args;
    int w = uart_write_bytes(*port, (const char *)buf, len);
    if (w < 0) { *err = 1; return 0; }
    return (size_t)w;
}

size_t scara_serial_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    size_t *port = (size_t *)transport->args;
    int r = uart_read_bytes(*port, buf, len, pdMS_TO_TICKS(timeout < 0 ? 0 : timeout));
    if (r < 0) { *err = 1; return 0; }
    return (size_t)r;
}
