/*
 * Transporte serial (UART0 = el puente USB de la DevKit) para micro-ROS / XRCE-DDS.
 * Basado en examples/int32_publisher_custom_transport del componente micro_ros_espidf_component.
 *
 * Como UART0 pasa a ser del transporte, en el modo micro-ROS la consola del ESP-IDF debe estar
 * desactivada (CONFIG_ESP_CONSOLE_NONE, ya incluido por tools/uros_select.py) y el firmware no
 * debe imprimir nada por printf/ESP_LOG. El monitor serie de PlatformIO ya no sirve en este modo.
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <uxr/client/transport.h>

bool   scara_serial_open(struct uxrCustomTransport *transport);
bool   scara_serial_close(struct uxrCustomTransport *transport);
size_t scara_serial_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t scara_serial_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);
