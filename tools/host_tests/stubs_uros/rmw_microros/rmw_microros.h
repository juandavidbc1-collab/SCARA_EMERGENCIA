#pragma once
#include <rcl/rcl.h>
#include <uxr/client/transport.h>
typedef bool (*open_custom_func)(struct uxrCustomTransport *);
typedef bool (*close_custom_func)(struct uxrCustomTransport *);
typedef size_t (*write_custom_func)(struct uxrCustomTransport *, const uint8_t *, size_t, uint8_t *);
typedef size_t (*read_custom_func)(struct uxrCustomTransport *, uint8_t *, size_t, int, uint8_t *);
rmw_ret_t rmw_uros_set_custom_transport(bool, void *, open_custom_func, close_custom_func, write_custom_func, read_custom_func);
rmw_ret_t rmw_uros_ping_agent(const int, const uint8_t);
rmw_ret_t rmw_uros_options_set_udp_address(const char *, const char *, rmw_init_options_t *);
rmw_ret_t rmw_uros_set_context_entity_destroy_session_timeout(rmw_context_t *, int);
