#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef int rcl_ret_t;
typedef int rmw_ret_t;
#define RCL_RET_OK 0
#define RMW_RET_OK 0
#define RCL_MS_TO_NS(x) ((int64_t)(x) * 1000000LL)
typedef struct { int a; } rcl_allocator_t;
typedef struct { int a; } rmw_context_t;
typedef struct { int a; } rmw_init_options_t;
typedef struct { int a; } rcl_context_t;
typedef struct { int a; } rcl_init_options_t;
typedef struct { int a; } rcl_node_t;
typedef struct { int a; } rcl_publisher_t;
typedef struct { int a; } rcl_subscription_t;
typedef struct { int a; } rcl_timer_t;
typedef struct { int a; } rosidl_message_type_support_t;
typedef void (*rcl_timer_callback_t)(rcl_timer_t *, int64_t);
#define ROSIDL_GET_MSG_TYPE_SUPPORT(pkg, kind, name) ((const rosidl_message_type_support_t *)0)
rcl_allocator_t rcl_get_default_allocator(void);
rcl_init_options_t rcl_get_zero_initialized_init_options(void);
rcl_ret_t rcl_init_options_init(rcl_init_options_t *, rcl_allocator_t);
rcl_ret_t rcl_init_options_fini(rcl_init_options_t *);
rmw_init_options_t *rcl_init_options_get_rmw_init_options(rcl_init_options_t *);
rcl_node_t rcl_get_zero_initialized_node(void);
rcl_ret_t rcl_node_fini(rcl_node_t *);
rcl_ret_t rcl_publisher_fini(rcl_publisher_t *, rcl_node_t *);
rcl_ret_t rcl_subscription_fini(rcl_subscription_t *, rcl_node_t *);
rcl_timer_t rcl_get_zero_initialized_timer(void);
rcl_ret_t rcl_timer_fini(rcl_timer_t *);
rcl_ret_t rcl_publish(const rcl_publisher_t *, const void *, void *);
rmw_context_t *rcl_context_get_rmw_context(rcl_context_t *);
