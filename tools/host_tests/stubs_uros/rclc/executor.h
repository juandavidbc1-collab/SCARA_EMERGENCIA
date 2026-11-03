#pragma once
#include <rclc/rclc.h>
typedef struct { int a; } rclc_executor_t;
typedef void (*rclc_subscription_callback_t)(const void *);
typedef enum { ON_NEW_DATA, ALWAYS } rclc_executor_handle_invocation_t;
rclc_executor_t rclc_executor_get_zero_initialized_executor(void);
rcl_ret_t rclc_executor_init(rclc_executor_t *, rcl_context_t *, const size_t, const rcl_allocator_t *);
rcl_ret_t rclc_executor_fini(rclc_executor_t *);
rcl_ret_t rclc_executor_add_timer(rclc_executor_t *, rcl_timer_t *);
rcl_ret_t rclc_executor_add_subscription(rclc_executor_t *, rcl_subscription_t *, void *, rclc_subscription_callback_t, rclc_executor_handle_invocation_t);
rcl_ret_t rclc_executor_spin_some(rclc_executor_t *, const uint64_t);
