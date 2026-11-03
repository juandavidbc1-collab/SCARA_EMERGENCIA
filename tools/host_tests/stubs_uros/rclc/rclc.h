#pragma once
#include <rcl/rcl.h>
typedef struct { rcl_context_t context; } rclc_support_t;
rcl_ret_t rclc_support_init_with_options(rclc_support_t *, int, char const *const *, rcl_init_options_t *, rcl_allocator_t *);
rcl_ret_t rclc_support_fini(rclc_support_t *);
rcl_ret_t rclc_node_init_default(rcl_node_t *, const char *, const char *, rclc_support_t *);
rcl_ret_t rclc_publisher_init_default(rcl_publisher_t *, const rcl_node_t *, const rosidl_message_type_support_t *, const char *);
rcl_ret_t rclc_publisher_init_best_effort(rcl_publisher_t *, const rcl_node_t *, const rosidl_message_type_support_t *, const char *);
rcl_ret_t rclc_subscription_init_default(rcl_subscription_t *, rcl_node_t *, const rosidl_message_type_support_t *, const char *);
rcl_ret_t rclc_subscription_init_best_effort(rcl_subscription_t *, rcl_node_t *, const rosidl_message_type_support_t *, const char *);
rcl_ret_t rclc_timer_init_default2(rcl_timer_t *, rclc_support_t *, const uint64_t, const rcl_timer_callback_t, bool);
