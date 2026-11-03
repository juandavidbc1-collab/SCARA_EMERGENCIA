#pragma once
#include <stddef.h>
#include <stdint.h>
typedef struct { float *data; size_t size; size_t capacity; } rosidl_runtime_c__float__Sequence;
typedef struct { void *data; size_t size; size_t capacity; } std_msgs__msg__MultiArrayDimension__Sequence;
typedef struct { std_msgs__msg__MultiArrayDimension__Sequence dim; uint32_t data_offset; } std_msgs__msg__MultiArrayLayout;
typedef struct { std_msgs__msg__MultiArrayLayout layout; rosidl_runtime_c__float__Sequence data; } std_msgs__msg__Float32MultiArray;
