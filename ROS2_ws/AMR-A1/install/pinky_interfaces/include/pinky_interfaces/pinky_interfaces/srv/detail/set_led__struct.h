// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pinky_interfaces:srv/SetLed.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinky_interfaces/srv/set_led.h"


#ifndef PINKY_INTERFACES__SRV__DETAIL__SET_LED__STRUCT_H_
#define PINKY_INTERFACES__SRV__DETAIL__SET_LED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"
// Member 'pixels'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/SetLed in the package pinky_interfaces.
typedef struct pinky_interfaces__srv__SetLed_Request
{
  rosidl_runtime_c__String command;
  /// 'set_pixel' 명령 시 사용할 픽셀 인덱스 배열
  rosidl_runtime_c__int32__Sequence pixels;
  int32_t r;
  int32_t g;
  int32_t b;
} pinky_interfaces__srv__SetLed_Request;

// Struct for a sequence of pinky_interfaces__srv__SetLed_Request.
typedef struct pinky_interfaces__srv__SetLed_Request__Sequence
{
  pinky_interfaces__srv__SetLed_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinky_interfaces__srv__SetLed_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SetLed in the package pinky_interfaces.
typedef struct pinky_interfaces__srv__SetLed_Response
{
  bool success;
  rosidl_runtime_c__String message;
} pinky_interfaces__srv__SetLed_Response;

// Struct for a sequence of pinky_interfaces__srv__SetLed_Response.
typedef struct pinky_interfaces__srv__SetLed_Response__Sequence
{
  pinky_interfaces__srv__SetLed_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinky_interfaces__srv__SetLed_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  pinky_interfaces__srv__SetLed_Event__request__MAX_SIZE = 1
};
// response
enum
{
  pinky_interfaces__srv__SetLed_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SetLed in the package pinky_interfaces.
typedef struct pinky_interfaces__srv__SetLed_Event
{
  service_msgs__msg__ServiceEventInfo info;
  pinky_interfaces__srv__SetLed_Request__Sequence request;
  pinky_interfaces__srv__SetLed_Response__Sequence response;
} pinky_interfaces__srv__SetLed_Event;

// Struct for a sequence of pinky_interfaces__srv__SetLed_Event.
typedef struct pinky_interfaces__srv__SetLed_Event__Sequence
{
  pinky_interfaces__srv__SetLed_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinky_interfaces__srv__SetLed_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PINKY_INTERFACES__SRV__DETAIL__SET_LED__STRUCT_H_
