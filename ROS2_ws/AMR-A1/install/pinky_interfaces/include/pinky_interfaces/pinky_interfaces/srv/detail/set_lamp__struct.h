// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pinky_interfaces:srv/SetLamp.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinky_interfaces/srv/set_lamp.h"


#ifndef PINKY_INTERFACES__SRV__DETAIL__SET_LAMP__STRUCT_H_
#define PINKY_INTERFACES__SRV__DETAIL__SET_LAMP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'color'
#include "std_msgs/msg/detail/color_rgba__struct.h"

/// Struct defined in srv/SetLamp in the package pinky_interfaces.
typedef struct pinky_interfaces__srv__SetLamp_Request
{
  std_msgs__msg__ColorRGBA color;
  uint8_t mode;
  uint16_t time;
} pinky_interfaces__srv__SetLamp_Request;

// Struct for a sequence of pinky_interfaces__srv__SetLamp_Request.
typedef struct pinky_interfaces__srv__SetLamp_Request__Sequence
{
  pinky_interfaces__srv__SetLamp_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinky_interfaces__srv__SetLamp_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/SetLamp in the package pinky_interfaces.
typedef struct pinky_interfaces__srv__SetLamp_Response
{
  bool result;
} pinky_interfaces__srv__SetLamp_Response;

// Struct for a sequence of pinky_interfaces__srv__SetLamp_Response.
typedef struct pinky_interfaces__srv__SetLamp_Response__Sequence
{
  pinky_interfaces__srv__SetLamp_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinky_interfaces__srv__SetLamp_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  pinky_interfaces__srv__SetLamp_Event__request__MAX_SIZE = 1
};
// response
enum
{
  pinky_interfaces__srv__SetLamp_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/SetLamp in the package pinky_interfaces.
typedef struct pinky_interfaces__srv__SetLamp_Event
{
  service_msgs__msg__ServiceEventInfo info;
  pinky_interfaces__srv__SetLamp_Request__Sequence request;
  pinky_interfaces__srv__SetLamp_Response__Sequence response;
} pinky_interfaces__srv__SetLamp_Event;

// Struct for a sequence of pinky_interfaces__srv__SetLamp_Event.
typedef struct pinky_interfaces__srv__SetLamp_Event__Sequence
{
  pinky_interfaces__srv__SetLamp_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pinky_interfaces__srv__SetLamp_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PINKY_INTERFACES__SRV__DETAIL__SET_LAMP__STRUCT_H_
