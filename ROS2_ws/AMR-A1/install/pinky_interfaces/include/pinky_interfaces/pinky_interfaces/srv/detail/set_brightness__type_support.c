// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from pinky_interfaces:srv/SetBrightness.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "pinky_interfaces/srv/detail/set_brightness__rosidl_typesupport_introspection_c.h"
#include "pinky_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "pinky_interfaces/srv/detail/set_brightness__functions.h"
#include "pinky_interfaces/srv/detail/set_brightness__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void pinky_interfaces__srv__SetBrightness_Request__rosidl_typesupport_introspection_c__SetBrightness_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinky_interfaces__srv__SetBrightness_Request__init(message_memory);
}

void pinky_interfaces__srv__SetBrightness_Request__rosidl_typesupport_introspection_c__SetBrightness_Request_fini_function(void * message_memory)
{
  pinky_interfaces__srv__SetBrightness_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pinky_interfaces__srv__SetBrightness_Request__rosidl_typesupport_introspection_c__SetBrightness_Request_message_member_array[1] = {
  {
    "brightness",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinky_interfaces__srv__SetBrightness_Request, brightness),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinky_interfaces__srv__SetBrightness_Request__rosidl_typesupport_introspection_c__SetBrightness_Request_message_members = {
  "pinky_interfaces__srv",  // message namespace
  "SetBrightness_Request",  // message name
  1,  // number of fields
  sizeof(pinky_interfaces__srv__SetBrightness_Request),
  false,  // has_any_key_member_
  pinky_interfaces__srv__SetBrightness_Request__rosidl_typesupport_introspection_c__SetBrightness_Request_message_member_array,  // message members
  pinky_interfaces__srv__SetBrightness_Request__rosidl_typesupport_introspection_c__SetBrightness_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  pinky_interfaces__srv__SetBrightness_Request__rosidl_typesupport_introspection_c__SetBrightness_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinky_interfaces__srv__SetBrightness_Request__rosidl_typesupport_introspection_c__SetBrightness_Request_message_type_support_handle = {
  0,
  &pinky_interfaces__srv__SetBrightness_Request__rosidl_typesupport_introspection_c__SetBrightness_Request_message_members,
  get_message_typesupport_handle_function,
  &pinky_interfaces__srv__SetBrightness_Request__get_type_hash,
  &pinky_interfaces__srv__SetBrightness_Request__get_type_description,
  &pinky_interfaces__srv__SetBrightness_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinky_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinky_interfaces, srv, SetBrightness_Request)() {
  if (!pinky_interfaces__srv__SetBrightness_Request__rosidl_typesupport_introspection_c__SetBrightness_Request_message_type_support_handle.typesupport_identifier) {
    pinky_interfaces__srv__SetBrightness_Request__rosidl_typesupport_introspection_c__SetBrightness_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinky_interfaces__srv__SetBrightness_Request__rosidl_typesupport_introspection_c__SetBrightness_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "pinky_interfaces/srv/detail/set_brightness__rosidl_typesupport_introspection_c.h"
// already included above
// #include "pinky_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "pinky_interfaces/srv/detail/set_brightness__functions.h"
// already included above
// #include "pinky_interfaces/srv/detail/set_brightness__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pinky_interfaces__srv__SetBrightness_Response__rosidl_typesupport_introspection_c__SetBrightness_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinky_interfaces__srv__SetBrightness_Response__init(message_memory);
}

void pinky_interfaces__srv__SetBrightness_Response__rosidl_typesupport_introspection_c__SetBrightness_Response_fini_function(void * message_memory)
{
  pinky_interfaces__srv__SetBrightness_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember pinky_interfaces__srv__SetBrightness_Response__rosidl_typesupport_introspection_c__SetBrightness_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinky_interfaces__srv__SetBrightness_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinky_interfaces__srv__SetBrightness_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinky_interfaces__srv__SetBrightness_Response__rosidl_typesupport_introspection_c__SetBrightness_Response_message_members = {
  "pinky_interfaces__srv",  // message namespace
  "SetBrightness_Response",  // message name
  2,  // number of fields
  sizeof(pinky_interfaces__srv__SetBrightness_Response),
  false,  // has_any_key_member_
  pinky_interfaces__srv__SetBrightness_Response__rosidl_typesupport_introspection_c__SetBrightness_Response_message_member_array,  // message members
  pinky_interfaces__srv__SetBrightness_Response__rosidl_typesupport_introspection_c__SetBrightness_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  pinky_interfaces__srv__SetBrightness_Response__rosidl_typesupport_introspection_c__SetBrightness_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinky_interfaces__srv__SetBrightness_Response__rosidl_typesupport_introspection_c__SetBrightness_Response_message_type_support_handle = {
  0,
  &pinky_interfaces__srv__SetBrightness_Response__rosidl_typesupport_introspection_c__SetBrightness_Response_message_members,
  get_message_typesupport_handle_function,
  &pinky_interfaces__srv__SetBrightness_Response__get_type_hash,
  &pinky_interfaces__srv__SetBrightness_Response__get_type_description,
  &pinky_interfaces__srv__SetBrightness_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinky_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinky_interfaces, srv, SetBrightness_Response)() {
  if (!pinky_interfaces__srv__SetBrightness_Response__rosidl_typesupport_introspection_c__SetBrightness_Response_message_type_support_handle.typesupport_identifier) {
    pinky_interfaces__srv__SetBrightness_Response__rosidl_typesupport_introspection_c__SetBrightness_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinky_interfaces__srv__SetBrightness_Response__rosidl_typesupport_introspection_c__SetBrightness_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "pinky_interfaces/srv/detail/set_brightness__rosidl_typesupport_introspection_c.h"
// already included above
// #include "pinky_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "pinky_interfaces/srv/detail/set_brightness__functions.h"
// already included above
// #include "pinky_interfaces/srv/detail/set_brightness__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "pinky_interfaces/srv/set_brightness.h"
// Member `request`
// Member `response`
// already included above
// #include "pinky_interfaces/srv/detail/set_brightness__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__SetBrightness_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  pinky_interfaces__srv__SetBrightness_Event__init(message_memory);
}

void pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__SetBrightness_Event_fini_function(void * message_memory)
{
  pinky_interfaces__srv__SetBrightness_Event__fini(message_memory);
}

size_t pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__size_function__SetBrightness_Event__request(
  const void * untyped_member)
{
  const pinky_interfaces__srv__SetBrightness_Request__Sequence * member =
    (const pinky_interfaces__srv__SetBrightness_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__get_const_function__SetBrightness_Event__request(
  const void * untyped_member, size_t index)
{
  const pinky_interfaces__srv__SetBrightness_Request__Sequence * member =
    (const pinky_interfaces__srv__SetBrightness_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__get_function__SetBrightness_Event__request(
  void * untyped_member, size_t index)
{
  pinky_interfaces__srv__SetBrightness_Request__Sequence * member =
    (pinky_interfaces__srv__SetBrightness_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__fetch_function__SetBrightness_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const pinky_interfaces__srv__SetBrightness_Request * item =
    ((const pinky_interfaces__srv__SetBrightness_Request *)
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__get_const_function__SetBrightness_Event__request(untyped_member, index));
  pinky_interfaces__srv__SetBrightness_Request * value =
    (pinky_interfaces__srv__SetBrightness_Request *)(untyped_value);
  *value = *item;
}

void pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__assign_function__SetBrightness_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  pinky_interfaces__srv__SetBrightness_Request * item =
    ((pinky_interfaces__srv__SetBrightness_Request *)
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__get_function__SetBrightness_Event__request(untyped_member, index));
  const pinky_interfaces__srv__SetBrightness_Request * value =
    (const pinky_interfaces__srv__SetBrightness_Request *)(untyped_value);
  *item = *value;
}

bool pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__resize_function__SetBrightness_Event__request(
  void * untyped_member, size_t size)
{
  pinky_interfaces__srv__SetBrightness_Request__Sequence * member =
    (pinky_interfaces__srv__SetBrightness_Request__Sequence *)(untyped_member);
  pinky_interfaces__srv__SetBrightness_Request__Sequence__fini(member);
  return pinky_interfaces__srv__SetBrightness_Request__Sequence__init(member, size);
}

size_t pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__size_function__SetBrightness_Event__response(
  const void * untyped_member)
{
  const pinky_interfaces__srv__SetBrightness_Response__Sequence * member =
    (const pinky_interfaces__srv__SetBrightness_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__get_const_function__SetBrightness_Event__response(
  const void * untyped_member, size_t index)
{
  const pinky_interfaces__srv__SetBrightness_Response__Sequence * member =
    (const pinky_interfaces__srv__SetBrightness_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__get_function__SetBrightness_Event__response(
  void * untyped_member, size_t index)
{
  pinky_interfaces__srv__SetBrightness_Response__Sequence * member =
    (pinky_interfaces__srv__SetBrightness_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__fetch_function__SetBrightness_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const pinky_interfaces__srv__SetBrightness_Response * item =
    ((const pinky_interfaces__srv__SetBrightness_Response *)
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__get_const_function__SetBrightness_Event__response(untyped_member, index));
  pinky_interfaces__srv__SetBrightness_Response * value =
    (pinky_interfaces__srv__SetBrightness_Response *)(untyped_value);
  *value = *item;
}

void pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__assign_function__SetBrightness_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  pinky_interfaces__srv__SetBrightness_Response * item =
    ((pinky_interfaces__srv__SetBrightness_Response *)
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__get_function__SetBrightness_Event__response(untyped_member, index));
  const pinky_interfaces__srv__SetBrightness_Response * value =
    (const pinky_interfaces__srv__SetBrightness_Response *)(untyped_value);
  *item = *value;
}

bool pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__resize_function__SetBrightness_Event__response(
  void * untyped_member, size_t size)
{
  pinky_interfaces__srv__SetBrightness_Response__Sequence * member =
    (pinky_interfaces__srv__SetBrightness_Response__Sequence *)(untyped_member);
  pinky_interfaces__srv__SetBrightness_Response__Sequence__fini(member);
  return pinky_interfaces__srv__SetBrightness_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__SetBrightness_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pinky_interfaces__srv__SetBrightness_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(pinky_interfaces__srv__SetBrightness_Event, request),  // bytes offset in struct
    NULL,  // default value
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__size_function__SetBrightness_Event__request,  // size() function pointer
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__get_const_function__SetBrightness_Event__request,  // get_const(index) function pointer
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__get_function__SetBrightness_Event__request,  // get(index) function pointer
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__fetch_function__SetBrightness_Event__request,  // fetch(index, &value) function pointer
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__assign_function__SetBrightness_Event__request,  // assign(index, value) function pointer
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__resize_function__SetBrightness_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(pinky_interfaces__srv__SetBrightness_Event, response),  // bytes offset in struct
    NULL,  // default value
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__size_function__SetBrightness_Event__response,  // size() function pointer
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__get_const_function__SetBrightness_Event__response,  // get_const(index) function pointer
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__get_function__SetBrightness_Event__response,  // get(index) function pointer
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__fetch_function__SetBrightness_Event__response,  // fetch(index, &value) function pointer
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__assign_function__SetBrightness_Event__response,  // assign(index, value) function pointer
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__resize_function__SetBrightness_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__SetBrightness_Event_message_members = {
  "pinky_interfaces__srv",  // message namespace
  "SetBrightness_Event",  // message name
  3,  // number of fields
  sizeof(pinky_interfaces__srv__SetBrightness_Event),
  false,  // has_any_key_member_
  pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__SetBrightness_Event_message_member_array,  // message members
  pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__SetBrightness_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__SetBrightness_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__SetBrightness_Event_message_type_support_handle = {
  0,
  &pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__SetBrightness_Event_message_members,
  get_message_typesupport_handle_function,
  &pinky_interfaces__srv__SetBrightness_Event__get_type_hash,
  &pinky_interfaces__srv__SetBrightness_Event__get_type_description,
  &pinky_interfaces__srv__SetBrightness_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinky_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinky_interfaces, srv, SetBrightness_Event)() {
  pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__SetBrightness_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__SetBrightness_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinky_interfaces, srv, SetBrightness_Request)();
  pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__SetBrightness_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinky_interfaces, srv, SetBrightness_Response)();
  if (!pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__SetBrightness_Event_message_type_support_handle.typesupport_identifier) {
    pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__SetBrightness_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__SetBrightness_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "pinky_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "pinky_interfaces/srv/detail/set_brightness__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers pinky_interfaces__srv__detail__set_brightness__rosidl_typesupport_introspection_c__SetBrightness_service_members = {
  "pinky_interfaces__srv",  // service namespace
  "SetBrightness",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // pinky_interfaces__srv__detail__set_brightness__rosidl_typesupport_introspection_c__SetBrightness_Request_message_type_support_handle,
  NULL,  // response message
  // pinky_interfaces__srv__detail__set_brightness__rosidl_typesupport_introspection_c__SetBrightness_Response_message_type_support_handle
  NULL  // event_message
  // pinky_interfaces__srv__detail__set_brightness__rosidl_typesupport_introspection_c__SetBrightness_Response_message_type_support_handle
};


static rosidl_service_type_support_t pinky_interfaces__srv__detail__set_brightness__rosidl_typesupport_introspection_c__SetBrightness_service_type_support_handle = {
  0,
  &pinky_interfaces__srv__detail__set_brightness__rosidl_typesupport_introspection_c__SetBrightness_service_members,
  get_service_typesupport_handle_function,
  &pinky_interfaces__srv__SetBrightness_Request__rosidl_typesupport_introspection_c__SetBrightness_Request_message_type_support_handle,
  &pinky_interfaces__srv__SetBrightness_Response__rosidl_typesupport_introspection_c__SetBrightness_Response_message_type_support_handle,
  &pinky_interfaces__srv__SetBrightness_Event__rosidl_typesupport_introspection_c__SetBrightness_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    pinky_interfaces,
    srv,
    SetBrightness
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    pinky_interfaces,
    srv,
    SetBrightness
  ),
  &pinky_interfaces__srv__SetBrightness__get_type_hash,
  &pinky_interfaces__srv__SetBrightness__get_type_description,
  &pinky_interfaces__srv__SetBrightness__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinky_interfaces, srv, SetBrightness_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinky_interfaces, srv, SetBrightness_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinky_interfaces, srv, SetBrightness_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_pinky_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinky_interfaces, srv, SetBrightness)(void) {
  if (!pinky_interfaces__srv__detail__set_brightness__rosidl_typesupport_introspection_c__SetBrightness_service_type_support_handle.typesupport_identifier) {
    pinky_interfaces__srv__detail__set_brightness__rosidl_typesupport_introspection_c__SetBrightness_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)pinky_interfaces__srv__detail__set_brightness__rosidl_typesupport_introspection_c__SetBrightness_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinky_interfaces, srv, SetBrightness_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinky_interfaces, srv, SetBrightness_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, pinky_interfaces, srv, SetBrightness_Event)()->data;
  }

  return &pinky_interfaces__srv__detail__set_brightness__rosidl_typesupport_introspection_c__SetBrightness_service_type_support_handle;
}
