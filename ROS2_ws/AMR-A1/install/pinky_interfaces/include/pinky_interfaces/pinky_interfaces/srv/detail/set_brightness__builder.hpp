// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pinky_interfaces:srv/SetBrightness.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinky_interfaces/srv/set_brightness.hpp"


#ifndef PINKY_INTERFACES__SRV__DETAIL__SET_BRIGHTNESS__BUILDER_HPP_
#define PINKY_INTERFACES__SRV__DETAIL__SET_BRIGHTNESS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pinky_interfaces/srv/detail/set_brightness__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pinky_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetBrightness_Request_brightness
{
public:
  Init_SetBrightness_Request_brightness()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pinky_interfaces::srv::SetBrightness_Request brightness(::pinky_interfaces::srv::SetBrightness_Request::_brightness_type arg)
  {
    msg_.brightness = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinky_interfaces::srv::SetBrightness_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinky_interfaces::srv::SetBrightness_Request>()
{
  return pinky_interfaces::srv::builder::Init_SetBrightness_Request_brightness();
}

}  // namespace pinky_interfaces


namespace pinky_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetBrightness_Response_message
{
public:
  explicit Init_SetBrightness_Response_message(::pinky_interfaces::srv::SetBrightness_Response & msg)
  : msg_(msg)
  {}
  ::pinky_interfaces::srv::SetBrightness_Response message(::pinky_interfaces::srv::SetBrightness_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinky_interfaces::srv::SetBrightness_Response msg_;
};

class Init_SetBrightness_Response_success
{
public:
  Init_SetBrightness_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetBrightness_Response_message success(::pinky_interfaces::srv::SetBrightness_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetBrightness_Response_message(msg_);
  }

private:
  ::pinky_interfaces::srv::SetBrightness_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinky_interfaces::srv::SetBrightness_Response>()
{
  return pinky_interfaces::srv::builder::Init_SetBrightness_Response_success();
}

}  // namespace pinky_interfaces


namespace pinky_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetBrightness_Event_response
{
public:
  explicit Init_SetBrightness_Event_response(::pinky_interfaces::srv::SetBrightness_Event & msg)
  : msg_(msg)
  {}
  ::pinky_interfaces::srv::SetBrightness_Event response(::pinky_interfaces::srv::SetBrightness_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinky_interfaces::srv::SetBrightness_Event msg_;
};

class Init_SetBrightness_Event_request
{
public:
  explicit Init_SetBrightness_Event_request(::pinky_interfaces::srv::SetBrightness_Event & msg)
  : msg_(msg)
  {}
  Init_SetBrightness_Event_response request(::pinky_interfaces::srv::SetBrightness_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetBrightness_Event_response(msg_);
  }

private:
  ::pinky_interfaces::srv::SetBrightness_Event msg_;
};

class Init_SetBrightness_Event_info
{
public:
  Init_SetBrightness_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetBrightness_Event_request info(::pinky_interfaces::srv::SetBrightness_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetBrightness_Event_request(msg_);
  }

private:
  ::pinky_interfaces::srv::SetBrightness_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinky_interfaces::srv::SetBrightness_Event>()
{
  return pinky_interfaces::srv::builder::Init_SetBrightness_Event_info();
}

}  // namespace pinky_interfaces

#endif  // PINKY_INTERFACES__SRV__DETAIL__SET_BRIGHTNESS__BUILDER_HPP_
