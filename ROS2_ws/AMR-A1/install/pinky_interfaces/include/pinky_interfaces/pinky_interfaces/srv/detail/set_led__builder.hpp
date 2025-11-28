// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pinky_interfaces:srv/SetLed.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinky_interfaces/srv/set_led.hpp"


#ifndef PINKY_INTERFACES__SRV__DETAIL__SET_LED__BUILDER_HPP_
#define PINKY_INTERFACES__SRV__DETAIL__SET_LED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pinky_interfaces/srv/detail/set_led__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pinky_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetLed_Request_b
{
public:
  explicit Init_SetLed_Request_b(::pinky_interfaces::srv::SetLed_Request & msg)
  : msg_(msg)
  {}
  ::pinky_interfaces::srv::SetLed_Request b(::pinky_interfaces::srv::SetLed_Request::_b_type arg)
  {
    msg_.b = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLed_Request msg_;
};

class Init_SetLed_Request_g
{
public:
  explicit Init_SetLed_Request_g(::pinky_interfaces::srv::SetLed_Request & msg)
  : msg_(msg)
  {}
  Init_SetLed_Request_b g(::pinky_interfaces::srv::SetLed_Request::_g_type arg)
  {
    msg_.g = std::move(arg);
    return Init_SetLed_Request_b(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLed_Request msg_;
};

class Init_SetLed_Request_r
{
public:
  explicit Init_SetLed_Request_r(::pinky_interfaces::srv::SetLed_Request & msg)
  : msg_(msg)
  {}
  Init_SetLed_Request_g r(::pinky_interfaces::srv::SetLed_Request::_r_type arg)
  {
    msg_.r = std::move(arg);
    return Init_SetLed_Request_g(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLed_Request msg_;
};

class Init_SetLed_Request_pixels
{
public:
  explicit Init_SetLed_Request_pixels(::pinky_interfaces::srv::SetLed_Request & msg)
  : msg_(msg)
  {}
  Init_SetLed_Request_r pixels(::pinky_interfaces::srv::SetLed_Request::_pixels_type arg)
  {
    msg_.pixels = std::move(arg);
    return Init_SetLed_Request_r(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLed_Request msg_;
};

class Init_SetLed_Request_command
{
public:
  Init_SetLed_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetLed_Request_pixels command(::pinky_interfaces::srv::SetLed_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_SetLed_Request_pixels(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLed_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinky_interfaces::srv::SetLed_Request>()
{
  return pinky_interfaces::srv::builder::Init_SetLed_Request_command();
}

}  // namespace pinky_interfaces


namespace pinky_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetLed_Response_message
{
public:
  explicit Init_SetLed_Response_message(::pinky_interfaces::srv::SetLed_Response & msg)
  : msg_(msg)
  {}
  ::pinky_interfaces::srv::SetLed_Response message(::pinky_interfaces::srv::SetLed_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLed_Response msg_;
};

class Init_SetLed_Response_success
{
public:
  Init_SetLed_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetLed_Response_message success(::pinky_interfaces::srv::SetLed_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetLed_Response_message(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLed_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinky_interfaces::srv::SetLed_Response>()
{
  return pinky_interfaces::srv::builder::Init_SetLed_Response_success();
}

}  // namespace pinky_interfaces


namespace pinky_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetLed_Event_response
{
public:
  explicit Init_SetLed_Event_response(::pinky_interfaces::srv::SetLed_Event & msg)
  : msg_(msg)
  {}
  ::pinky_interfaces::srv::SetLed_Event response(::pinky_interfaces::srv::SetLed_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLed_Event msg_;
};

class Init_SetLed_Event_request
{
public:
  explicit Init_SetLed_Event_request(::pinky_interfaces::srv::SetLed_Event & msg)
  : msg_(msg)
  {}
  Init_SetLed_Event_response request(::pinky_interfaces::srv::SetLed_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetLed_Event_response(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLed_Event msg_;
};

class Init_SetLed_Event_info
{
public:
  Init_SetLed_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetLed_Event_request info(::pinky_interfaces::srv::SetLed_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetLed_Event_request(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLed_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinky_interfaces::srv::SetLed_Event>()
{
  return pinky_interfaces::srv::builder::Init_SetLed_Event_info();
}

}  // namespace pinky_interfaces

#endif  // PINKY_INTERFACES__SRV__DETAIL__SET_LED__BUILDER_HPP_
