// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pinky_interfaces:srv/SetLamp.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinky_interfaces/srv/set_lamp.hpp"


#ifndef PINKY_INTERFACES__SRV__DETAIL__SET_LAMP__BUILDER_HPP_
#define PINKY_INTERFACES__SRV__DETAIL__SET_LAMP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pinky_interfaces/srv/detail/set_lamp__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pinky_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetLamp_Request_time
{
public:
  explicit Init_SetLamp_Request_time(::pinky_interfaces::srv::SetLamp_Request & msg)
  : msg_(msg)
  {}
  ::pinky_interfaces::srv::SetLamp_Request time(::pinky_interfaces::srv::SetLamp_Request::_time_type arg)
  {
    msg_.time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLamp_Request msg_;
};

class Init_SetLamp_Request_mode
{
public:
  explicit Init_SetLamp_Request_mode(::pinky_interfaces::srv::SetLamp_Request & msg)
  : msg_(msg)
  {}
  Init_SetLamp_Request_time mode(::pinky_interfaces::srv::SetLamp_Request::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_SetLamp_Request_time(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLamp_Request msg_;
};

class Init_SetLamp_Request_color
{
public:
  Init_SetLamp_Request_color()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetLamp_Request_mode color(::pinky_interfaces::srv::SetLamp_Request::_color_type arg)
  {
    msg_.color = std::move(arg);
    return Init_SetLamp_Request_mode(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLamp_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinky_interfaces::srv::SetLamp_Request>()
{
  return pinky_interfaces::srv::builder::Init_SetLamp_Request_color();
}

}  // namespace pinky_interfaces


namespace pinky_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetLamp_Response_result
{
public:
  Init_SetLamp_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pinky_interfaces::srv::SetLamp_Response result(::pinky_interfaces::srv::SetLamp_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLamp_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinky_interfaces::srv::SetLamp_Response>()
{
  return pinky_interfaces::srv::builder::Init_SetLamp_Response_result();
}

}  // namespace pinky_interfaces


namespace pinky_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetLamp_Event_response
{
public:
  explicit Init_SetLamp_Event_response(::pinky_interfaces::srv::SetLamp_Event & msg)
  : msg_(msg)
  {}
  ::pinky_interfaces::srv::SetLamp_Event response(::pinky_interfaces::srv::SetLamp_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLamp_Event msg_;
};

class Init_SetLamp_Event_request
{
public:
  explicit Init_SetLamp_Event_request(::pinky_interfaces::srv::SetLamp_Event & msg)
  : msg_(msg)
  {}
  Init_SetLamp_Event_response request(::pinky_interfaces::srv::SetLamp_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetLamp_Event_response(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLamp_Event msg_;
};

class Init_SetLamp_Event_info
{
public:
  Init_SetLamp_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetLamp_Event_request info(::pinky_interfaces::srv::SetLamp_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetLamp_Event_request(msg_);
  }

private:
  ::pinky_interfaces::srv::SetLamp_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pinky_interfaces::srv::SetLamp_Event>()
{
  return pinky_interfaces::srv::builder::Init_SetLamp_Event_info();
}

}  // namespace pinky_interfaces

#endif  // PINKY_INTERFACES__SRV__DETAIL__SET_LAMP__BUILDER_HPP_
