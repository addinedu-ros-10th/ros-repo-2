// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pinky_interfaces:srv/SetLamp.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinky_interfaces/srv/set_lamp.hpp"


#ifndef PINKY_INTERFACES__SRV__DETAIL__SET_LAMP__TRAITS_HPP_
#define PINKY_INTERFACES__SRV__DETAIL__SET_LAMP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pinky_interfaces/srv/detail/set_lamp__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'color'
#include "std_msgs/msg/detail/color_rgba__traits.hpp"

namespace pinky_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetLamp_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: color
  {
    out << "color: ";
    to_flow_style_yaml(msg.color, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: time
  {
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetLamp_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "color:\n";
    to_block_style_yaml(msg.color, out, indentation + 2);
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetLamp_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace pinky_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use pinky_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pinky_interfaces::srv::SetLamp_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  pinky_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pinky_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const pinky_interfaces::srv::SetLamp_Request & msg)
{
  return pinky_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pinky_interfaces::srv::SetLamp_Request>()
{
  return "pinky_interfaces::srv::SetLamp_Request";
}

template<>
inline const char * name<pinky_interfaces::srv::SetLamp_Request>()
{
  return "pinky_interfaces/srv/SetLamp_Request";
}

template<>
struct has_fixed_size<pinky_interfaces::srv::SetLamp_Request>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::ColorRGBA>::value> {};

template<>
struct has_bounded_size<pinky_interfaces::srv::SetLamp_Request>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::ColorRGBA>::value> {};

template<>
struct is_message<pinky_interfaces::srv::SetLamp_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace pinky_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetLamp_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetLamp_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetLamp_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace pinky_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use pinky_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pinky_interfaces::srv::SetLamp_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  pinky_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pinky_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const pinky_interfaces::srv::SetLamp_Response & msg)
{
  return pinky_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pinky_interfaces::srv::SetLamp_Response>()
{
  return "pinky_interfaces::srv::SetLamp_Response";
}

template<>
inline const char * name<pinky_interfaces::srv::SetLamp_Response>()
{
  return "pinky_interfaces/srv/SetLamp_Response";
}

template<>
struct has_fixed_size<pinky_interfaces::srv::SetLamp_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pinky_interfaces::srv::SetLamp_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pinky_interfaces::srv::SetLamp_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace pinky_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetLamp_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetLamp_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetLamp_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace pinky_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use pinky_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pinky_interfaces::srv::SetLamp_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  pinky_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pinky_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const pinky_interfaces::srv::SetLamp_Event & msg)
{
  return pinky_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pinky_interfaces::srv::SetLamp_Event>()
{
  return "pinky_interfaces::srv::SetLamp_Event";
}

template<>
inline const char * name<pinky_interfaces::srv::SetLamp_Event>()
{
  return "pinky_interfaces/srv/SetLamp_Event";
}

template<>
struct has_fixed_size<pinky_interfaces::srv::SetLamp_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pinky_interfaces::srv::SetLamp_Event>
  : std::integral_constant<bool, has_bounded_size<pinky_interfaces::srv::SetLamp_Request>::value && has_bounded_size<pinky_interfaces::srv::SetLamp_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<pinky_interfaces::srv::SetLamp_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pinky_interfaces::srv::SetLamp>()
{
  return "pinky_interfaces::srv::SetLamp";
}

template<>
inline const char * name<pinky_interfaces::srv::SetLamp>()
{
  return "pinky_interfaces/srv/SetLamp";
}

template<>
struct has_fixed_size<pinky_interfaces::srv::SetLamp>
  : std::integral_constant<
    bool,
    has_fixed_size<pinky_interfaces::srv::SetLamp_Request>::value &&
    has_fixed_size<pinky_interfaces::srv::SetLamp_Response>::value
  >
{
};

template<>
struct has_bounded_size<pinky_interfaces::srv::SetLamp>
  : std::integral_constant<
    bool,
    has_bounded_size<pinky_interfaces::srv::SetLamp_Request>::value &&
    has_bounded_size<pinky_interfaces::srv::SetLamp_Response>::value
  >
{
};

template<>
struct is_service<pinky_interfaces::srv::SetLamp>
  : std::true_type
{
};

template<>
struct is_service_request<pinky_interfaces::srv::SetLamp_Request>
  : std::true_type
{
};

template<>
struct is_service_response<pinky_interfaces::srv::SetLamp_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PINKY_INTERFACES__SRV__DETAIL__SET_LAMP__TRAITS_HPP_
