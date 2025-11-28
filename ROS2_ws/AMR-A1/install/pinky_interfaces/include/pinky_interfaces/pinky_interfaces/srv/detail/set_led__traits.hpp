// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pinky_interfaces:srv/SetLed.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinky_interfaces/srv/set_led.hpp"


#ifndef PINKY_INTERFACES__SRV__DETAIL__SET_LED__TRAITS_HPP_
#define PINKY_INTERFACES__SRV__DETAIL__SET_LED__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pinky_interfaces/srv/detail/set_led__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pinky_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetLed_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: pixels
  {
    if (msg.pixels.size() == 0) {
      out << "pixels: []";
    } else {
      out << "pixels: [";
      size_t pending_items = msg.pixels.size();
      for (auto item : msg.pixels) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: r
  {
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << ", ";
  }

  // member: g
  {
    out << "g: ";
    rosidl_generator_traits::value_to_yaml(msg.g, out);
    out << ", ";
  }

  // member: b
  {
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetLed_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: pixels
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pixels.size() == 0) {
      out << "pixels: []\n";
    } else {
      out << "pixels:\n";
      for (auto item : msg.pixels) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << "\n";
  }

  // member: g
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "g: ";
    rosidl_generator_traits::value_to_yaml(msg.g, out);
    out << "\n";
  }

  // member: b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetLed_Request & msg, bool use_flow_style = false)
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
  const pinky_interfaces::srv::SetLed_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  pinky_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pinky_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const pinky_interfaces::srv::SetLed_Request & msg)
{
  return pinky_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pinky_interfaces::srv::SetLed_Request>()
{
  return "pinky_interfaces::srv::SetLed_Request";
}

template<>
inline const char * name<pinky_interfaces::srv::SetLed_Request>()
{
  return "pinky_interfaces/srv/SetLed_Request";
}

template<>
struct has_fixed_size<pinky_interfaces::srv::SetLed_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pinky_interfaces::srv::SetLed_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pinky_interfaces::srv::SetLed_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace pinky_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetLed_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetLed_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetLed_Response & msg, bool use_flow_style = false)
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
  const pinky_interfaces::srv::SetLed_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  pinky_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pinky_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const pinky_interfaces::srv::SetLed_Response & msg)
{
  return pinky_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pinky_interfaces::srv::SetLed_Response>()
{
  return "pinky_interfaces::srv::SetLed_Response";
}

template<>
inline const char * name<pinky_interfaces::srv::SetLed_Response>()
{
  return "pinky_interfaces/srv/SetLed_Response";
}

template<>
struct has_fixed_size<pinky_interfaces::srv::SetLed_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pinky_interfaces::srv::SetLed_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pinky_interfaces::srv::SetLed_Response>
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
  const SetLed_Event & msg,
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
  const SetLed_Event & msg,
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

inline std::string to_yaml(const SetLed_Event & msg, bool use_flow_style = false)
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
  const pinky_interfaces::srv::SetLed_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  pinky_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pinky_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const pinky_interfaces::srv::SetLed_Event & msg)
{
  return pinky_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pinky_interfaces::srv::SetLed_Event>()
{
  return "pinky_interfaces::srv::SetLed_Event";
}

template<>
inline const char * name<pinky_interfaces::srv::SetLed_Event>()
{
  return "pinky_interfaces/srv/SetLed_Event";
}

template<>
struct has_fixed_size<pinky_interfaces::srv::SetLed_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pinky_interfaces::srv::SetLed_Event>
  : std::integral_constant<bool, has_bounded_size<pinky_interfaces::srv::SetLed_Request>::value && has_bounded_size<pinky_interfaces::srv::SetLed_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<pinky_interfaces::srv::SetLed_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pinky_interfaces::srv::SetLed>()
{
  return "pinky_interfaces::srv::SetLed";
}

template<>
inline const char * name<pinky_interfaces::srv::SetLed>()
{
  return "pinky_interfaces/srv/SetLed";
}

template<>
struct has_fixed_size<pinky_interfaces::srv::SetLed>
  : std::integral_constant<
    bool,
    has_fixed_size<pinky_interfaces::srv::SetLed_Request>::value &&
    has_fixed_size<pinky_interfaces::srv::SetLed_Response>::value
  >
{
};

template<>
struct has_bounded_size<pinky_interfaces::srv::SetLed>
  : std::integral_constant<
    bool,
    has_bounded_size<pinky_interfaces::srv::SetLed_Request>::value &&
    has_bounded_size<pinky_interfaces::srv::SetLed_Response>::value
  >
{
};

template<>
struct is_service<pinky_interfaces::srv::SetLed>
  : std::true_type
{
};

template<>
struct is_service_request<pinky_interfaces::srv::SetLed_Request>
  : std::true_type
{
};

template<>
struct is_service_response<pinky_interfaces::srv::SetLed_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PINKY_INTERFACES__SRV__DETAIL__SET_LED__TRAITS_HPP_
