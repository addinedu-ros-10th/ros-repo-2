// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pinky_interfaces:srv/SetLamp.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinky_interfaces/srv/set_lamp.hpp"


#ifndef PINKY_INTERFACES__SRV__DETAIL__SET_LAMP__STRUCT_HPP_
#define PINKY_INTERFACES__SRV__DETAIL__SET_LAMP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'color'
#include "std_msgs/msg/detail/color_rgba__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pinky_interfaces__srv__SetLamp_Request __attribute__((deprecated))
#else
# define DEPRECATED__pinky_interfaces__srv__SetLamp_Request __declspec(deprecated)
#endif

namespace pinky_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLamp_Request_
{
  using Type = SetLamp_Request_<ContainerAllocator>;

  explicit SetLamp_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : color(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->time = 0;
    }
  }

  explicit SetLamp_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : color(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->time = 0;
    }
  }

  // field types and members
  using _color_type =
    std_msgs::msg::ColorRGBA_<ContainerAllocator>;
  _color_type color;
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _time_type =
    uint16_t;
  _time_type time;

  // setters for named parameter idiom
  Type & set__color(
    const std_msgs::msg::ColorRGBA_<ContainerAllocator> & _arg)
  {
    this->color = _arg;
    return *this;
  }
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__time(
    const uint16_t & _arg)
  {
    this->time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinky_interfaces::srv::SetLamp_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinky_interfaces::srv::SetLamp_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinky_interfaces::srv::SetLamp_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinky_interfaces::srv::SetLamp_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinky_interfaces::srv::SetLamp_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinky_interfaces::srv::SetLamp_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinky_interfaces::srv::SetLamp_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinky_interfaces::srv::SetLamp_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinky_interfaces::srv::SetLamp_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinky_interfaces::srv::SetLamp_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinky_interfaces__srv__SetLamp_Request
    std::shared_ptr<pinky_interfaces::srv::SetLamp_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinky_interfaces__srv__SetLamp_Request
    std::shared_ptr<pinky_interfaces::srv::SetLamp_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLamp_Request_ & other) const
  {
    if (this->color != other.color) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->time != other.time) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetLamp_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLamp_Request_

// alias to use template instance with default allocator
using SetLamp_Request =
  pinky_interfaces::srv::SetLamp_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pinky_interfaces


#ifndef _WIN32
# define DEPRECATED__pinky_interfaces__srv__SetLamp_Response __attribute__((deprecated))
#else
# define DEPRECATED__pinky_interfaces__srv__SetLamp_Response __declspec(deprecated)
#endif

namespace pinky_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLamp_Response_
{
  using Type = SetLamp_Response_<ContainerAllocator>;

  explicit SetLamp_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  explicit SetLamp_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  // field types and members
  using _result_type =
    bool;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const bool & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinky_interfaces::srv::SetLamp_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinky_interfaces::srv::SetLamp_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinky_interfaces::srv::SetLamp_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinky_interfaces::srv::SetLamp_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinky_interfaces::srv::SetLamp_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinky_interfaces::srv::SetLamp_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinky_interfaces::srv::SetLamp_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinky_interfaces::srv::SetLamp_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinky_interfaces::srv::SetLamp_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinky_interfaces::srv::SetLamp_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinky_interfaces__srv__SetLamp_Response
    std::shared_ptr<pinky_interfaces::srv::SetLamp_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinky_interfaces__srv__SetLamp_Response
    std::shared_ptr<pinky_interfaces::srv::SetLamp_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLamp_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetLamp_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLamp_Response_

// alias to use template instance with default allocator
using SetLamp_Response =
  pinky_interfaces::srv::SetLamp_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pinky_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pinky_interfaces__srv__SetLamp_Event __attribute__((deprecated))
#else
# define DEPRECATED__pinky_interfaces__srv__SetLamp_Event __declspec(deprecated)
#endif

namespace pinky_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLamp_Event_
{
  using Type = SetLamp_Event_<ContainerAllocator>;

  explicit SetLamp_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit SetLamp_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<pinky_interfaces::srv::SetLamp_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinky_interfaces::srv::SetLamp_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<pinky_interfaces::srv::SetLamp_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinky_interfaces::srv::SetLamp_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<pinky_interfaces::srv::SetLamp_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinky_interfaces::srv::SetLamp_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<pinky_interfaces::srv::SetLamp_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinky_interfaces::srv::SetLamp_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinky_interfaces::srv::SetLamp_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinky_interfaces::srv::SetLamp_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinky_interfaces::srv::SetLamp_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinky_interfaces::srv::SetLamp_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinky_interfaces::srv::SetLamp_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinky_interfaces::srv::SetLamp_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinky_interfaces::srv::SetLamp_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinky_interfaces::srv::SetLamp_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinky_interfaces::srv::SetLamp_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinky_interfaces::srv::SetLamp_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinky_interfaces__srv__SetLamp_Event
    std::shared_ptr<pinky_interfaces::srv::SetLamp_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinky_interfaces__srv__SetLamp_Event
    std::shared_ptr<pinky_interfaces::srv::SetLamp_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLamp_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetLamp_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLamp_Event_

// alias to use template instance with default allocator
using SetLamp_Event =
  pinky_interfaces::srv::SetLamp_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pinky_interfaces

namespace pinky_interfaces
{

namespace srv
{

struct SetLamp
{
  using Request = pinky_interfaces::srv::SetLamp_Request;
  using Response = pinky_interfaces::srv::SetLamp_Response;
  using Event = pinky_interfaces::srv::SetLamp_Event;
};

}  // namespace srv

}  // namespace pinky_interfaces

#endif  // PINKY_INTERFACES__SRV__DETAIL__SET_LAMP__STRUCT_HPP_
