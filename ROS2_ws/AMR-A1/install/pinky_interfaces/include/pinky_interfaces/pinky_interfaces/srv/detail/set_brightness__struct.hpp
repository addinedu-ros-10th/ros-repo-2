// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pinky_interfaces:srv/SetBrightness.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "pinky_interfaces/srv/set_brightness.hpp"


#ifndef PINKY_INTERFACES__SRV__DETAIL__SET_BRIGHTNESS__STRUCT_HPP_
#define PINKY_INTERFACES__SRV__DETAIL__SET_BRIGHTNESS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pinky_interfaces__srv__SetBrightness_Request __attribute__((deprecated))
#else
# define DEPRECATED__pinky_interfaces__srv__SetBrightness_Request __declspec(deprecated)
#endif

namespace pinky_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetBrightness_Request_
{
  using Type = SetBrightness_Request_<ContainerAllocator>;

  explicit SetBrightness_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->brightness = 0l;
    }
  }

  explicit SetBrightness_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->brightness = 0l;
    }
  }

  // field types and members
  using _brightness_type =
    int32_t;
  _brightness_type brightness;

  // setters for named parameter idiom
  Type & set__brightness(
    const int32_t & _arg)
  {
    this->brightness = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinky_interfaces::srv::SetBrightness_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinky_interfaces::srv::SetBrightness_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinky_interfaces::srv::SetBrightness_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinky_interfaces::srv::SetBrightness_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinky_interfaces::srv::SetBrightness_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinky_interfaces::srv::SetBrightness_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinky_interfaces::srv::SetBrightness_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinky_interfaces::srv::SetBrightness_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinky_interfaces::srv::SetBrightness_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinky_interfaces::srv::SetBrightness_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinky_interfaces__srv__SetBrightness_Request
    std::shared_ptr<pinky_interfaces::srv::SetBrightness_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinky_interfaces__srv__SetBrightness_Request
    std::shared_ptr<pinky_interfaces::srv::SetBrightness_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetBrightness_Request_ & other) const
  {
    if (this->brightness != other.brightness) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetBrightness_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetBrightness_Request_

// alias to use template instance with default allocator
using SetBrightness_Request =
  pinky_interfaces::srv::SetBrightness_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pinky_interfaces


#ifndef _WIN32
# define DEPRECATED__pinky_interfaces__srv__SetBrightness_Response __attribute__((deprecated))
#else
# define DEPRECATED__pinky_interfaces__srv__SetBrightness_Response __declspec(deprecated)
#endif

namespace pinky_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetBrightness_Response_
{
  using Type = SetBrightness_Response_<ContainerAllocator>;

  explicit SetBrightness_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetBrightness_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinky_interfaces::srv::SetBrightness_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinky_interfaces::srv::SetBrightness_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinky_interfaces::srv::SetBrightness_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinky_interfaces::srv::SetBrightness_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinky_interfaces::srv::SetBrightness_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinky_interfaces::srv::SetBrightness_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinky_interfaces::srv::SetBrightness_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinky_interfaces::srv::SetBrightness_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinky_interfaces::srv::SetBrightness_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinky_interfaces::srv::SetBrightness_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinky_interfaces__srv__SetBrightness_Response
    std::shared_ptr<pinky_interfaces::srv::SetBrightness_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinky_interfaces__srv__SetBrightness_Response
    std::shared_ptr<pinky_interfaces::srv::SetBrightness_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetBrightness_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetBrightness_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetBrightness_Response_

// alias to use template instance with default allocator
using SetBrightness_Response =
  pinky_interfaces::srv::SetBrightness_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pinky_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__pinky_interfaces__srv__SetBrightness_Event __attribute__((deprecated))
#else
# define DEPRECATED__pinky_interfaces__srv__SetBrightness_Event __declspec(deprecated)
#endif

namespace pinky_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetBrightness_Event_
{
  using Type = SetBrightness_Event_<ContainerAllocator>;

  explicit SetBrightness_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit SetBrightness_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<pinky_interfaces::srv::SetBrightness_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinky_interfaces::srv::SetBrightness_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<pinky_interfaces::srv::SetBrightness_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinky_interfaces::srv::SetBrightness_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<pinky_interfaces::srv::SetBrightness_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinky_interfaces::srv::SetBrightness_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<pinky_interfaces::srv::SetBrightness_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<pinky_interfaces::srv::SetBrightness_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pinky_interfaces::srv::SetBrightness_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const pinky_interfaces::srv::SetBrightness_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pinky_interfaces::srv::SetBrightness_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pinky_interfaces::srv::SetBrightness_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pinky_interfaces::srv::SetBrightness_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pinky_interfaces::srv::SetBrightness_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pinky_interfaces::srv::SetBrightness_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pinky_interfaces::srv::SetBrightness_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pinky_interfaces::srv::SetBrightness_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pinky_interfaces::srv::SetBrightness_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pinky_interfaces__srv__SetBrightness_Event
    std::shared_ptr<pinky_interfaces::srv::SetBrightness_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pinky_interfaces__srv__SetBrightness_Event
    std::shared_ptr<pinky_interfaces::srv::SetBrightness_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetBrightness_Event_ & other) const
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
  bool operator!=(const SetBrightness_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetBrightness_Event_

// alias to use template instance with default allocator
using SetBrightness_Event =
  pinky_interfaces::srv::SetBrightness_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pinky_interfaces

namespace pinky_interfaces
{

namespace srv
{

struct SetBrightness
{
  using Request = pinky_interfaces::srv::SetBrightness_Request;
  using Response = pinky_interfaces::srv::SetBrightness_Response;
  using Event = pinky_interfaces::srv::SetBrightness_Event;
};

}  // namespace srv

}  // namespace pinky_interfaces

#endif  // PINKY_INTERFACES__SRV__DETAIL__SET_BRIGHTNESS__STRUCT_HPP_
