// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from orbbec_camera_msgs:srv/GetString.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "orbbec_camera_msgs/srv/get_string.hpp"


#ifndef ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_STRING__STRUCT_HPP_
#define ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_STRING__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__orbbec_camera_msgs__srv__GetString_Request __attribute__((deprecated))
#else
# define DEPRECATED__orbbec_camera_msgs__srv__GetString_Request __declspec(deprecated)
#endif

namespace orbbec_camera_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetString_Request_
{
  using Type = GetString_Request_<ContainerAllocator>;

  explicit GetString_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetString_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    orbbec_camera_msgs::srv::GetString_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const orbbec_camera_msgs::srv::GetString_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<orbbec_camera_msgs::srv::GetString_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<orbbec_camera_msgs::srv::GetString_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      orbbec_camera_msgs::srv::GetString_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<orbbec_camera_msgs::srv::GetString_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      orbbec_camera_msgs::srv::GetString_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<orbbec_camera_msgs::srv::GetString_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<orbbec_camera_msgs::srv::GetString_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<orbbec_camera_msgs::srv::GetString_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__orbbec_camera_msgs__srv__GetString_Request
    std::shared_ptr<orbbec_camera_msgs::srv::GetString_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__orbbec_camera_msgs__srv__GetString_Request
    std::shared_ptr<orbbec_camera_msgs::srv::GetString_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetString_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetString_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetString_Request_

// alias to use template instance with default allocator
using GetString_Request =
  orbbec_camera_msgs::srv::GetString_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace orbbec_camera_msgs


#ifndef _WIN32
# define DEPRECATED__orbbec_camera_msgs__srv__GetString_Response __attribute__((deprecated))
#else
# define DEPRECATED__orbbec_camera_msgs__srv__GetString_Response __declspec(deprecated)
#endif

namespace orbbec_camera_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetString_Response_
{
  using Type = GetString_Response_<ContainerAllocator>;

  explicit GetString_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
      this->success = false;
      this->message = "";
    }
  }

  explicit GetString_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_alloc),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _data_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _data_type data;
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__data(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->data = _arg;
    return *this;
  }
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
    orbbec_camera_msgs::srv::GetString_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const orbbec_camera_msgs::srv::GetString_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<orbbec_camera_msgs::srv::GetString_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<orbbec_camera_msgs::srv::GetString_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      orbbec_camera_msgs::srv::GetString_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<orbbec_camera_msgs::srv::GetString_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      orbbec_camera_msgs::srv::GetString_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<orbbec_camera_msgs::srv::GetString_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<orbbec_camera_msgs::srv::GetString_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<orbbec_camera_msgs::srv::GetString_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__orbbec_camera_msgs__srv__GetString_Response
    std::shared_ptr<orbbec_camera_msgs::srv::GetString_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__orbbec_camera_msgs__srv__GetString_Response
    std::shared_ptr<orbbec_camera_msgs::srv::GetString_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetString_Response_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetString_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetString_Response_

// alias to use template instance with default allocator
using GetString_Response =
  orbbec_camera_msgs::srv::GetString_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace orbbec_camera_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__orbbec_camera_msgs__srv__GetString_Event __attribute__((deprecated))
#else
# define DEPRECATED__orbbec_camera_msgs__srv__GetString_Event __declspec(deprecated)
#endif

namespace orbbec_camera_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetString_Event_
{
  using Type = GetString_Event_<ContainerAllocator>;

  explicit GetString_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit GetString_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<orbbec_camera_msgs::srv::GetString_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<orbbec_camera_msgs::srv::GetString_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<orbbec_camera_msgs::srv::GetString_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<orbbec_camera_msgs::srv::GetString_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<orbbec_camera_msgs::srv::GetString_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<orbbec_camera_msgs::srv::GetString_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<orbbec_camera_msgs::srv::GetString_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<orbbec_camera_msgs::srv::GetString_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    orbbec_camera_msgs::srv::GetString_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const orbbec_camera_msgs::srv::GetString_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<orbbec_camera_msgs::srv::GetString_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<orbbec_camera_msgs::srv::GetString_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      orbbec_camera_msgs::srv::GetString_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<orbbec_camera_msgs::srv::GetString_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      orbbec_camera_msgs::srv::GetString_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<orbbec_camera_msgs::srv::GetString_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<orbbec_camera_msgs::srv::GetString_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<orbbec_camera_msgs::srv::GetString_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__orbbec_camera_msgs__srv__GetString_Event
    std::shared_ptr<orbbec_camera_msgs::srv::GetString_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__orbbec_camera_msgs__srv__GetString_Event
    std::shared_ptr<orbbec_camera_msgs::srv::GetString_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetString_Event_ & other) const
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
  bool operator!=(const GetString_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetString_Event_

// alias to use template instance with default allocator
using GetString_Event =
  orbbec_camera_msgs::srv::GetString_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace orbbec_camera_msgs

namespace orbbec_camera_msgs
{

namespace srv
{

struct GetString
{
  using Request = orbbec_camera_msgs::srv::GetString_Request;
  using Response = orbbec_camera_msgs::srv::GetString_Response;
  using Event = orbbec_camera_msgs::srv::GetString_Event;
};

}  // namespace srv

}  // namespace orbbec_camera_msgs

#endif  // ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_STRING__STRUCT_HPP_
