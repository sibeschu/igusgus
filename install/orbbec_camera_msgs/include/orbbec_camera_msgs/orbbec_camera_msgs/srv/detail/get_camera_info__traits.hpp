// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from orbbec_camera_msgs:srv/GetCameraInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "orbbec_camera_msgs/srv/get_camera_info.hpp"


#ifndef ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_CAMERA_INFO__TRAITS_HPP_
#define ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_CAMERA_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "orbbec_camera_msgs/srv/detail/get_camera_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace orbbec_camera_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetCameraInfo_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetCameraInfo_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetCameraInfo_Request & msg, bool use_flow_style = false)
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

}  // namespace orbbec_camera_msgs

namespace rosidl_generator_traits
{

[[deprecated("use orbbec_camera_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const orbbec_camera_msgs::srv::GetCameraInfo_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  orbbec_camera_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use orbbec_camera_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const orbbec_camera_msgs::srv::GetCameraInfo_Request & msg)
{
  return orbbec_camera_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<orbbec_camera_msgs::srv::GetCameraInfo_Request>()
{
  return "orbbec_camera_msgs::srv::GetCameraInfo_Request";
}

template<>
inline const char * name<orbbec_camera_msgs::srv::GetCameraInfo_Request>()
{
  return "orbbec_camera_msgs/srv/GetCameraInfo_Request";
}

template<>
struct has_fixed_size<orbbec_camera_msgs::srv::GetCameraInfo_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<orbbec_camera_msgs::srv::GetCameraInfo_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<orbbec_camera_msgs::srv::GetCameraInfo_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "sensor_msgs/msg/detail/camera_info__traits.hpp"

namespace orbbec_camera_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetCameraInfo_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetCameraInfo_Response & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetCameraInfo_Response & msg, bool use_flow_style = false)
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

}  // namespace orbbec_camera_msgs

namespace rosidl_generator_traits
{

[[deprecated("use orbbec_camera_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const orbbec_camera_msgs::srv::GetCameraInfo_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  orbbec_camera_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use orbbec_camera_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const orbbec_camera_msgs::srv::GetCameraInfo_Response & msg)
{
  return orbbec_camera_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<orbbec_camera_msgs::srv::GetCameraInfo_Response>()
{
  return "orbbec_camera_msgs::srv::GetCameraInfo_Response";
}

template<>
inline const char * name<orbbec_camera_msgs::srv::GetCameraInfo_Response>()
{
  return "orbbec_camera_msgs/srv/GetCameraInfo_Response";
}

template<>
struct has_fixed_size<orbbec_camera_msgs::srv::GetCameraInfo_Response>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::CameraInfo>::value> {};

template<>
struct has_bounded_size<orbbec_camera_msgs::srv::GetCameraInfo_Response>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::CameraInfo>::value> {};

template<>
struct is_message<orbbec_camera_msgs::srv::GetCameraInfo_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace orbbec_camera_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetCameraInfo_Event & msg,
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
  const GetCameraInfo_Event & msg,
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

inline std::string to_yaml(const GetCameraInfo_Event & msg, bool use_flow_style = false)
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

}  // namespace orbbec_camera_msgs

namespace rosidl_generator_traits
{

[[deprecated("use orbbec_camera_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const orbbec_camera_msgs::srv::GetCameraInfo_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  orbbec_camera_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use orbbec_camera_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const orbbec_camera_msgs::srv::GetCameraInfo_Event & msg)
{
  return orbbec_camera_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<orbbec_camera_msgs::srv::GetCameraInfo_Event>()
{
  return "orbbec_camera_msgs::srv::GetCameraInfo_Event";
}

template<>
inline const char * name<orbbec_camera_msgs::srv::GetCameraInfo_Event>()
{
  return "orbbec_camera_msgs/srv/GetCameraInfo_Event";
}

template<>
struct has_fixed_size<orbbec_camera_msgs::srv::GetCameraInfo_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<orbbec_camera_msgs::srv::GetCameraInfo_Event>
  : std::integral_constant<bool, has_bounded_size<orbbec_camera_msgs::srv::GetCameraInfo_Request>::value && has_bounded_size<orbbec_camera_msgs::srv::GetCameraInfo_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<orbbec_camera_msgs::srv::GetCameraInfo_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<orbbec_camera_msgs::srv::GetCameraInfo>()
{
  return "orbbec_camera_msgs::srv::GetCameraInfo";
}

template<>
inline const char * name<orbbec_camera_msgs::srv::GetCameraInfo>()
{
  return "orbbec_camera_msgs/srv/GetCameraInfo";
}

template<>
struct has_fixed_size<orbbec_camera_msgs::srv::GetCameraInfo>
  : std::integral_constant<
    bool,
    has_fixed_size<orbbec_camera_msgs::srv::GetCameraInfo_Request>::value &&
    has_fixed_size<orbbec_camera_msgs::srv::GetCameraInfo_Response>::value
  >
{
};

template<>
struct has_bounded_size<orbbec_camera_msgs::srv::GetCameraInfo>
  : std::integral_constant<
    bool,
    has_bounded_size<orbbec_camera_msgs::srv::GetCameraInfo_Request>::value &&
    has_bounded_size<orbbec_camera_msgs::srv::GetCameraInfo_Response>::value
  >
{
};

template<>
struct is_service<orbbec_camera_msgs::srv::GetCameraInfo>
  : std::true_type
{
};

template<>
struct is_service_request<orbbec_camera_msgs::srv::GetCameraInfo_Request>
  : std::true_type
{
};

template<>
struct is_service_response<orbbec_camera_msgs::srv::GetCameraInfo_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ORBBEC_CAMERA_MSGS__SRV__DETAIL__GET_CAMERA_INFO__TRAITS_HPP_
