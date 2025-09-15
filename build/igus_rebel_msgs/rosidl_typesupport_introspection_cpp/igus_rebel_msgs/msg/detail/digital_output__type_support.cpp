// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from igus_rebel_msgs:msg/DigitalOutput.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "igus_rebel_msgs/msg/detail/digital_output__functions.h"
#include "igus_rebel_msgs/msg/detail/digital_output__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace igus_rebel_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void DigitalOutput_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) igus_rebel_msgs::msg::DigitalOutput(_init);
}

void DigitalOutput_fini_function(void * message_memory)
{
  auto typed_message = static_cast<igus_rebel_msgs::msg::DigitalOutput *>(message_memory);
  typed_message->~DigitalOutput();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DigitalOutput_message_member_array[2] = {
  {
    "output",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(igus_rebel_msgs::msg::DigitalOutput, output),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "is_on",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(igus_rebel_msgs::msg::DigitalOutput, is_on),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DigitalOutput_message_members = {
  "igus_rebel_msgs::msg",  // message namespace
  "DigitalOutput",  // message name
  2,  // number of fields
  sizeof(igus_rebel_msgs::msg::DigitalOutput),
  false,  // has_any_key_member_
  DigitalOutput_message_member_array,  // message members
  DigitalOutput_init_function,  // function to initialize message memory (memory has to be allocated)
  DigitalOutput_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DigitalOutput_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DigitalOutput_message_members,
  get_message_typesupport_handle_function,
  &igus_rebel_msgs__msg__DigitalOutput__get_type_hash,
  &igus_rebel_msgs__msg__DigitalOutput__get_type_description,
  &igus_rebel_msgs__msg__DigitalOutput__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace igus_rebel_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<igus_rebel_msgs::msg::DigitalOutput>()
{
  return &::igus_rebel_msgs::msg::rosidl_typesupport_introspection_cpp::DigitalOutput_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, igus_rebel_msgs, msg, DigitalOutput)() {
  return &::igus_rebel_msgs::msg::rosidl_typesupport_introspection_cpp::DigitalOutput_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
