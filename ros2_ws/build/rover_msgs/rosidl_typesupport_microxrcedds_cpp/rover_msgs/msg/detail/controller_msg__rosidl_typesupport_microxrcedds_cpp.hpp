// generated from rosidl_typesupport_microxrcedds_cpp/resource/idl__rosidl_typesupport_microxrcedds_cpp.hpp.em
// with input from rover_msgs:msg/ControllerMsg.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__CONTROLLER_MSG__ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_HPP_
#define ROVER_MSGS__MSG__CONTROLLER_MSG__ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "rover_msgs/msg/rosidl_typesupport_microxrcedds_cpp__visibility_control.h"
#include "rover_msgs/msg/detail/controller_msg__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "ucdr/microcdr.h"

namespace rover_msgs
{

namespace msg
{

namespace typesupport_microxrcedds_cpp
{

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_rover_msgs
cdr_serialize(
  const rover_msgs::msg::ControllerMsg & ros_message,
  ucdrBuffer * cdr);

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_rover_msgs
cdr_deserialize(
  ucdrBuffer * cdr,
  rover_msgs::msg::ControllerMsg & ros_message);

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_rover_msgs
get_serialized_size(
  const rover_msgs::msg::ControllerMsg & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_rover_msgs
max_serialized_size_ControllerMsg(
  bool * full_bounded,
  size_t current_alignment);

}  // namespace typesupport_microxrcedds_cpp

}  // namespace msg

}  // namespace rover_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_rover_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_cpp, rover_msgs, msg, ControllerMsg)();

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__CONTROLLER_MSG__ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_HPP_
