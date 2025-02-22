// generated from rosidl_typesupport_microxrcedds_cpp/resource/idl__type_support.cpp.em
// with input from rover_msgs:msg/ControllerMsg.idl
// generated code does not contain a copyright notice
#include "rover_msgs/msg/detail/controller_msg__rosidl_typesupport_microxrcedds_cpp.hpp"
#include "rover_msgs/msg/detail/controller_msg__struct.hpp"

#include <limits>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <cstring>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_microxrcedds_cpp/identifier.hpp"
#include "rosidl_typesupport_microxrcedds_c/message_type_support.h"
#include "rosidl_typesupport_microxrcedds_cpp/message_type_support_decl.hpp"
#include "ucdr/microcdr.h"

#define MICROXRCEDDS_PADDING sizeof(uint32_t)

// forward declaration of message dependencies and their conversion functions

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
  ucdrBuffer * cdr)
{
  (void) ros_message;
  (void) cdr;
  bool rv = false;

  // Member: x
  rv = ucdr_serialize_float(cdr, ros_message.x);
  // Member: y
  rv = ucdr_serialize_float(cdr, ros_message.y);
  // Member: throttle
  rv = ucdr_serialize_float(cdr, ros_message.throttle);
  // Member: camerax
  rv = ucdr_serialize_int32_t(cdr, ros_message.camerax);
  // Member: cameray
  rv = ucdr_serialize_int32_t(cdr, ros_message.cameray);
  // Member: light
  rv = ucdr_serialize_int32_t(cdr, ros_message.light);
  // Member: gear
  rv = ucdr_serialize_int8_t(cdr, ros_message.gear);

  return rv;
}

bool
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_rover_msgs
cdr_deserialize(
  ucdrBuffer * cdr,
  rover_msgs::msg::ControllerMsg & ros_message)
{
  (void) cdr;
  (void) ros_message;
  bool rv = false;

  // Member: x
  rv = ucdr_deserialize_float(cdr, &ros_message.x);
  // Member: y
  rv = ucdr_deserialize_float(cdr, &ros_message.y);
  // Member: throttle
  rv = ucdr_deserialize_float(cdr, &ros_message.throttle);
  // Member: camerax
  rv = ucdr_deserialize_int32_t(cdr, &ros_message.camerax);
  // Member: cameray
  rv = ucdr_deserialize_int32_t(cdr, &ros_message.cameray);
  // Member: light
  rv = ucdr_deserialize_int32_t(cdr, &ros_message.light);
  // Member: gear
  rv = ucdr_deserialize_int8_t(cdr, &ros_message.gear);

  return rv;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_rover_msgs
get_serialized_size(
  const rover_msgs::msg::ControllerMsg & ros_message,
  size_t current_alignment)
{
  (void) current_alignment;
  (void) ros_message;

  const size_t initial_alignment = current_alignment;

  // Member: x
  {
    const size_t item_size = sizeof(ros_message.x);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: y
  {
    const size_t item_size = sizeof(ros_message.y);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: throttle
  {
    const size_t item_size = sizeof(ros_message.throttle);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: camerax
  {
    const size_t item_size = sizeof(ros_message.camerax);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: cameray
  {
    const size_t item_size = sizeof(ros_message.cameray);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: light
  {
    const size_t item_size = sizeof(ros_message.light);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: gear
  {
    const size_t item_size = sizeof(ros_message.gear);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_PUBLIC_rover_msgs
max_serialized_size_ControllerMsg(
  bool * full_bounded,
  size_t current_alignment)
{
  (void) current_alignment;
  *full_bounded = true;

  const size_t initial_alignment = current_alignment;

  // Member: x
  current_alignment += ucdr_alignment(current_alignment, sizeof(float)) + sizeof(float);
  // Member: y
  current_alignment += ucdr_alignment(current_alignment, sizeof(float)) + sizeof(float);
  // Member: throttle
  current_alignment += ucdr_alignment(current_alignment, sizeof(float)) + sizeof(float);
  // Member: camerax
  current_alignment += ucdr_alignment(current_alignment, sizeof(int32_t)) + sizeof(int32_t);
  // Member: cameray
  current_alignment += ucdr_alignment(current_alignment, sizeof(int32_t)) + sizeof(int32_t);
  // Member: light
  current_alignment += ucdr_alignment(current_alignment, sizeof(int32_t)) + sizeof(int32_t);
  // Member: gear
  current_alignment += ucdr_alignment(current_alignment, sizeof(int8_t)) + sizeof(int8_t);

  return current_alignment - initial_alignment;
}

static bool _ControllerMsg__cdr_serialize(
  const void * untyped_ros_message,
  ucdrBuffer * cdr)
{
  auto typed_message =
    static_cast<const rover_msgs::msg::ControllerMsg *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ControllerMsg__cdr_deserialize(
  ucdrBuffer * cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<rover_msgs::msg::ControllerMsg *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ControllerMsg__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const rover_msgs::msg::ControllerMsg *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ControllerMsg__get_serialized_size_with_initial_alignment(
  const void * untyped_ros_message, size_t current_alignment)
{
  auto typed_message =
    static_cast<const rover_msgs::msg::ControllerMsg *>(
    untyped_ros_message);
  return static_cast<size_t>(get_serialized_size(*typed_message, current_alignment));
}

static size_t _ControllerMsg__max_serialized_size()
{
  bool full_bounded;
  return max_serialized_size_ControllerMsg(&full_bounded, 0);
}

static message_type_support_callbacks_t _ControllerMsg__callbacks = {
  "rover_msgs::msg",
  "ControllerMsg",
  _ControllerMsg__cdr_serialize,
  _ControllerMsg__cdr_deserialize,
  _ControllerMsg__get_serialized_size,
  _ControllerMsg__get_serialized_size_with_initial_alignment,
  _ControllerMsg__max_serialized_size
};

static rosidl_message_type_support_t _ControllerMsg__handle = {
  rosidl_typesupport_microxrcedds_cpp::typesupport_identifier,
  &_ControllerMsg__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_microxrcedds_cpp

}  // namespace msg

}  // namespace rover_msgs

namespace rosidl_typesupport_microxrcedds_cpp
{

template<>
ROSIDL_TYPESUPPORT_MICROXRCEDDS_CPP_EXPORT_rover_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<rover_msgs::msg::ControllerMsg>()
{
  return &rover_msgs::msg::typesupport_microxrcedds_cpp::_ControllerMsg__handle;
}

}  // namespace rosidl_typesupport_microxrcedds_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_cpp, rover_msgs, msg, ControllerMsg)() {
  return &rover_msgs::msg::typesupport_microxrcedds_cpp::_ControllerMsg__handle;
}

#ifdef __cplusplus
}
#endif
