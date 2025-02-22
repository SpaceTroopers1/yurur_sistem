// generated from rosidl_typesupport_microxrcedds_c/resource/idl__type_support_c.c.em
// with input from rover_msgs:msg/ControllerMsg.idl
// generated code does not contain a copyright notice
#include "rover_msgs/msg/detail/controller_msg__rosidl_typesupport_microxrcedds_c.h"


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "rosidl_typesupport_microxrcedds_c/identifier.h"
#include "rosidl_typesupport_microxrcedds_c/message_type_support.h"
#include "rover_msgs/msg/rosidl_typesupport_microxrcedds_c__visibility_control.h"
#include "rover_msgs/msg/detail/controller_msg__struct.h"
#include "rover_msgs/msg/detail/controller_msg__functions.h"

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

#define MICROXRCEDDS_PADDING sizeof(uint32_t)

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


typedef rover_msgs__msg__ControllerMsg _ControllerMsg__ros_msg_type;

static bool _ControllerMsg__cdr_serialize(
  const void * untyped_ros_message,
  ucdrBuffer * cdr)
{
  (void) untyped_ros_message;
  (void) cdr;

  bool rv = false;

  if (!untyped_ros_message) {
    return false;
  }

  _ControllerMsg__ros_msg_type * ros_message = (_ControllerMsg__ros_msg_type *)(untyped_ros_message);
  (void)ros_message;

  // Member: x
  rv = ucdr_serialize_float(cdr, ros_message->x);
  // Member: y
  rv = ucdr_serialize_float(cdr, ros_message->y);
  // Member: throttle
  rv = ucdr_serialize_float(cdr, ros_message->throttle);
  // Member: camerax
  rv = ucdr_serialize_int32_t(cdr, ros_message->camerax);
  // Member: cameray
  rv = ucdr_serialize_int32_t(cdr, ros_message->cameray);
  // Member: light
  rv = ucdr_serialize_int32_t(cdr, ros_message->light);
  // Member: gear
  rv = ucdr_serialize_int8_t(cdr, ros_message->gear);

  return rv;
}

static bool _ControllerMsg__cdr_deserialize(
  ucdrBuffer * cdr,
  void * untyped_ros_message)
{
  (void) cdr;

  bool rv = false;

  if (!untyped_ros_message) {
    return false;
  }
  _ControllerMsg__ros_msg_type * ros_message = (_ControllerMsg__ros_msg_type *)(untyped_ros_message);
  (void)ros_message;

  // Field name: x
  rv = ucdr_deserialize_float(cdr, &ros_message->x);
  // Field name: y
  rv = ucdr_deserialize_float(cdr, &ros_message->y);
  // Field name: throttle
  rv = ucdr_deserialize_float(cdr, &ros_message->throttle);
  // Field name: camerax
  rv = ucdr_deserialize_int32_t(cdr, &ros_message->camerax);
  // Field name: cameray
  rv = ucdr_deserialize_int32_t(cdr, &ros_message->cameray);
  // Field name: light
  rv = ucdr_deserialize_int32_t(cdr, &ros_message->light);
  // Field name: gear
  rv = ucdr_deserialize_int8_t(cdr, &ros_message->gear);
  return rv;
}

ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_PUBLIC_rover_msgs
size_t get_serialized_size_rover_msgs__msg__ControllerMsg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  if (!untyped_ros_message) {
    return 0;
  }

  const _ControllerMsg__ros_msg_type * ros_message = (const _ControllerMsg__ros_msg_type *)(untyped_ros_message);
  (void)ros_message;

  const size_t initial_alignment = current_alignment;

  // Member: x
  {
    const size_t item_size = sizeof(ros_message->x);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: y
  {
    const size_t item_size = sizeof(ros_message->y);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: throttle
  {
    const size_t item_size = sizeof(ros_message->throttle);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: camerax
  {
    const size_t item_size = sizeof(ros_message->camerax);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: cameray
  {
    const size_t item_size = sizeof(ros_message->cameray);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: light
  {
    const size_t item_size = sizeof(ros_message->light);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }
  // Member: gear
  {
    const size_t item_size = sizeof(ros_message->gear);
    current_alignment += ucdr_alignment(current_alignment, item_size) + item_size;
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ControllerMsg__get_serialized_size(const void * untyped_ros_message)
{
  return (uint32_t)(
    get_serialized_size_rover_msgs__msg__ControllerMsg(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_MICROXRCEDDS_C_PUBLIC_rover_msgs
size_t max_serialized_size_rover_msgs__msg__ControllerMsg(
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

static size_t _ControllerMsg__max_serialized_size()
{
  bool full_bounded;
  return max_serialized_size_rover_msgs__msg__ControllerMsg(&full_bounded, 0);
}

static message_type_support_callbacks_t __callbacks_ControllerMsg = {
  "rover_msgs::msg",
  "ControllerMsg",
  _ControllerMsg__cdr_serialize,
  _ControllerMsg__cdr_deserialize,
  _ControllerMsg__get_serialized_size,
  get_serialized_size_rover_msgs__msg__ControllerMsg,
  _ControllerMsg__max_serialized_size
};

static rosidl_message_type_support_t _ControllerMsg__type_support = {
  ROSIDL_TYPESUPPORT_MICROXRCEDDS_C__IDENTIFIER_VALUE,
  &__callbacks_ControllerMsg,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_microxrcedds_c, rover_msgs, msg, ControllerMsg)() {
  return &_ControllerMsg__type_support;
}

#if defined(__cplusplus)
}
#endif
