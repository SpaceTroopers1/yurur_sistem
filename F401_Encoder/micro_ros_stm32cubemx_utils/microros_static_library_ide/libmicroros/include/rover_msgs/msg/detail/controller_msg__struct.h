// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:msg/ControllerMsg.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__CONTROLLER_MSG__STRUCT_H_
#define ROVER_MSGS__MSG__DETAIL__CONTROLLER_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/ControllerMsg in the package rover_msgs.
typedef struct rover_msgs__msg__ControllerMsg
{
  float x;
  float y;
  float throttle;
  int32_t camerax;
  int32_t cameray;
  int32_t light;
  uint8_t gear;
} rover_msgs__msg__ControllerMsg;

// Struct for a sequence of rover_msgs__msg__ControllerMsg.
typedef struct rover_msgs__msg__ControllerMsg__Sequence
{
  rover_msgs__msg__ControllerMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__msg__ControllerMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__CONTROLLER_MSG__STRUCT_H_
