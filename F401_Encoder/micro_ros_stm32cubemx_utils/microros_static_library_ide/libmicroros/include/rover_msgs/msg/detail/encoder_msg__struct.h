// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rover_msgs:msg/EncoderMsg.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__ENCODER_MSG__STRUCT_H_
#define ROVER_MSGS__MSG__DETAIL__ENCODER_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/EncoderMsg in the package rover_msgs.
typedef struct rover_msgs__msg__EncoderMsg
{
  int32_t m1;
  int32_t m2;
  int32_t m3;
  int32_t m4;
  int32_t roll;
  int32_t yaw;
  int32_t pitch;
} rover_msgs__msg__EncoderMsg;

// Struct for a sequence of rover_msgs__msg__EncoderMsg.
typedef struct rover_msgs__msg__EncoderMsg__Sequence
{
  rover_msgs__msg__EncoderMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rover_msgs__msg__EncoderMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__ENCODER_MSG__STRUCT_H_
