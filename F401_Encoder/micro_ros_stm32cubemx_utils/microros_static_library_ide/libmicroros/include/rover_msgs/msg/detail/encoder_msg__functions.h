// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rover_msgs:msg/EncoderMsg.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__ENCODER_MSG__FUNCTIONS_H_
#define ROVER_MSGS__MSG__DETAIL__ENCODER_MSG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "rover_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "rover_msgs/msg/detail/encoder_msg__struct.h"

/// Initialize msg/EncoderMsg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rover_msgs__msg__EncoderMsg
 * )) before or use
 * rover_msgs__msg__EncoderMsg__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
bool
rover_msgs__msg__EncoderMsg__init(rover_msgs__msg__EncoderMsg * msg);

/// Finalize msg/EncoderMsg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
void
rover_msgs__msg__EncoderMsg__fini(rover_msgs__msg__EncoderMsg * msg);

/// Create msg/EncoderMsg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rover_msgs__msg__EncoderMsg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
rover_msgs__msg__EncoderMsg *
rover_msgs__msg__EncoderMsg__create();

/// Destroy msg/EncoderMsg message.
/**
 * It calls
 * rover_msgs__msg__EncoderMsg__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
void
rover_msgs__msg__EncoderMsg__destroy(rover_msgs__msg__EncoderMsg * msg);

/// Check for msg/EncoderMsg message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
bool
rover_msgs__msg__EncoderMsg__are_equal(const rover_msgs__msg__EncoderMsg * lhs, const rover_msgs__msg__EncoderMsg * rhs);

/// Copy a msg/EncoderMsg message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
bool
rover_msgs__msg__EncoderMsg__copy(
  const rover_msgs__msg__EncoderMsg * input,
  rover_msgs__msg__EncoderMsg * output);

/// Initialize array of msg/EncoderMsg messages.
/**
 * It allocates the memory for the number of elements and calls
 * rover_msgs__msg__EncoderMsg__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
bool
rover_msgs__msg__EncoderMsg__Sequence__init(rover_msgs__msg__EncoderMsg__Sequence * array, size_t size);

/// Finalize array of msg/EncoderMsg messages.
/**
 * It calls
 * rover_msgs__msg__EncoderMsg__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
void
rover_msgs__msg__EncoderMsg__Sequence__fini(rover_msgs__msg__EncoderMsg__Sequence * array);

/// Create array of msg/EncoderMsg messages.
/**
 * It allocates the memory for the array and calls
 * rover_msgs__msg__EncoderMsg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
rover_msgs__msg__EncoderMsg__Sequence *
rover_msgs__msg__EncoderMsg__Sequence__create(size_t size);

/// Destroy array of msg/EncoderMsg messages.
/**
 * It calls
 * rover_msgs__msg__EncoderMsg__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
void
rover_msgs__msg__EncoderMsg__Sequence__destroy(rover_msgs__msg__EncoderMsg__Sequence * array);

/// Check for msg/EncoderMsg message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
bool
rover_msgs__msg__EncoderMsg__Sequence__are_equal(const rover_msgs__msg__EncoderMsg__Sequence * lhs, const rover_msgs__msg__EncoderMsg__Sequence * rhs);

/// Copy an array of msg/EncoderMsg messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rover_msgs
bool
rover_msgs__msg__EncoderMsg__Sequence__copy(
  const rover_msgs__msg__EncoderMsg__Sequence * input,
  rover_msgs__msg__EncoderMsg__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROVER_MSGS__MSG__DETAIL__ENCODER_MSG__FUNCTIONS_H_
