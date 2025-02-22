// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rover_msgs:msg/ControllerMsg.idl
// generated code does not contain a copyright notice
#include "rover_msgs/msg/detail/controller_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
rover_msgs__msg__ControllerMsg__init(rover_msgs__msg__ControllerMsg * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // throttle
  // camerax
  // cameray
  // light
  // gear
  return true;
}

void
rover_msgs__msg__ControllerMsg__fini(rover_msgs__msg__ControllerMsg * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // throttle
  // camerax
  // cameray
  // light
  // gear
}

bool
rover_msgs__msg__ControllerMsg__are_equal(const rover_msgs__msg__ControllerMsg * lhs, const rover_msgs__msg__ControllerMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // throttle
  if (lhs->throttle != rhs->throttle) {
    return false;
  }
  // camerax
  if (lhs->camerax != rhs->camerax) {
    return false;
  }
  // cameray
  if (lhs->cameray != rhs->cameray) {
    return false;
  }
  // light
  if (lhs->light != rhs->light) {
    return false;
  }
  // gear
  if (lhs->gear != rhs->gear) {
    return false;
  }
  return true;
}

bool
rover_msgs__msg__ControllerMsg__copy(
  const rover_msgs__msg__ControllerMsg * input,
  rover_msgs__msg__ControllerMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // throttle
  output->throttle = input->throttle;
  // camerax
  output->camerax = input->camerax;
  // cameray
  output->cameray = input->cameray;
  // light
  output->light = input->light;
  // gear
  output->gear = input->gear;
  return true;
}

rover_msgs__msg__ControllerMsg *
rover_msgs__msg__ControllerMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__ControllerMsg * msg = (rover_msgs__msg__ControllerMsg *)allocator.allocate(sizeof(rover_msgs__msg__ControllerMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rover_msgs__msg__ControllerMsg));
  bool success = rover_msgs__msg__ControllerMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rover_msgs__msg__ControllerMsg__destroy(rover_msgs__msg__ControllerMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rover_msgs__msg__ControllerMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rover_msgs__msg__ControllerMsg__Sequence__init(rover_msgs__msg__ControllerMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__ControllerMsg * data = NULL;

  if (size) {
    data = (rover_msgs__msg__ControllerMsg *)allocator.zero_allocate(size, sizeof(rover_msgs__msg__ControllerMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rover_msgs__msg__ControllerMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rover_msgs__msg__ControllerMsg__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rover_msgs__msg__ControllerMsg__Sequence__fini(rover_msgs__msg__ControllerMsg__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rover_msgs__msg__ControllerMsg__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rover_msgs__msg__ControllerMsg__Sequence *
rover_msgs__msg__ControllerMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rover_msgs__msg__ControllerMsg__Sequence * array = (rover_msgs__msg__ControllerMsg__Sequence *)allocator.allocate(sizeof(rover_msgs__msg__ControllerMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rover_msgs__msg__ControllerMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rover_msgs__msg__ControllerMsg__Sequence__destroy(rover_msgs__msg__ControllerMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rover_msgs__msg__ControllerMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rover_msgs__msg__ControllerMsg__Sequence__are_equal(const rover_msgs__msg__ControllerMsg__Sequence * lhs, const rover_msgs__msg__ControllerMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rover_msgs__msg__ControllerMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rover_msgs__msg__ControllerMsg__Sequence__copy(
  const rover_msgs__msg__ControllerMsg__Sequence * input,
  rover_msgs__msg__ControllerMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rover_msgs__msg__ControllerMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rover_msgs__msg__ControllerMsg * data =
      (rover_msgs__msg__ControllerMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rover_msgs__msg__ControllerMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rover_msgs__msg__ControllerMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rover_msgs__msg__ControllerMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
