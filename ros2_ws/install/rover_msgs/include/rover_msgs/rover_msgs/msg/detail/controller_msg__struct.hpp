// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rover_msgs:msg/ControllerMsg.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__CONTROLLER_MSG__STRUCT_HPP_
#define ROVER_MSGS__MSG__DETAIL__CONTROLLER_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rover_msgs__msg__ControllerMsg __attribute__((deprecated))
#else
# define DEPRECATED__rover_msgs__msg__ControllerMsg __declspec(deprecated)
#endif

namespace rover_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ControllerMsg_
{
  using Type = ControllerMsg_<ContainerAllocator>;

  explicit ControllerMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->throttle = 0.0f;
      this->camerax = 0l;
      this->cameray = 0l;
      this->light = 0l;
      this->gear = 0;
    }
  }

  explicit ControllerMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->throttle = 0.0f;
      this->camerax = 0l;
      this->cameray = 0l;
      this->light = 0l;
      this->gear = 0;
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _throttle_type =
    float;
  _throttle_type throttle;
  using _camerax_type =
    int32_t;
  _camerax_type camerax;
  using _cameray_type =
    int32_t;
  _cameray_type cameray;
  using _light_type =
    int32_t;
  _light_type light;
  using _gear_type =
    int8_t;
  _gear_type gear;

  // setters for named parameter idiom
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__throttle(
    const float & _arg)
  {
    this->throttle = _arg;
    return *this;
  }
  Type & set__camerax(
    const int32_t & _arg)
  {
    this->camerax = _arg;
    return *this;
  }
  Type & set__cameray(
    const int32_t & _arg)
  {
    this->cameray = _arg;
    return *this;
  }
  Type & set__light(
    const int32_t & _arg)
  {
    this->light = _arg;
    return *this;
  }
  Type & set__gear(
    const int8_t & _arg)
  {
    this->gear = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rover_msgs::msg::ControllerMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const rover_msgs::msg::ControllerMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rover_msgs::msg::ControllerMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rover_msgs::msg::ControllerMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::ControllerMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::ControllerMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rover_msgs::msg::ControllerMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rover_msgs::msg::ControllerMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rover_msgs::msg::ControllerMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rover_msgs::msg::ControllerMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rover_msgs__msg__ControllerMsg
    std::shared_ptr<rover_msgs::msg::ControllerMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rover_msgs__msg__ControllerMsg
    std::shared_ptr<rover_msgs::msg::ControllerMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControllerMsg_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->throttle != other.throttle) {
      return false;
    }
    if (this->camerax != other.camerax) {
      return false;
    }
    if (this->cameray != other.cameray) {
      return false;
    }
    if (this->light != other.light) {
      return false;
    }
    if (this->gear != other.gear) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControllerMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControllerMsg_

// alias to use template instance with default allocator
using ControllerMsg =
  rover_msgs::msg::ControllerMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__CONTROLLER_MSG__STRUCT_HPP_
