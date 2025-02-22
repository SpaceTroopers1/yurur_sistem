// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rover_msgs:msg/ControllerMsg.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__CONTROLLER_MSG__TRAITS_HPP_
#define ROVER_MSGS__MSG__DETAIL__CONTROLLER_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rover_msgs/msg/detail/controller_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rover_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ControllerMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: throttle
  {
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << ", ";
  }

  // member: camerax
  {
    out << "camerax: ";
    rosidl_generator_traits::value_to_yaml(msg.camerax, out);
    out << ", ";
  }

  // member: cameray
  {
    out << "cameray: ";
    rosidl_generator_traits::value_to_yaml(msg.cameray, out);
    out << ", ";
  }

  // member: light
  {
    out << "light: ";
    rosidl_generator_traits::value_to_yaml(msg.light, out);
    out << ", ";
  }

  // member: gear
  {
    out << "gear: ";
    rosidl_generator_traits::value_to_yaml(msg.gear, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControllerMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: throttle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << "\n";
  }

  // member: camerax
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "camerax: ";
    rosidl_generator_traits::value_to_yaml(msg.camerax, out);
    out << "\n";
  }

  // member: cameray
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cameray: ";
    rosidl_generator_traits::value_to_yaml(msg.cameray, out);
    out << "\n";
  }

  // member: light
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "light: ";
    rosidl_generator_traits::value_to_yaml(msg.light, out);
    out << "\n";
  }

  // member: gear
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gear: ";
    rosidl_generator_traits::value_to_yaml(msg.gear, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControllerMsg & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rover_msgs

namespace rosidl_generator_traits
{

[[deprecated("use rover_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rover_msgs::msg::ControllerMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  rover_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rover_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const rover_msgs::msg::ControllerMsg & msg)
{
  return rover_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rover_msgs::msg::ControllerMsg>()
{
  return "rover_msgs::msg::ControllerMsg";
}

template<>
inline const char * name<rover_msgs::msg::ControllerMsg>()
{
  return "rover_msgs/msg/ControllerMsg";
}

template<>
struct has_fixed_size<rover_msgs::msg::ControllerMsg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<rover_msgs::msg::ControllerMsg>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<rover_msgs::msg::ControllerMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROVER_MSGS__MSG__DETAIL__CONTROLLER_MSG__TRAITS_HPP_
