// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rover_msgs:msg/ControllerMsg.idl
// generated code does not contain a copyright notice

#ifndef ROVER_MSGS__MSG__DETAIL__CONTROLLER_MSG__BUILDER_HPP_
#define ROVER_MSGS__MSG__DETAIL__CONTROLLER_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rover_msgs/msg/detail/controller_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rover_msgs
{

namespace msg
{

namespace builder
{

class Init_ControllerMsg_gear
{
public:
  explicit Init_ControllerMsg_gear(::rover_msgs::msg::ControllerMsg & msg)
  : msg_(msg)
  {}
  ::rover_msgs::msg::ControllerMsg gear(::rover_msgs::msg::ControllerMsg::_gear_type arg)
  {
    msg_.gear = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rover_msgs::msg::ControllerMsg msg_;
};

class Init_ControllerMsg_light
{
public:
  explicit Init_ControllerMsg_light(::rover_msgs::msg::ControllerMsg & msg)
  : msg_(msg)
  {}
  Init_ControllerMsg_gear light(::rover_msgs::msg::ControllerMsg::_light_type arg)
  {
    msg_.light = std::move(arg);
    return Init_ControllerMsg_gear(msg_);
  }

private:
  ::rover_msgs::msg::ControllerMsg msg_;
};

class Init_ControllerMsg_cameray
{
public:
  explicit Init_ControllerMsg_cameray(::rover_msgs::msg::ControllerMsg & msg)
  : msg_(msg)
  {}
  Init_ControllerMsg_light cameray(::rover_msgs::msg::ControllerMsg::_cameray_type arg)
  {
    msg_.cameray = std::move(arg);
    return Init_ControllerMsg_light(msg_);
  }

private:
  ::rover_msgs::msg::ControllerMsg msg_;
};

class Init_ControllerMsg_camerax
{
public:
  explicit Init_ControllerMsg_camerax(::rover_msgs::msg::ControllerMsg & msg)
  : msg_(msg)
  {}
  Init_ControllerMsg_cameray camerax(::rover_msgs::msg::ControllerMsg::_camerax_type arg)
  {
    msg_.camerax = std::move(arg);
    return Init_ControllerMsg_cameray(msg_);
  }

private:
  ::rover_msgs::msg::ControllerMsg msg_;
};

class Init_ControllerMsg_throttle
{
public:
  explicit Init_ControllerMsg_throttle(::rover_msgs::msg::ControllerMsg & msg)
  : msg_(msg)
  {}
  Init_ControllerMsg_camerax throttle(::rover_msgs::msg::ControllerMsg::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_ControllerMsg_camerax(msg_);
  }

private:
  ::rover_msgs::msg::ControllerMsg msg_;
};

class Init_ControllerMsg_y
{
public:
  explicit Init_ControllerMsg_y(::rover_msgs::msg::ControllerMsg & msg)
  : msg_(msg)
  {}
  Init_ControllerMsg_throttle y(::rover_msgs::msg::ControllerMsg::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_ControllerMsg_throttle(msg_);
  }

private:
  ::rover_msgs::msg::ControllerMsg msg_;
};

class Init_ControllerMsg_x
{
public:
  Init_ControllerMsg_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControllerMsg_y x(::rover_msgs::msg::ControllerMsg::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_ControllerMsg_y(msg_);
  }

private:
  ::rover_msgs::msg::ControllerMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rover_msgs::msg::ControllerMsg>()
{
  return rover_msgs::msg::builder::Init_ControllerMsg_x();
}

}  // namespace rover_msgs

#endif  // ROVER_MSGS__MSG__DETAIL__CONTROLLER_MSG__BUILDER_HPP_
