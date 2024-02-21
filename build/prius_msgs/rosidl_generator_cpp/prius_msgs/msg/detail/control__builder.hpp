// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from prius_msgs:msg/Control.idl
// generated code does not contain a copyright notice

#ifndef PRIUS_MSGS__MSG__DETAIL__CONTROL__BUILDER_HPP_
#define PRIUS_MSGS__MSG__DETAIL__CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "prius_msgs/msg/detail/control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace prius_msgs
{

namespace msg
{

namespace builder
{

class Init_Control_shift_gears
{
public:
  explicit Init_Control_shift_gears(::prius_msgs::msg::Control & msg)
  : msg_(msg)
  {}
  ::prius_msgs::msg::Control shift_gears(::prius_msgs::msg::Control::_shift_gears_type arg)
  {
    msg_.shift_gears = std::move(arg);
    return std::move(msg_);
  }

private:
  ::prius_msgs::msg::Control msg_;
};

class Init_Control_steer
{
public:
  explicit Init_Control_steer(::prius_msgs::msg::Control & msg)
  : msg_(msg)
  {}
  Init_Control_shift_gears steer(::prius_msgs::msg::Control::_steer_type arg)
  {
    msg_.steer = std::move(arg);
    return Init_Control_shift_gears(msg_);
  }

private:
  ::prius_msgs::msg::Control msg_;
};

class Init_Control_brake
{
public:
  explicit Init_Control_brake(::prius_msgs::msg::Control & msg)
  : msg_(msg)
  {}
  Init_Control_steer brake(::prius_msgs::msg::Control::_brake_type arg)
  {
    msg_.brake = std::move(arg);
    return Init_Control_steer(msg_);
  }

private:
  ::prius_msgs::msg::Control msg_;
};

class Init_Control_throttle
{
public:
  explicit Init_Control_throttle(::prius_msgs::msg::Control & msg)
  : msg_(msg)
  {}
  Init_Control_brake throttle(::prius_msgs::msg::Control::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_Control_brake(msg_);
  }

private:
  ::prius_msgs::msg::Control msg_;
};

class Init_Control_header
{
public:
  Init_Control_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Control_throttle header(::prius_msgs::msg::Control::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Control_throttle(msg_);
  }

private:
  ::prius_msgs::msg::Control msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::prius_msgs::msg::Control>()
{
  return prius_msgs::msg::builder::Init_Control_header();
}

}  // namespace prius_msgs

#endif  // PRIUS_MSGS__MSG__DETAIL__CONTROL__BUILDER_HPP_
