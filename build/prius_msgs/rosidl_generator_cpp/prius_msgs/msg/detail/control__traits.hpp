// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from prius_msgs:msg/Control.idl
// generated code does not contain a copyright notice

#ifndef PRIUS_MSGS__MSG__DETAIL__CONTROL__TRAITS_HPP_
#define PRIUS_MSGS__MSG__DETAIL__CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "prius_msgs/msg/detail/control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace prius_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Control & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: throttle
  {
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << ", ";
  }

  // member: brake
  {
    out << "brake: ";
    rosidl_generator_traits::value_to_yaml(msg.brake, out);
    out << ", ";
  }

  // member: steer
  {
    out << "steer: ";
    rosidl_generator_traits::value_to_yaml(msg.steer, out);
    out << ", ";
  }

  // member: shift_gears
  {
    out << "shift_gears: ";
    rosidl_generator_traits::value_to_yaml(msg.shift_gears, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Control & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
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

  // member: brake
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "brake: ";
    rosidl_generator_traits::value_to_yaml(msg.brake, out);
    out << "\n";
  }

  // member: steer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steer: ";
    rosidl_generator_traits::value_to_yaml(msg.steer, out);
    out << "\n";
  }

  // member: shift_gears
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shift_gears: ";
    rosidl_generator_traits::value_to_yaml(msg.shift_gears, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Control & msg, bool use_flow_style = false)
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

}  // namespace prius_msgs

namespace rosidl_generator_traits
{

[[deprecated("use prius_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const prius_msgs::msg::Control & msg,
  std::ostream & out, size_t indentation = 0)
{
  prius_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use prius_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const prius_msgs::msg::Control & msg)
{
  return prius_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<prius_msgs::msg::Control>()
{
  return "prius_msgs::msg::Control";
}

template<>
inline const char * name<prius_msgs::msg::Control>()
{
  return "prius_msgs/msg/Control";
}

template<>
struct has_fixed_size<prius_msgs::msg::Control>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<prius_msgs::msg::Control>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<prius_msgs::msg::Control>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PRIUS_MSGS__MSG__DETAIL__CONTROL__TRAITS_HPP_
