// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from prius_msgs:msg/Control.idl
// generated code does not contain a copyright notice

#ifndef PRIUS_MSGS__MSG__DETAIL__CONTROL__STRUCT_HPP_
#define PRIUS_MSGS__MSG__DETAIL__CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__prius_msgs__msg__Control __attribute__((deprecated))
#else
# define DEPRECATED__prius_msgs__msg__Control __declspec(deprecated)
#endif

namespace prius_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Control_
{
  using Type = Control_<ContainerAllocator>;

  explicit Control_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->throttle = 0.0;
      this->brake = 0.0;
      this->steer = 0.0;
      this->shift_gears = 0;
    }
  }

  explicit Control_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->throttle = 0.0;
      this->brake = 0.0;
      this->steer = 0.0;
      this->shift_gears = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _throttle_type =
    double;
  _throttle_type throttle;
  using _brake_type =
    double;
  _brake_type brake;
  using _steer_type =
    double;
  _steer_type steer;
  using _shift_gears_type =
    uint8_t;
  _shift_gears_type shift_gears;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__throttle(
    const double & _arg)
  {
    this->throttle = _arg;
    return *this;
  }
  Type & set__brake(
    const double & _arg)
  {
    this->brake = _arg;
    return *this;
  }
  Type & set__steer(
    const double & _arg)
  {
    this->steer = _arg;
    return *this;
  }
  Type & set__shift_gears(
    const uint8_t & _arg)
  {
    this->shift_gears = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t NO_COMMAND =
    0u;
  static constexpr uint8_t NEUTRAL =
    1u;
  static constexpr uint8_t FORWARD =
    2u;
  static constexpr uint8_t REVERSE =
    3u;

  // pointer types
  using RawPtr =
    prius_msgs::msg::Control_<ContainerAllocator> *;
  using ConstRawPtr =
    const prius_msgs::msg::Control_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<prius_msgs::msg::Control_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<prius_msgs::msg::Control_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      prius_msgs::msg::Control_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<prius_msgs::msg::Control_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      prius_msgs::msg::Control_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<prius_msgs::msg::Control_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<prius_msgs::msg::Control_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<prius_msgs::msg::Control_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__prius_msgs__msg__Control
    std::shared_ptr<prius_msgs::msg::Control_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__prius_msgs__msg__Control
    std::shared_ptr<prius_msgs::msg::Control_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Control_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->throttle != other.throttle) {
      return false;
    }
    if (this->brake != other.brake) {
      return false;
    }
    if (this->steer != other.steer) {
      return false;
    }
    if (this->shift_gears != other.shift_gears) {
      return false;
    }
    return true;
  }
  bool operator!=(const Control_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Control_

// alias to use template instance with default allocator
using Control =
  prius_msgs::msg::Control_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Control_<ContainerAllocator>::NO_COMMAND;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Control_<ContainerAllocator>::NEUTRAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Control_<ContainerAllocator>::FORWARD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Control_<ContainerAllocator>::REVERSE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace prius_msgs

#endif  // PRIUS_MSGS__MSG__DETAIL__CONTROL__STRUCT_HPP_
