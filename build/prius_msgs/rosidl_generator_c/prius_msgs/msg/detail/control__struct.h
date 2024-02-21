// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from prius_msgs:msg/Control.idl
// generated code does not contain a copyright notice

#ifndef PRIUS_MSGS__MSG__DETAIL__CONTROL__STRUCT_H_
#define PRIUS_MSGS__MSG__DETAIL__CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NO_COMMAND'.
enum
{
  prius_msgs__msg__Control__NO_COMMAND = 0
};

/// Constant 'NEUTRAL'.
enum
{
  prius_msgs__msg__Control__NEUTRAL = 1
};

/// Constant 'FORWARD'.
enum
{
  prius_msgs__msg__Control__FORWARD = 2
};

/// Constant 'REVERSE'.
enum
{
  prius_msgs__msg__Control__REVERSE = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/Control in the package prius_msgs.
typedef struct prius_msgs__msg__Control
{
  std_msgs__msg__Header header;
  /// Range 0 to 1, 1 is max throttle
  double throttle;
  /// Range 0 to 1, 1 is max brake
  double brake;
  /// Range -1 to +1, +1 is maximum left turn
  double steer;
  uint8_t shift_gears;
} prius_msgs__msg__Control;

// Struct for a sequence of prius_msgs__msg__Control.
typedef struct prius_msgs__msg__Control__Sequence
{
  prius_msgs__msg__Control * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} prius_msgs__msg__Control__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PRIUS_MSGS__MSG__DETAIL__CONTROL__STRUCT_H_
