// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_robot_interfaces:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_H_
#define MY_ROBOT_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'label'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/Detection in the package my_robot_interfaces.
/**
  * A perceived object, fused over several observations. SPEC §7.
 */
typedef struct my_robot_interfaces__msg__Detection
{
  /// frame_id: map (pose expressed in map == Gazebo world, DC-9)
  std_msgs__msg__Header header;
  /// "target" | "box"
  rosidl_runtime_c__String label;
  /// target: cylinder axis point at mid-height; box: centre of the box floor (inside)
  geometry_msgs__msg__Pose pose;
  /// 0..1
  float confidence;
  /// m, std-dev of the fused observations
  float position_stddev;
  uint32_t observation_count;
} my_robot_interfaces__msg__Detection;

// Struct for a sequence of my_robot_interfaces__msg__Detection.
typedef struct my_robot_interfaces__msg__Detection__Sequence
{
  my_robot_interfaces__msg__Detection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__msg__Detection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_ROBOT_INTERFACES__MSG__DETAIL__DETECTION__STRUCT_H_
