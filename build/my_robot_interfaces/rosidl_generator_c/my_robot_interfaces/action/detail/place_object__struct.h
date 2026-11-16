// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_robot_interfaces:action/PlaceObject.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_INTERFACES__ACTION__DETAIL__PLACE_OBJECT__STRUCT_H_
#define MY_ROBOT_INTERFACES__ACTION__DETAIL__PLACE_OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'place_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/PlaceObject in the package my_robot_interfaces.
typedef struct my_robot_interfaces__action__PlaceObject_Goal
{
  /// desired object centre (axis mid-height) when released; frame: odom
  geometry_msgs__msg__PoseStamped place_pose;
} my_robot_interfaces__action__PlaceObject_Goal;

// Struct for a sequence of my_robot_interfaces__action__PlaceObject_Goal.
typedef struct my_robot_interfaces__action__PlaceObject_Goal__Sequence
{
  my_robot_interfaces__action__PlaceObject_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__action__PlaceObject_Goal__Sequence;


// Constants defined in the message

/// Constant 'NONE'.
enum
{
  my_robot_interfaces__action__PlaceObject_Result__NONE = 0
};

/// Constant 'UNREACHABLE'.
enum
{
  my_robot_interfaces__action__PlaceObject_Result__UNREACHABLE = 1
};

/// Constant 'PLANNING_FAILED'.
enum
{
  my_robot_interfaces__action__PlaceObject_Result__PLANNING_FAILED = 2
};

/// Constant 'EXECUTION_FAILED'.
enum
{
  my_robot_interfaces__action__PlaceObject_Result__EXECUTION_FAILED = 3
};

/// Constant 'RELEASE_NOT_CONFIRMED'.
enum
{
  my_robot_interfaces__action__PlaceObject_Result__RELEASE_NOT_CONFIRMED = 4
};

/// Constant 'TIMEOUT'.
enum
{
  my_robot_interfaces__action__PlaceObject_Result__TIMEOUT = 6
};

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/PlaceObject in the package my_robot_interfaces.
typedef struct my_robot_interfaces__action__PlaceObject_Result
{
  bool success;
  uint8_t error_code;
  rosidl_runtime_c__String message;
} my_robot_interfaces__action__PlaceObject_Result;

// Struct for a sequence of my_robot_interfaces__action__PlaceObject_Result.
typedef struct my_robot_interfaces__action__PlaceObject_Result__Sequence
{
  my_robot_interfaces__action__PlaceObject_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__action__PlaceObject_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stage'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/PlaceObject in the package my_robot_interfaces.
typedef struct my_robot_interfaces__action__PlaceObject_Feedback
{
  /// "preplace", "descend", "release", "verify", "retract"
  rosidl_runtime_c__String stage;
} my_robot_interfaces__action__PlaceObject_Feedback;

// Struct for a sequence of my_robot_interfaces__action__PlaceObject_Feedback.
typedef struct my_robot_interfaces__action__PlaceObject_Feedback__Sequence
{
  my_robot_interfaces__action__PlaceObject_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__action__PlaceObject_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "my_robot_interfaces/action/detail/place_object__struct.h"

/// Struct defined in action/PlaceObject in the package my_robot_interfaces.
typedef struct my_robot_interfaces__action__PlaceObject_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  my_robot_interfaces__action__PlaceObject_Goal goal;
} my_robot_interfaces__action__PlaceObject_SendGoal_Request;

// Struct for a sequence of my_robot_interfaces__action__PlaceObject_SendGoal_Request.
typedef struct my_robot_interfaces__action__PlaceObject_SendGoal_Request__Sequence
{
  my_robot_interfaces__action__PlaceObject_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__action__PlaceObject_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/PlaceObject in the package my_robot_interfaces.
typedef struct my_robot_interfaces__action__PlaceObject_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} my_robot_interfaces__action__PlaceObject_SendGoal_Response;

// Struct for a sequence of my_robot_interfaces__action__PlaceObject_SendGoal_Response.
typedef struct my_robot_interfaces__action__PlaceObject_SendGoal_Response__Sequence
{
  my_robot_interfaces__action__PlaceObject_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__action__PlaceObject_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/PlaceObject in the package my_robot_interfaces.
typedef struct my_robot_interfaces__action__PlaceObject_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} my_robot_interfaces__action__PlaceObject_GetResult_Request;

// Struct for a sequence of my_robot_interfaces__action__PlaceObject_GetResult_Request.
typedef struct my_robot_interfaces__action__PlaceObject_GetResult_Request__Sequence
{
  my_robot_interfaces__action__PlaceObject_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__action__PlaceObject_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "my_robot_interfaces/action/detail/place_object__struct.h"

/// Struct defined in action/PlaceObject in the package my_robot_interfaces.
typedef struct my_robot_interfaces__action__PlaceObject_GetResult_Response
{
  int8_t status;
  my_robot_interfaces__action__PlaceObject_Result result;
} my_robot_interfaces__action__PlaceObject_GetResult_Response;

// Struct for a sequence of my_robot_interfaces__action__PlaceObject_GetResult_Response.
typedef struct my_robot_interfaces__action__PlaceObject_GetResult_Response__Sequence
{
  my_robot_interfaces__action__PlaceObject_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__action__PlaceObject_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "my_robot_interfaces/action/detail/place_object__struct.h"

/// Struct defined in action/PlaceObject in the package my_robot_interfaces.
typedef struct my_robot_interfaces__action__PlaceObject_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  my_robot_interfaces__action__PlaceObject_Feedback feedback;
} my_robot_interfaces__action__PlaceObject_FeedbackMessage;

// Struct for a sequence of my_robot_interfaces__action__PlaceObject_FeedbackMessage.
typedef struct my_robot_interfaces__action__PlaceObject_FeedbackMessage__Sequence
{
  my_robot_interfaces__action__PlaceObject_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__action__PlaceObject_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_ROBOT_INTERFACES__ACTION__DETAIL__PLACE_OBJECT__STRUCT_H_
