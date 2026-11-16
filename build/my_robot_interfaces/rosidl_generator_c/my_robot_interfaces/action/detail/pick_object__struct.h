// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_robot_interfaces:action/PickObject.idl
// generated code does not contain a copyright notice

#ifndef MY_ROBOT_INTERFACES__ACTION__DETAIL__PICK_OBJECT__STRUCT_H_
#define MY_ROBOT_INTERFACES__ACTION__DETAIL__PICK_OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'object_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in action/PickObject in the package my_robot_interfaces.
typedef struct my_robot_interfaces__action__PickObject_Goal
{
  /// cylinder axis point at mid-height; frame: odom
  geometry_msgs__msg__PoseStamped object_pose;
  /// 0.05
  float object_diameter;
  /// 0.15
  float object_height;
} my_robot_interfaces__action__PickObject_Goal;

// Struct for a sequence of my_robot_interfaces__action__PickObject_Goal.
typedef struct my_robot_interfaces__action__PickObject_Goal__Sequence
{
  my_robot_interfaces__action__PickObject_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__action__PickObject_Goal__Sequence;


// Constants defined in the message

/// Constant 'NONE'.
enum
{
  my_robot_interfaces__action__PickObject_Result__NONE = 0
};

/// Constant 'UNREACHABLE'.
enum
{
  my_robot_interfaces__action__PickObject_Result__UNREACHABLE = 1
};

/// Constant 'PLANNING_FAILED'.
enum
{
  my_robot_interfaces__action__PickObject_Result__PLANNING_FAILED = 2
};

/// Constant 'EXECUTION_FAILED'.
enum
{
  my_robot_interfaces__action__PickObject_Result__EXECUTION_FAILED = 3
};

/// Constant 'GRASP_NOT_DETECTED'.
enum
{
  my_robot_interfaces__action__PickObject_Result__GRASP_NOT_DETECTED = 4
};

/// Constant 'OBJECT_LOST'.
enum
{
  my_robot_interfaces__action__PickObject_Result__OBJECT_LOST = 5
};

/// Constant 'TIMEOUT'.
enum
{
  my_robot_interfaces__action__PickObject_Result__TIMEOUT = 6
};

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/PickObject in the package my_robot_interfaces.
typedef struct my_robot_interfaces__action__PickObject_Result
{
  bool success;
  uint8_t error_code;
  rosidl_runtime_c__String message;
} my_robot_interfaces__action__PickObject_Result;

// Struct for a sequence of my_robot_interfaces__action__PickObject_Result.
typedef struct my_robot_interfaces__action__PickObject_Result__Sequence
{
  my_robot_interfaces__action__PickObject_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__action__PickObject_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stage'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/PickObject in the package my_robot_interfaces.
typedef struct my_robot_interfaces__action__PickObject_Feedback
{
  /// "open", "pregrasp", "descend", "close", "verify", "lift", "transport"
  rosidl_runtime_c__String stage;
} my_robot_interfaces__action__PickObject_Feedback;

// Struct for a sequence of my_robot_interfaces__action__PickObject_Feedback.
typedef struct my_robot_interfaces__action__PickObject_Feedback__Sequence
{
  my_robot_interfaces__action__PickObject_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__action__PickObject_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "my_robot_interfaces/action/detail/pick_object__struct.h"

/// Struct defined in action/PickObject in the package my_robot_interfaces.
typedef struct my_robot_interfaces__action__PickObject_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  my_robot_interfaces__action__PickObject_Goal goal;
} my_robot_interfaces__action__PickObject_SendGoal_Request;

// Struct for a sequence of my_robot_interfaces__action__PickObject_SendGoal_Request.
typedef struct my_robot_interfaces__action__PickObject_SendGoal_Request__Sequence
{
  my_robot_interfaces__action__PickObject_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__action__PickObject_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/PickObject in the package my_robot_interfaces.
typedef struct my_robot_interfaces__action__PickObject_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} my_robot_interfaces__action__PickObject_SendGoal_Response;

// Struct for a sequence of my_robot_interfaces__action__PickObject_SendGoal_Response.
typedef struct my_robot_interfaces__action__PickObject_SendGoal_Response__Sequence
{
  my_robot_interfaces__action__PickObject_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__action__PickObject_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/PickObject in the package my_robot_interfaces.
typedef struct my_robot_interfaces__action__PickObject_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} my_robot_interfaces__action__PickObject_GetResult_Request;

// Struct for a sequence of my_robot_interfaces__action__PickObject_GetResult_Request.
typedef struct my_robot_interfaces__action__PickObject_GetResult_Request__Sequence
{
  my_robot_interfaces__action__PickObject_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__action__PickObject_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "my_robot_interfaces/action/detail/pick_object__struct.h"

/// Struct defined in action/PickObject in the package my_robot_interfaces.
typedef struct my_robot_interfaces__action__PickObject_GetResult_Response
{
  int8_t status;
  my_robot_interfaces__action__PickObject_Result result;
} my_robot_interfaces__action__PickObject_GetResult_Response;

// Struct for a sequence of my_robot_interfaces__action__PickObject_GetResult_Response.
typedef struct my_robot_interfaces__action__PickObject_GetResult_Response__Sequence
{
  my_robot_interfaces__action__PickObject_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__action__PickObject_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "my_robot_interfaces/action/detail/pick_object__struct.h"

/// Struct defined in action/PickObject in the package my_robot_interfaces.
typedef struct my_robot_interfaces__action__PickObject_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  my_robot_interfaces__action__PickObject_Feedback feedback;
} my_robot_interfaces__action__PickObject_FeedbackMessage;

// Struct for a sequence of my_robot_interfaces__action__PickObject_FeedbackMessage.
typedef struct my_robot_interfaces__action__PickObject_FeedbackMessage__Sequence
{
  my_robot_interfaces__action__PickObject_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_robot_interfaces__action__PickObject_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_ROBOT_INTERFACES__ACTION__DETAIL__PICK_OBJECT__STRUCT_H_
