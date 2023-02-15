// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from capella_ros_service_interfaces:action/TestAction.idl
// generated code does not contain a copyright notice

#ifndef CAPELLA_ROS_SERVICE_INTERFACES__ACTION__DETAIL__TEST_ACTION__STRUCT_H_
#define CAPELLA_ROS_SERVICE_INTERFACES__ACTION__DETAIL__TEST_ACTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in action/TestAction in the package capella_ros_service_interfaces.
typedef struct capella_ros_service_interfaces__action__TestAction_Goal
{
  int32_t order;
} capella_ros_service_interfaces__action__TestAction_Goal;

// Struct for a sequence of capella_ros_service_interfaces__action__TestAction_Goal.
typedef struct capella_ros_service_interfaces__action__TestAction_Goal__Sequence
{
  capella_ros_service_interfaces__action__TestAction_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} capella_ros_service_interfaces__action__TestAction_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'sequence'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in action/TestAction in the package capella_ros_service_interfaces.
typedef struct capella_ros_service_interfaces__action__TestAction_Result
{
  rosidl_runtime_c__int32__Sequence sequence;
} capella_ros_service_interfaces__action__TestAction_Result;

// Struct for a sequence of capella_ros_service_interfaces__action__TestAction_Result.
typedef struct capella_ros_service_interfaces__action__TestAction_Result__Sequence
{
  capella_ros_service_interfaces__action__TestAction_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} capella_ros_service_interfaces__action__TestAction_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'partial_sequence'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in action/TestAction in the package capella_ros_service_interfaces.
typedef struct capella_ros_service_interfaces__action__TestAction_Feedback
{
  rosidl_runtime_c__int32__Sequence partial_sequence;
} capella_ros_service_interfaces__action__TestAction_Feedback;

// Struct for a sequence of capella_ros_service_interfaces__action__TestAction_Feedback.
typedef struct capella_ros_service_interfaces__action__TestAction_Feedback__Sequence
{
  capella_ros_service_interfaces__action__TestAction_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} capella_ros_service_interfaces__action__TestAction_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "capella_ros_service_interfaces/action/detail/test_action__struct.h"

// Struct defined in action/TestAction in the package capella_ros_service_interfaces.
typedef struct capella_ros_service_interfaces__action__TestAction_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  capella_ros_service_interfaces__action__TestAction_Goal goal;
} capella_ros_service_interfaces__action__TestAction_SendGoal_Request;

// Struct for a sequence of capella_ros_service_interfaces__action__TestAction_SendGoal_Request.
typedef struct capella_ros_service_interfaces__action__TestAction_SendGoal_Request__Sequence
{
  capella_ros_service_interfaces__action__TestAction_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} capella_ros_service_interfaces__action__TestAction_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in action/TestAction in the package capella_ros_service_interfaces.
typedef struct capella_ros_service_interfaces__action__TestAction_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} capella_ros_service_interfaces__action__TestAction_SendGoal_Response;

// Struct for a sequence of capella_ros_service_interfaces__action__TestAction_SendGoal_Response.
typedef struct capella_ros_service_interfaces__action__TestAction_SendGoal_Response__Sequence
{
  capella_ros_service_interfaces__action__TestAction_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} capella_ros_service_interfaces__action__TestAction_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

// Struct defined in action/TestAction in the package capella_ros_service_interfaces.
typedef struct capella_ros_service_interfaces__action__TestAction_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} capella_ros_service_interfaces__action__TestAction_GetResult_Request;

// Struct for a sequence of capella_ros_service_interfaces__action__TestAction_GetResult_Request.
typedef struct capella_ros_service_interfaces__action__TestAction_GetResult_Request__Sequence
{
  capella_ros_service_interfaces__action__TestAction_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} capella_ros_service_interfaces__action__TestAction_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "capella_ros_service_interfaces/action/detail/test_action__struct.h"

// Struct defined in action/TestAction in the package capella_ros_service_interfaces.
typedef struct capella_ros_service_interfaces__action__TestAction_GetResult_Response
{
  int8_t status;
  capella_ros_service_interfaces__action__TestAction_Result result;
} capella_ros_service_interfaces__action__TestAction_GetResult_Response;

// Struct for a sequence of capella_ros_service_interfaces__action__TestAction_GetResult_Response.
typedef struct capella_ros_service_interfaces__action__TestAction_GetResult_Response__Sequence
{
  capella_ros_service_interfaces__action__TestAction_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} capella_ros_service_interfaces__action__TestAction_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "capella_ros_service_interfaces/action/detail/test_action__struct.h"

// Struct defined in action/TestAction in the package capella_ros_service_interfaces.
typedef struct capella_ros_service_interfaces__action__TestAction_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  capella_ros_service_interfaces__action__TestAction_Feedback feedback;
} capella_ros_service_interfaces__action__TestAction_FeedbackMessage;

// Struct for a sequence of capella_ros_service_interfaces__action__TestAction_FeedbackMessage.
typedef struct capella_ros_service_interfaces__action__TestAction_FeedbackMessage__Sequence
{
  capella_ros_service_interfaces__action__TestAction_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} capella_ros_service_interfaces__action__TestAction_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CAPELLA_ROS_SERVICE_INTERFACES__ACTION__DETAIL__TEST_ACTION__STRUCT_H_
