// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from capella_ros_service_interfaces:msg/ChargerState.idl
// generated code does not contain a copyright notice

#ifndef CAPELLA_ROS_SERVICE_INTERFACES__MSG__DETAIL__CHARGER_STATE__STRUCT_H_
#define CAPELLA_ROS_SERVICE_INTERFACES__MSG__DETAIL__CHARGER_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pid'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/ChargerState in the package capella_ros_service_interfaces.
typedef struct capella_ros_service_interfaces__msg__ChargerState
{
  rosidl_runtime_c__String pid;
  bool is_charging;
  bool is_docking;
  bool has_contact;
} capella_ros_service_interfaces__msg__ChargerState;

// Struct for a sequence of capella_ros_service_interfaces__msg__ChargerState.
typedef struct capella_ros_service_interfaces__msg__ChargerState__Sequence
{
  capella_ros_service_interfaces__msg__ChargerState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} capella_ros_service_interfaces__msg__ChargerState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CAPELLA_ROS_SERVICE_INTERFACES__MSG__DETAIL__CHARGER_STATE__STRUCT_H_
