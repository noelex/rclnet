// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from capella_ros_service_interfaces:srv/CartographerMode.idl
// generated code does not contain a copyright notice

#ifndef CAPELLA_ROS_SERVICE_INTERFACES__SRV__DETAIL__CARTOGRAPHER_MODE__STRUCT_H_
#define CAPELLA_ROS_SERVICE_INTERFACES__SRV__DETAIL__CARTOGRAPHER_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/CartographerMode in the package capella_ros_service_interfaces.
typedef struct capella_ros_service_interfaces__srv__CartographerMode_Request
{
  uint8_t structure_needs_at_least_one_member;
} capella_ros_service_interfaces__srv__CartographerMode_Request;

// Struct for a sequence of capella_ros_service_interfaces__srv__CartographerMode_Request.
typedef struct capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence
{
  capella_ros_service_interfaces__srv__CartographerMode_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/CartographerMode in the package capella_ros_service_interfaces.
typedef struct capella_ros_service_interfaces__srv__CartographerMode_Response
{
  uint8_t cart_status;
} capella_ros_service_interfaces__srv__CartographerMode_Response;

// Struct for a sequence of capella_ros_service_interfaces__srv__CartographerMode_Response.
typedef struct capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence
{
  capella_ros_service_interfaces__srv__CartographerMode_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CAPELLA_ROS_SERVICE_INTERFACES__SRV__DETAIL__CARTOGRAPHER_MODE__STRUCT_H_
