// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from capella_ros_service_interfaces:srv/CartographerMode.idl
// generated code does not contain a copyright notice

#ifndef CAPELLA_ROS_SERVICE_INTERFACES__SRV__DETAIL__CARTOGRAPHER_MODE__FUNCTIONS_H_
#define CAPELLA_ROS_SERVICE_INTERFACES__SRV__DETAIL__CARTOGRAPHER_MODE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "capella_ros_service_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "capella_ros_service_interfaces/srv/detail/cartographer_mode__struct.h"

/// Initialize srv/CartographerMode message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * capella_ros_service_interfaces__srv__CartographerMode_Request
 * )) before or use
 * capella_ros_service_interfaces__srv__CartographerMode_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
bool
capella_ros_service_interfaces__srv__CartographerMode_Request__init(capella_ros_service_interfaces__srv__CartographerMode_Request * msg);

/// Finalize srv/CartographerMode message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
void
capella_ros_service_interfaces__srv__CartographerMode_Request__fini(capella_ros_service_interfaces__srv__CartographerMode_Request * msg);

/// Create srv/CartographerMode message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * capella_ros_service_interfaces__srv__CartographerMode_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
capella_ros_service_interfaces__srv__CartographerMode_Request *
capella_ros_service_interfaces__srv__CartographerMode_Request__create();

/// Destroy srv/CartographerMode message.
/**
 * It calls
 * capella_ros_service_interfaces__srv__CartographerMode_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
void
capella_ros_service_interfaces__srv__CartographerMode_Request__destroy(capella_ros_service_interfaces__srv__CartographerMode_Request * msg);

/// Check for srv/CartographerMode message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
bool
capella_ros_service_interfaces__srv__CartographerMode_Request__are_equal(const capella_ros_service_interfaces__srv__CartographerMode_Request * lhs, const capella_ros_service_interfaces__srv__CartographerMode_Request * rhs);

/// Copy a srv/CartographerMode message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
bool
capella_ros_service_interfaces__srv__CartographerMode_Request__copy(
  const capella_ros_service_interfaces__srv__CartographerMode_Request * input,
  capella_ros_service_interfaces__srv__CartographerMode_Request * output);

/// Initialize array of srv/CartographerMode messages.
/**
 * It allocates the memory for the number of elements and calls
 * capella_ros_service_interfaces__srv__CartographerMode_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
bool
capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence__init(capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence * array, size_t size);

/// Finalize array of srv/CartographerMode messages.
/**
 * It calls
 * capella_ros_service_interfaces__srv__CartographerMode_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
void
capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence__fini(capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence * array);

/// Create array of srv/CartographerMode messages.
/**
 * It allocates the memory for the array and calls
 * capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence *
capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence__create(size_t size);

/// Destroy array of srv/CartographerMode messages.
/**
 * It calls
 * capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
void
capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence__destroy(capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence * array);

/// Check for srv/CartographerMode message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
bool
capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence__are_equal(const capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence * lhs, const capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence * rhs);

/// Copy an array of srv/CartographerMode messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
bool
capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence__copy(
  const capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence * input,
  capella_ros_service_interfaces__srv__CartographerMode_Request__Sequence * output);

/// Initialize srv/CartographerMode message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * capella_ros_service_interfaces__srv__CartographerMode_Response
 * )) before or use
 * capella_ros_service_interfaces__srv__CartographerMode_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
bool
capella_ros_service_interfaces__srv__CartographerMode_Response__init(capella_ros_service_interfaces__srv__CartographerMode_Response * msg);

/// Finalize srv/CartographerMode message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
void
capella_ros_service_interfaces__srv__CartographerMode_Response__fini(capella_ros_service_interfaces__srv__CartographerMode_Response * msg);

/// Create srv/CartographerMode message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * capella_ros_service_interfaces__srv__CartographerMode_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
capella_ros_service_interfaces__srv__CartographerMode_Response *
capella_ros_service_interfaces__srv__CartographerMode_Response__create();

/// Destroy srv/CartographerMode message.
/**
 * It calls
 * capella_ros_service_interfaces__srv__CartographerMode_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
void
capella_ros_service_interfaces__srv__CartographerMode_Response__destroy(capella_ros_service_interfaces__srv__CartographerMode_Response * msg);

/// Check for srv/CartographerMode message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
bool
capella_ros_service_interfaces__srv__CartographerMode_Response__are_equal(const capella_ros_service_interfaces__srv__CartographerMode_Response * lhs, const capella_ros_service_interfaces__srv__CartographerMode_Response * rhs);

/// Copy a srv/CartographerMode message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
bool
capella_ros_service_interfaces__srv__CartographerMode_Response__copy(
  const capella_ros_service_interfaces__srv__CartographerMode_Response * input,
  capella_ros_service_interfaces__srv__CartographerMode_Response * output);

/// Initialize array of srv/CartographerMode messages.
/**
 * It allocates the memory for the number of elements and calls
 * capella_ros_service_interfaces__srv__CartographerMode_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
bool
capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence__init(capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence * array, size_t size);

/// Finalize array of srv/CartographerMode messages.
/**
 * It calls
 * capella_ros_service_interfaces__srv__CartographerMode_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
void
capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence__fini(capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence * array);

/// Create array of srv/CartographerMode messages.
/**
 * It allocates the memory for the array and calls
 * capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence *
capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence__create(size_t size);

/// Destroy array of srv/CartographerMode messages.
/**
 * It calls
 * capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
void
capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence__destroy(capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence * array);

/// Check for srv/CartographerMode message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
bool
capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence__are_equal(const capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence * lhs, const capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence * rhs);

/// Copy an array of srv/CartographerMode messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_capella_ros_service_interfaces
bool
capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence__copy(
  const capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence * input,
  capella_ros_service_interfaces__srv__CartographerMode_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CAPELLA_ROS_SERVICE_INTERFACES__SRV__DETAIL__CARTOGRAPHER_MODE__FUNCTIONS_H_
