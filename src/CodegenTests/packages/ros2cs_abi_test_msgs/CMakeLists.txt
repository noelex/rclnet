cmake_minimum_required(VERSION 3.10)
project(ros2cs_abi_test_msgs)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundedPrimitiveSequence.msg"
  "msg/ComplexSequence.msg"
  "msg/FixedComplexArray.msg"
  "msg/NameCollision.msg"
  "msg/Nested.msg"
  "msg/PrimitiveSequence.msg"
  "msg/Scalar.msg"
  "msg/StringSequence.msg"
  "srv/SequenceService.srv"
  "action/Sequence.action"
  DEPENDENCIES action_msgs builtin_interfaces unique_identifier_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
