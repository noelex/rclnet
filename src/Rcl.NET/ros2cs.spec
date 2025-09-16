# rcl_interfaces/msg/ParameterDescriptor has been changed since humble.
# ros2cs this file on humble and manually make a backward
# compatible copy of ParameterDescriptor and friends is required.

output ./Interfaces

internal

from-directory ../../modules/rcl_interfaces
from-directory ../../modules

include rcl_interfaces
include action_msgs
include builtin_interfaces
include unique_identifier_msgs
include rosgraph_msgs