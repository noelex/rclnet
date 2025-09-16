output ./Interfaces

internal
service-introspection on
action-details on

from-directory ../../modules/geometry2
from-directory ../../modules/common_interfaces
from-directory ../../modules/rcl_interfaces
from-directory ../../modules

include std_msgs geometry_msgs sensor_msgs tf2_msgs
exclude builtin_interfaces unique_identifier_msgs