# Specifiy the output directory.
# A path relative to this file or an absolute path can be used.
output ./

# Ignore missing included/depedency packages, proceed with packages currently available anyway.
#
# By default, ros2cs will abort with exit code 1 if any package dependency is missing,
# unless missing packages are excluded explicitly using 'exclude' option.
# Enabling this option forces ros2cs to generate code even if some packages are missing.
#
# This option is disabled by default.
# ignore-missing

# Generate code for service introspection.
# This feature requires 'service_msgs' package.
# The default value is 'off'.
# service-introspection [on|off]

# Generate implementation details for actions, including FeedbackMessage,
# SendGoal request/response and GetResult request/response.
#
# This feature requires 'unique_identifier_msgs' and 'builtin_interfaces' package.
#
# Detail messages are not neccesary for using action servers and clients as rclnet
# utilizes introspection to construct and extract Feedback / Goal and Result messages.
# Enabling 'action-details' will allow you to perform service introspection on Result
# and Goal services, or manually subscribe Feedback messages, at the cost of increased
# code size.
# The default value is 'off'.
# action-details [on|off]

# Generate classes with internal visibility rather than public.
# internal

# Set the default root namespace.
namespace Rosidl.Messages

# Read packages from ament prefix directories as specified by AMENT_PREFIX_PATH.
# If this flag specified, it will always load before all other from-directory directives.
from-ament-index

# Read packages from specified root package directory containing one or more packages.
# A path relative to this file or an absolute path can be used.
# from-directory .

# Generate codes for specified packages only.
# Dependencies of the specified packages will also be included recursively.
# If no include directive is specified, ros2cs will generate codes for all discovered packages.
# include [package]...

# Explicitly exclude packages from code generation.
# exclude [package]...

# Set the root namespace of a specific package,
# e.g. 'map-name my_messages:My.Namespace' will map definitions in package 'my_messages'
# into C# namespace My.Namespace.MyMessages.
# If not specified, default root namespace is used.
# map-namespace [package]:[namespace]

# Set the name of a specific package,
# e.g. 'map-name my_messages:Messages' will map definitions in package 'my_messages'
# into C# namespace Rosidl.Messages.Messages.
# If not specified, ros2cs will determine the package name automatically.
# map-name [package]:[name]