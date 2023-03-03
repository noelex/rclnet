# Specifiy the output directory.
# A path relative to this file or an absolute path can be used.
output ./

# Generate classes with internal visibility rather than public.
# internal

# Set the default root namespace.
namespace Company.MessageLibrary1

# Read packages from ament prefix directories as specified by AMENT_PREFIX_PATH.
# If this flag specified, it will always load before all other from-directory directives.
from-ament-index

# Read packages from specified root package directory containing one or more packages.
# A path relative to this file or an absolute path can be used.
# from-directory .

# Generate codes for specified packages only.
# Dependencies of the specified packages will also be included recursively.
# If no include directive is specified, ros2cs will generate codes for all discovered packages.
# include geometry_msgs

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