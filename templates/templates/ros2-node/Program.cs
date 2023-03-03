// TODO: Edit ros2cs.spec to include ROS 2 interface packages you wish to use,
// and run 'dotnet ros2cs' in the project root to generate messages.
//
// See https://github.com/noelex/rclnet for more information.
using Rcl;
using Rcl.Logging;

using var context = new RclContext(args);
using var node = context.CreateNode("my_ros2_node");

node.Logger.LogInformation($"Hello ROS 2 {RosEnvironment.Distribution}!");