using Rcl;
using Rcl.Logging;

await using var context = new RclContext(args);
using var node = context.CreateNode("my_ros2_node");

node.Logger.LogInformation($"Hello ROS 2 {RosEnvironment.Distribution}!");