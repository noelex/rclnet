using Rcl;
using Rcl.Logging;

await using var context = new RclContext(args);
using var node = context.CreateNode("node2");

node.Logger.LogInformation($"Hello ROS 2 {RosEnvironment.Distribution} from Node2!");

using var sub = node.CreateSubscription<Messages.Std.String>("/hello_topic");

await foreach(var msg in sub.ReadAllAsync())
{
    node.Logger.LogInformation(msg.Data);
}