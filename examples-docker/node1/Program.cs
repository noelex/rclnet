using Rcl;
using Rcl.Logging;

await using var context = new RclContext(args);
using var node = context.CreateNode("node1");

node.Logger.LogInformation($"Hello ROS 2 {RosEnvironment.Distribution} from Node1!");

using var pub = node.CreatePublisher<Messages.Std.String>("/hello_topic");
using var timer = context.CreateTimer(node.Clock, TimeSpan.FromSeconds(1));

var count = 0;
while (true)
{
    await timer.WaitOneAsync();
    pub.Publish(new Messages.Std.String($"node1 says: Count = {count++}"));
}