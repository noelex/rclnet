using Rcl;
using Rcl.Logging;
using Rosidl.Messages.Turtlesim;

await using var ctx = new RclContext(args);
using var node = ctx.CreateNode("turtle_pose");
using var sub = node.CreateNativeSubscription<Pose>("/turtle1/pose");

using var cts = new CancellationTokenSource();
_ = Task.Run(async () =>
{
    await foreach (var buffer in sub.ReadAllAsync(cts.Token))
    {
        using (buffer) ProcessMessage(node, buffer);
    }

    static void ProcessMessage(IRclNode node, RosMessageBuffer buffer)
    {
        ref var msg = ref buffer.AsRef<Pose.Priv>();
        node.Logger.LogInformation($"X: {msg.X}, Y: {msg.Y}, Theta: {msg.Theta}");
    }
});

Console.ReadLine();