using Rcl;
using Rosidl.Messages.Turtlesim;

using var ctx = new RclContext(args);
using var node = ctx.CreateNode("turtle_pose");
using var sub = node.CreateNativeSubscription<Pose>("/turtle1/pose");

using var cts = new CancellationTokenSource();
_ = Task.Run(async () =>
{
    await foreach(var buffer in sub.ReadAllAsync(cts.Token))
    {
        using(buffer) ProcessMessage(buffer);
    }

    static void ProcessMessage(RosMessageBuffer buffer)
    {
        ref var msg = ref buffer.AsRef<Pose.Priv>();
        Console.WriteLine($"X: {msg.X}, Y: {msg.Y}, Theta: {msg.Theta}");
    }
});

Console.ReadLine();