using Rcl;
using Rosidl.Messages.Geometry;

var r = 2f;
if(args.Length > 0 && float.TryParse(args[0], out var input))
{
    r = input;
}

using var ctx = new RclContext(args);
using var node = ctx.CreateNode("turtle_circling");
using var pub = node.CreatePublisher<Twist>("/turtle1/cmd_vel");

using var cts=new CancellationTokenSource();
_ = Task.Run(async () =>
{
    var twist = new Twist(
        linear: new(x: r),
        angular: new(z: 1.8f)
    );

    while (!cts.IsCancellationRequested)
    {
        pub.Publish(twist);
        await Task.Delay(1000, cts.Token);
    }
});

Console.ReadLine();