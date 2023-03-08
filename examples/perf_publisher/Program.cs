// See https://aka.ms/new-console-template for more information
using Rcl;
using Rcl.Logging;
using Rosidl.Messages.Sensor;

var frameRate = GetArg(args, new[] { "--fps", "-f" }, x => double.TryParse(x, out var r) ? r : 60);
var frameSize = GetArg(args, new[] { "--frame-size", "-s" }, x => int.TryParse(x, out var r) ? r : 3 * 1920 * 1080);

await using var ctx = new RclContext(args);
using var node = ctx.CreateNode("perf_publisher");
using var pub = node.CreatePublisher<Image>("/test_image");

using var cts = new CancellationTokenSource();

node.Logger.LogInformation($"RMW: {RosEnvironment.RmwImplementationIdentifier}");
node.Logger.LogInformation($"Frame Rate: {frameRate}");
node.Logger.LogInformation($"Frame Size: {frameSize} B");
node.Logger.LogInformation("Publisher is running. Press ENTER to stop publishing.");

var body = new byte[frameSize];
using var nativeMsg = pub.CreateBuffer();
nativeMsg.AsRef<Image.Priv>().Data.CopyFrom(body);

var t = Task.Run(async () =>
{
    using var timer = ctx.CreateTimer(node.Clock, TimeSpan.FromMilliseconds(1000 / frameRate));
    while (!cts.Token.IsCancellationRequested)
    {
        var now = (long)node.Clock.Elapsed.TotalNanoseconds;
        nativeMsg.AsRef<Image.Priv>().Header.Stamp.Sec = (int)(now / 1_000_000_000);
        nativeMsg.AsRef<Image.Priv>().Header.Stamp.Nanosec = (uint)(now % 1_000_000_000);

        pub.Publish(nativeMsg);

        // Doesn't need asynchronous scheduling because the time spent here
        // is near zero, the event loop should be happy with that.
        await timer.WaitOneAsync(false, cts.Token);
    }
});

Console.ReadLine();
cts.Cancel();
await Task.WhenAny(t);

static T GetArg<T>(string[] args, string[] matcher, Func<string?, T> func)
{
    var idx = -1;
    foreach (var m in matcher)
    {
        var i = Array.IndexOf(args, m);
        if (i >= 0 && i < args.Length - 1 && i > idx)
        {
            idx = i;
        }
    }

    return func(idx < 0 ? null : args[idx + 1]);
}