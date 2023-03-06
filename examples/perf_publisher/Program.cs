// See https://aka.ms/new-console-template for more information
using Rcl;
using Rcl.Logging;
using Rosidl.Messages.Sensor;

await using var ctx = new RclContext(args);
using var node = ctx.CreateNode("perf_publisher");
using var pub = node.CreatePublisher<Image>("/test_image");

using var cts=new CancellationTokenSource();

_ = Task.Run(() =>
{
    Console.ReadLine();
    cts.Cancel();
});

node.Logger.LogInformation("Publisher is running. Press ENTER to stop publishing.");

var body = new byte[3 * 1920 * 1080];
var nativeMsg = RosMessageBuffer.Create<Image>();
nativeMsg.AsRef<Image.Priv>().Data.CopyFrom(body);

using var timer = ctx.CreateTimer(node.Clock, TimeSpan.FromMilliseconds(10));
while (!cts.Token.IsCancellationRequested)
{
    var now = (long)node.Clock.Elapsed.TotalNanoseconds;
    nativeMsg.AsRef<Image.Priv>().Header.Stamp.Sec = (int)(now / 1_000_000_000);
    nativeMsg.AsRef<Image.Priv>().Header.Stamp.Nanosec = (uint)(now % 1_000_000_000);

    pub.Publish(nativeMsg);
    await timer.WaitOneAsync(cts.Token);
}