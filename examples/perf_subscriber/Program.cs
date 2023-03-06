// See https://aka.ms/new-console-template for more information
using Rcl;
using Rcl.Logging;
using Rosidl.Messages.Sensor;
using System.Collections.Concurrent;

await using var ctx = new RclContext(args);
using var node = ctx.CreateNode("perf_subscriber1");
using var sub = node.CreateNativeSubscription<Image>("/test_image", new SubscriptionOptions(queueSize: 128));

using var cts = new CancellationTokenSource();
_ = Task.Run(() =>
{
    Console.ReadLine();
    cts.Cancel();
});

var queue = new ConcurrentQueue<double>();

_ = Task.Run(async () =>
{
    using var statTimer = new PeriodicTimer(TimeSpan.FromSeconds(1));
    while (!cts.IsCancellationRequested)
    {
        await statTimer.WaitForNextTickAsync(cts.Token);
        PrintStats(queue);
    }
});
await foreach (var msg in sub.ReadAllAsync(cts.Token))
{
    using (msg) Enqueue(node.Clock, queue, msg);
}

void Enqueue(RclClock clock, ConcurrentQueue<double> queue, RosMessageBuffer msg)
{
    var received = clock.Elapsed;
    var stamp = msg.AsRef<Image.Priv>().Header.Stamp;
    var sent = TimeSpan.FromSeconds(stamp.Sec + stamp.Nanosec / 1_000_000_000.0);

    queue.Enqueue((received - sent).TotalMilliseconds);
}

void PrintStats(ConcurrentQueue<double> queue)
{
    node.Logger.LogInformation("FPS: " + queue.Count);
    if (queue.Count > 0)
    {
        node.Logger.LogInformation("Max: " + queue.Max());
        node.Logger.LogInformation("Min: " + queue.Min());
        node.Logger.LogInformation("Avg: " + queue.Average());

        var histogram = queue.GroupBy(v => (int)Math.Round(v) / 10).OrderBy(x => x.Key).ToDictionary(x => x.Key, x => x.Count());
        node.Logger.LogInformation("Histogram: ");
        foreach (var item in histogram)
        {
            node.Logger.LogInformation($"~ {item.Key * 10 + 10} ms: {item.Value}");
        }
    }
    node.Logger.LogInformation("----------------");

    queue.Clear();
}