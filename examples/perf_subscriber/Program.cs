// See https://aka.ms/new-console-template for more information
using Rcl;
using Rcl.Logging;
using Rosidl.Messages.Sensor;
using System.Collections.Concurrent;

var poco = args.Any(x => x == "--use-poco");

var topic = "/test_image";
var opts = new SubscriptionOptions();

await using var ctx = new RclContext(args);
using var node = ctx.CreateNode("perf_subscriber");

node.Logger.LogInformation($"RMW: {RosEnvironment.RmwImplementationIdentifier}");
node.Logger.LogInformation($"Receiving messages using {(poco ? "POCO" : "native")} subscription.");

var queue = new ConcurrentQueue<double>();
using var cts = new CancellationTokenSource();

var subscriberTask = Task.Run(async () =>
{
    if (!poco)
    {
        using var sub = node.CreateNativeSubscription<Image>(topic, opts);
        await foreach (var msg in sub.ReadAllAsync(cts.Token))
        {
            using (msg)
            {
                var received = node.Clock.Elapsed;
                var stamp = msg.AsRef<Image.Priv>().Header.Stamp;
                var sent = TimeSpan.FromSeconds(stamp.Sec + stamp.Nanosec / 1_000_000_000.0);
                queue.Enqueue((received - sent).TotalMilliseconds);
            }
        }
    }
    else
    {
        using var sub = node.CreateSubscription<Image>(topic, opts);
        await foreach (var msg in sub.ReadAllAsync(cts.Token))
        {
            var received = node.Clock.Elapsed;
            var stamp = msg.Header.Stamp;
            var sent = TimeSpan.FromSeconds(stamp.Sec + stamp.Nanosec / 1_000_000_000.0);
            queue.Enqueue((received - sent).TotalMilliseconds);
        }
    }
});

var statPrinterTask = Task.Run(async () =>
{
    using var statTimer = ctx.CreateTimer(node.Clock, TimeSpan.FromSeconds(1));
    while (!cts.IsCancellationRequested)
    {
        await statTimer.WaitOneAsync(true, cts.Token);

        node.Logger.LogInformation("FPS: " + queue.Count);
        if (queue.Count > 0)
        {
            node.Logger.LogInformation("Latency Max: " + queue.Max());
            node.Logger.LogInformation("Latency Min: " + queue.Min());
            node.Logger.LogInformation("Latency Avg: " + queue.Average());

            var histogram = queue.GroupBy(v => (int)Math.Round(v) / 10).OrderBy(x => x.Key).ToDictionary(x => x.Key, x => x.Count());
            node.Logger.LogInformation("Latency Histogram: ");
            foreach (var item in histogram)
            {
                node.Logger.LogInformation($"~ {item.Key * 10 + 10} ms: {item.Value}");
            }
        }
        node.Logger.LogInformation("----------------");

        queue.Clear();
    }
});

Console.ReadLine();
cts.Cancel();

await Task.WhenAll(subscriberTask, statPrinterTask)
    .ContinueWith(t => { });