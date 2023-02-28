using Rcl;
using Rcl.Qos;
using Rosidl.Messages.Rosgraph;

const int resolution = 10_000_000; // ns

var scale = args.Length > 0 && double.TryParse(args[0], out var r) ? r : 1.0;

using var ctx = new RclContext(args);
using var node = ctx.CreateNode("fake_clock");

using var clockPub = node.CreatePublisher<Clock>("/clock", new(qos: QosProfile.Clock));
using var cts = new CancellationTokenSource();

var t = Task.Run(async () =>
{
    var current = 0L;
    using var periodicTimer = new PeriodicTimer(TimeSpan.FromMilliseconds(resolution / 1_000_000));
    using var buffer = RosMessageBuffer.Create<Clock>();

    while (!cts.IsCancellationRequested)
    {
        current += (long)(resolution * scale);
        UpdateClock(buffer, current);
        clockPub.Publish(buffer);
        await periodicTimer.WaitForNextTickAsync(cts.Token);
    }

    static void UpdateClock(RosMessageBuffer buffer, long time)
    {
        ref var clock = ref buffer.AsRef<Clock.Priv>();
        clock.Clock_.Sec = (int)(time / 1_000_000_000);
        clock.Clock_.Nanosec = (uint)(time % 1_000_000_000);
    }
});

Console.ReadLine();
cts.Cancel();

// Make sure the clock loop is completed before exit,
// otherwise we may hit segmentation fault.
await Task.WhenAny(t);