using Rcl.Logging;
using Rcl.Qos;
using Rosidl.Messages.Rosgraph;
using System.Diagnostics;

namespace Rcl.NET.Tests;

public class ClockTests
{
    private static async Task GenerateClockAsync(double scale = 1.0, CancellationToken cancellationToken = default)
    {
        const int resolution = 1_000_000;

        await using var context = new RclContext();
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var clockPub = node.CreatePublisher<Clock>("/clock", new(qos: QosProfile.Clock));

        var current = 0L;
        using var periodicTimer = context.CreateTimer(TimeSpan.FromMilliseconds(resolution / 1_000_000.0));
        using var buffer = RosMessageBuffer.Create<Clock>();

        while (!cancellationToken.IsCancellationRequested)
        {
            UpdateClock(buffer, current);
            clockPub.Publish(buffer);
            await periodicTimer.WaitOneAsync(false, cancellationToken).ConfigureAwait(false);
            current += (long)(resolution * scale);
        }

        static void UpdateClock(RosMessageBuffer buffer, long time)
        {
            ref var clock = ref buffer.AsRef<Clock.Priv>();
            clock.Clock_.Sec = (int)(time / 1_000_000_000);
            clock.Clock_.Nanosec = (uint)(time % 1_000_000_000);
        }
    }

    [Theory]
    [InlineData(0.5, 500, 1000, 300)]
    [InlineData(1, 500, 500, 300)]
    [InlineData(2, 1000, 500, 300)]
    public Task TestCancellationTokenSourceWithRosClock(double scale, int rosTime, int actualTime, double tol)
    {
        return Task.Run(async () =>
        {
            using var clockCancellation = new CancellationTokenSource();
            await using var ctx = new RclContext(TestConfig.DefaultContextArguments);

            var task = GenerateClockAsync(scale, clockCancellation.Token);

            using var node = ctx.CreateNode(NameGenerator.GenerateNodeName(),
                options: new(arguments: new[] { "--ros-args", "-p", "use_sim_time:=true" }));

            var sw = Stopwatch.StartNew();
            try
            {
                using var cts = new CancellationTokenSource();
                cts.CancelAfter(10_000);
                using var reg = cts.CancelAfter(rosTime, node);

                await Task.Delay(-1, cts.Token);
            }
            catch (OperationCanceledException)
            {
                sw.Stop();
                Assert.Equal(actualTime, sw.ElapsedMilliseconds, tol);
            }
            finally
            {
                clockCancellation.Cancel();
                await Task.WhenAny(task).WaitAsync(TimeSpan.FromSeconds(5));
            }
        });
    }
}
