using Rcl.Qos;
using Rosidl.Messages.Rosgraph;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.NET.Tests;
public class ClockTest
{
    private static readonly SemaphoreSlim _concurrencyLimit = new(1);

    private string[] NodeArgs = new[] { "--ros-args", "-p", "use_sim_time:=true", "--log-level", "debug" };

    private static async Task GenerateClockAsync(RclContext context, double scale = 1.0, CancellationToken cancellationToken = default)
    {
        const int resolution = 10_000_000;

        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var clockPub = node.CreatePublisher<Clock>("/clock", new(qos: QosProfile.Clock));

        var current = 0L;
        using var periodicTimer = new PeriodicTimer(TimeSpan.FromMilliseconds(resolution / 1_000_000.0));
        using var buffer = RosMessageBuffer.Create<Clock>();

        while (!cancellationToken.IsCancellationRequested)
        {
            UpdateClock(buffer, current);
            clockPub.Publish(buffer);
            await periodicTimer.WaitForNextTickAsync(cancellationToken);
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
    [InlineData(0.5, 500, 1000, 50)]
    [InlineData(1, 1000, 1000, 50)]
    [InlineData(2, 1000, 500, 50)]
    public async Task TestCancellationTokenSourceWithRosClock(double scale, int rosTime, int actualTime, double tol)
    {
        // We don't want /clock topics interfere each other.
        await _concurrencyLimit.WaitAsync();
        Debug.WriteLine("Starting with scale " + scale);
        try
        {
            using var clockCancellation = new CancellationTokenSource();
            using var ctx = new RclContext();
            var task = GenerateClockAsync(ctx, scale, clockCancellation.Token);
            using var node = ctx.CreateNode(NameGenerator.GenerateNodeName(), options: new(arguments: NodeArgs));

            var sw = Stopwatch.StartNew();
            try
            {
                using var cts = new CancellationTokenSource();
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
                await Task.WhenAny(task);
            }
        }
        finally
        {
            Debug.WriteLine("Exiting with scale " + scale);
            _concurrencyLimit.Release();
        }
    }
}
