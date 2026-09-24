using Rosidl.Messages.Builtin;
using Rosidl.Messages.Rcl;
using System.Diagnostics;
using Xunit.Abstractions;

namespace Rcl.NET.Tests;

public class LifecyclePerformanceTests(ITestOutputHelper output)
{
    [Fact]
    [Trait("Category", "PerformanceBaseline")]
    public async Task LifecycleCostBreakdown()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var clock = new RclClock(RclClockType.Steady);
        using var publisher = node.CreatePublisher<Time>("/lifecycle_performance");
        using var client = node.CreateClient<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>("/lifecycle_performance");
        using var buffer = RosMessageBuffer.Create<Time>();
        long clockValue = 0;
        bool available = false;
        Measure("clock-read", 100_000, () => clockValue = clock.Elapsed.Ticks, zeroAllocation: true);
        Measure("client-query", 10_000, () => available = client.IsServerAvailable, zeroAllocation: true);
        Measure("native-publish", 10_000, () => publisher.Publish(buffer), zeroAllocation: true);
        Measure("clock-construct-and-release", 2_000, () =>
        {
            using var owned = new RclClock(RclClockType.Steady);
        });

        for (int sample = 0; sample < 5; sample++)
        {
            var clocks = Enumerable.Range(0, 2000).Select(_ => new RclClock(RclClockType.Steady)).ToArray();
            long allocated = GC.GetAllocatedBytesForCurrentThread();
            long start = Stopwatch.GetTimestamp();

            foreach (var owned in clocks)
            {
                owned.Dispose();
            }

            Report("clock-final-release", sample, start, clocks.Length, GC.GetAllocatedBytesForCurrentThread() - allocated);
        }

        for (int sample = 0; sample < 5; sample++)
        {
            long allocated = GC.GetTotalAllocatedBytes(true);
            long start = Stopwatch.GetTimestamp();

            for (int i = 0; i < 1000; i++)
            {
                using var guard = context.CreateGuardCondition();
            }

            await context.Yield();
            Report("register-unregister-drain-process-allocation", sample, start, 1000,
                GC.GetTotalAllocatedBytes(true) - allocated);
        }

        var latencies = new double[1000];

        for (int i = 0; i < latencies.Length; i++)
        {
            long start = Stopwatch.GetTimestamp();
            await context.Yield();
            latencies[i] = Stopwatch.GetElapsedTime(start).TotalNanoseconds;
        }

        Array.Sort(latencies);
        output.WriteLine($"wait-roundtrip: p50={latencies[500]:F2}, p95={latencies[950]:F2}, p99={latencies[990]:F2} ns");
        GC.KeepAlive(clockValue);
        GC.KeepAlive(available);
    }

    private void Measure(string name, int iterations, Action action, bool zeroAllocation = false)
    {
        for (int i = 0; i < iterations; i++)
        {
            action();
        }

        for (int sample = 0; sample < 5; sample++)
        {
            long allocated = GC.GetAllocatedBytesForCurrentThread();
            long start = Stopwatch.GetTimestamp();

            for (int i = 0; i < iterations; i++)
            {
                action();
            }

            long bytes = GC.GetAllocatedBytesForCurrentThread() - allocated;
            Report(name, sample, start, iterations, bytes);

            if (zeroAllocation)
            {
                Assert.Equal(0, bytes);
            }
        }
    }

    private void Report(string name, int sample, long start, int iterations, long bytes)
    {
        double ns = Stopwatch.GetElapsedTime(start).TotalNanoseconds / iterations;
        output.WriteLine($"{name} sample={sample + 1}: {ns:F2} ns/op, {bytes} bytes/{iterations} operations");
    }
}
