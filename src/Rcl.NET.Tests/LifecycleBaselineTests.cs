using System.Diagnostics;
using Xunit.Abstractions;

namespace Rcl.NET.Tests;

public class LifecycleBaselineTests
{
    private readonly ITestOutputHelper _output;

    public LifecycleBaselineTests(ITestOutputHelper output) => _output = output;

    [Fact]
    [Trait("Category", "PerformanceBaseline")]
    public void SteadyClockReadBaseline()
    {
        using var clock = new RclClock(RclClockType.Steady);
        const int iterations = 100_000;
        long ticks = 0;

        for (int i = 0; i < iterations; i++)
        {
            ticks = clock.Elapsed.Ticks;
        }

        for (int sample = 0; sample < 5; sample++)
        {
            long allocated = GC.GetAllocatedBytesForCurrentThread();
            long start = Stopwatch.GetTimestamp();

            for (int i = 0; i < iterations; i++)
            {
                ticks = clock.Elapsed.Ticks;
            }

            double nsPerCall = Stopwatch.GetElapsedTime(start).TotalNanoseconds / iterations;
            long bytes = GC.GetAllocatedBytesForCurrentThread() - allocated;
            _output.WriteLine($"Sample {sample + 1}: {nsPerCall:F2} ns/call, {bytes} bytes/{iterations} calls");
        }

        GC.KeepAlive(ticks);
    }
}
