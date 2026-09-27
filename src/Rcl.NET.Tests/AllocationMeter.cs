using System.Diagnostics;
using Xunit.Abstractions;

namespace Rcl.NET.Tests;

internal sealed class AllocationMeter(ITestOutputHelper output)
{
    public const int SampleCount = 5;

    public async Task MeasureAsync(string name, int iterations, Func<ValueTask> action)
    {
        for (int i = 0; i < iterations; i++)
        {
            await action();
        }

        // Includes producer, consumer and scheduling allocations across threads, plus background process noise.
        for (int sample = 0; sample < SampleCount; sample++)
        {
            long allocated = GC.GetTotalAllocatedBytes(true);
            long start = Stopwatch.GetTimestamp();

            for (int i = 0; i < iterations; i++)
            {
                await action();
            }

            Report(name, sample, Stopwatch.GetTimestamp() - start, iterations, GC.GetTotalAllocatedBytes(true) - allocated);
        }
    }

    public void Measure(string name, int iterations, Action action, bool zeroAllocation = false)
    {
        for (int i = 0; i < iterations; i++)
        {
            action();
        }

        for (int sample = 0; sample < SampleCount; sample++)
        {
            long allocated = GC.GetAllocatedBytesForCurrentThread();
            long start = Stopwatch.GetTimestamp();

            for (int i = 0; i < iterations; i++)
            {
                action();
            }

            long bytes = GC.GetAllocatedBytesForCurrentThread() - allocated;
            Report(name, sample, Stopwatch.GetTimestamp() - start, iterations, bytes);

            if (zeroAllocation)
            {
                Assert.Equal(0, bytes);
            }
        }
    }

    public void Report(string name, int sample, long elapsedTicks, int iterations, long bytes)
    {
        double ns = elapsedTicks * 1e9 / Stopwatch.Frequency / iterations;
        output.WriteLine($"{name} sample={sample + 1}: {ns:F2} ns/op, {bytes / (double)iterations:F2} B/op ({bytes} bytes/{iterations} operations)");
    }
}
