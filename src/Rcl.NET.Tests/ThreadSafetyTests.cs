using System.Xml.Linq;
using Xunit.Abstractions;

namespace Rcl.NET.Tests;

public class ThreadSafetyTests
{
    private static readonly int s_concurrency = Math.Max(2, Environment.ProcessorCount);

    [SkippableTheory]
    [InlineData(RclClockType.Steady, false)]
    [InlineData(RclClockType.System, false)]
    [InlineData(RclClockType.Ros, false)]
    [InlineData(RclClockType.Steady, true)]
    [InlineData(RclClockType.System, true)]
    [InlineData(RclClockType.Ros, true)]
    public async Task ConcurrentTimerCreationAndDisposal_MultipleContexts(RclClockType clockType, bool synchronousDispose)
    {
        // Foxy / Cyclone blocks in native DDS entity deletion during concurrent context teardown.
        // The pre-rewrite implementation also reproduces it; native Linux stacks wait inside
        // dds_delete_impl_pinned from rcl_service_fini/rcl_node_fini. Keep other Linux scenarios enabled.
        // Related teardown-hang report (not confirmed to be the same internal defect):
        // https://github.com/ros2/rmw_cyclonedds/issues/104
        Skip.If(RosEnvironment.IsFoxy && RosEnvironment.RmwImplementationIdentifier == "rmw_cyclonedds_cpp",
            "Foxy / Cyclone DDS: native concurrent teardown hangs.");

        // A standalone C++ RCL program with independent contexts and endpoints also crashes on
        // Humble and Iron / Fast DDS, without .NET or SafeHandle. Humble's native stack identifies
        // a null call in StatefulWriter::deliver_sample_to_intraprocesses during endpoint teardown.
        // Restrict the exclusion to this overlapping multi-context endpoint scenario.
        Skip.If((RosEnvironment.IsHumble || RosEnvironment.IsIron)
            && RosEnvironment.RmwImplementationIdentifier == "rmw_fastrtps_cpp",
            "Humble/Iron / Fast DDS: native concurrent endpoint teardown crashes.");

        // Both managed tests and a standalone C++ RCL reproduction abort in Lyrical / Cyclone
        // at ddsi_fini's ddsrt_avl_is_empty(&gv->typelib) assertion when contexts close concurrently.
        // Context-only and single-context endpoint tests remain enabled.
        Skip.If(RosEnvironment.IsLyrical && RosEnvironment.RmwImplementationIdentifier == "rmw_cyclonedds_cpp",
            "Lyrical / Cyclone DDS: native concurrent teardown assertion fails.");

        await RunConcurrently(async index =>
        {
            var context = new RclContext(TestConfig.DefaultContextArguments);

            try
            {
                using var node = context.CreateNode(NameGenerator.GenerateNodeName());
                await CreateTimerWaitAndDisposeAsync(node, clockType, index % 4 + 1);
            }
            finally
            {
                await RclContext.YieldBackground();
                Assert.False(context.IsCurrent);

                if (synchronousDispose)
                {
                    context.Dispose();
                }
                else
                {
                    await context.DisposeAsync();
                }
            }
        });
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task ConcurrentContextCreationAndDisposal(bool synchronousDispose)
    {
        await RunConcurrently(async _ =>
        {
            var context = new RclContext(TestConfig.DefaultContextArguments);

            try
            {
                await Task.Yield();
            }
            finally
            {
                if (synchronousDispose)
                {
                    context.Dispose();
                }
                else
                {
                    await context.DisposeAsync();
                }
            }
        });
    }

    [Theory]
    [InlineData(RclClockType.Steady)]
    [InlineData(RclClockType.System)]
    [InlineData(RclClockType.Ros)]
    public async Task ConcurrentTimerCreationAndDisposal_SingleContext(RclClockType clockType)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        await RunConcurrently(index => CreateTimerWaitAndDisposeAsync(node, clockType, index % 4 + 1));
    }

    [Fact]
    public async Task ConcurrentGuardConditionCreation_SingleContext()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        await RunConcurrently(index => CreateGuardConditionWaitAndDisposeAsync(context, index % 4 + 1));
    }

    private static async Task RunConcurrently(Func<int, Task> action)
    {
        var start = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var workers = Enumerable.Range(0, s_concurrency).Select(index => Task.Run(async () =>
        {
            await start.Task;
            await action(index);
        })).ToArray();

        start.SetResult();
        await Task.WhenAll(workers).WaitAsync(TimeSpan.FromSeconds(30));
    }

    private static async Task CreateTimerWaitAndDisposeAsync(IRclNode node, RclClockType type, int timeout)
    {
        var clock = type switch
        {
            RclClockType.Steady => RclClock.SteadyClock,
            RclClockType.System => RclClock.SystemClock,
            RclClockType.Ros => node.Clock,
            _ => throw new NotSupportedException()
        };

        using var cts = new CancellationTokenSource(1000);
        using var timer = node.Context.CreateTimer(clock, TimeSpan.FromMilliseconds(timeout));
        await timer.WaitOneAsync(cts.Token);
    }

    private static async Task CreateGuardConditionWaitAndDisposeAsync(IRclContext context, int timeout)
    {
        using var cts = new CancellationTokenSource(1000);
        using var gc = context.CreateGuardCondition();
        _ = Task.Delay(timeout).ContinueWith(x => gc.Trigger());
        await gc.WaitOneAsync(cts.Token);
    }
}

static class LinqExtensions
{
    public static IEnumerable<T> Merge<T>(this IEnumerable<T> src, IEnumerable<T> other)
    {
        using var sourceEnumerator = src.GetEnumerator();
        using var otherEnumerator = other.GetEnumerator();

        bool continueSource = true, continueOther = true;
        do
        {
            if (continueSource)
            {
                if (sourceEnumerator.MoveNext())
                {
                    yield return sourceEnumerator.Current;
                }
                else
                {
                    continueSource = false;
                }
            }

            if (continueOther)
            {
                if (otherEnumerator.MoveNext())
                {
                    yield return otherEnumerator.Current;
                }
                else
                {
                    continueOther = false;
                }
            }

        } while (continueSource || continueOther);
    }
}
