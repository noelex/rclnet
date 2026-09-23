using Rcl.Qos;
using Rosidl.Messages.Rosgraph;
using Rosidl.Runtime;

namespace Rcl.NET.Tests;

public class ClockTests
{
    private const int Timeout = 5_000;

    [Fact]
    public async Task TimeProviderPreservesNanosecondTimestamps()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var publisherNode = context.CreateNode(NameGenerator.GenerateNodeName());
        using var clockPublisher = publisherNode.CreatePublisher<Clock>("/clock", new(qos: QosProfile.Clock));
        using var node = context.CreateNode(NameGenerator.GenerateNodeName(),
            options: new(arguments: new[] { "--ros-args", "-p", "use_sim_time:=true" }));

        await WaitForSubscribersAsync(clockPublisher);
        using var buffer = RosMessageBuffer.Create<Clock>();
        var provider = node.TimeProvider;
        Assert.Equal(1_000_000_000L, provider.TimestampFrequency);

        // An epoch-sized value with sub-tick precision exposes floating-point rounding.
        const long initialNanoseconds = 1_700_000_000_000_000_123;
        foreach (var nanoseconds in new[] { initialNanoseconds, initialNanoseconds + 1_000 })
        {
            PublishClock(clockPublisher, buffer, nanoseconds);
            for (var retry = 0; provider.GetTimestamp() != nanoseconds && retry < 500; retry++)
            {
                await Task.Delay(10);
            }

            Assert.Equal(nanoseconds, provider.GetTimestamp());
        }

        Assert.Equal(TimeSpan.FromTicks(10), provider.GetElapsedTime(initialNanoseconds, provider.GetTimestamp()));
    }

    [Fact]
    public async Task CancellationTokenSourceUsesRosClock()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var publisherNode = context.CreateNode(NameGenerator.GenerateNodeName());
        using var clockPublisher = publisherNode.CreatePublisher<Clock>("/clock", new(qos: QosProfile.Clock));
        using var node = context.CreateNode(NameGenerator.GenerateNodeName(),
            options: new(arguments: new[] { "--ros-args", "-p", "use_sim_time:=true" }));

        await AssertCancellationUsesRosTimeAsync(clockPublisher, node, node.Clock);
    }

    [Fact]
    public async Task CancelWithOverrideClock()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var publisherNode = context.CreateNode(NameGenerator.GenerateNodeName());
        using var clockPublisher = publisherNode.CreatePublisher<Clock>("/clock", new(qos: QosProfile.Clock));
        using var clockProducer = context.CreateNode(NameGenerator.GenerateNodeName(),
            options: new(arguments: new[] { "--ros-args", "-p", "use_sim_time:=true" }));
        using var clockConsumer = context.CreateNode(NameGenerator.GenerateNodeName(), clockProducer.Clock);

        await AssertCancellationUsesRosTimeAsync(clockPublisher, clockConsumer, clockProducer.Clock);
    }

    [Fact]
    public async Task TimeProviderUsesRosClockAndThreadPoolCallbacks()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var publisherNode = context.CreateNode(NameGenerator.GenerateNodeName());
        using var clockPublisher = publisherNode.CreatePublisher<Clock>("/clock", new(qos: QosProfile.Clock));
        using var node = context.CreateNode(NameGenerator.GenerateNodeName(),
            options: new(arguments: new[] { "--ros-args", "-p", "use_sim_time:=true" }));

        var initialTime = TimeSpan.FromSeconds(1);
        var timeout = TimeSpan.FromMilliseconds(100);
        await WaitForSubscribersAsync(clockPublisher);
        using var buffer = RosMessageBuffer.Create<Clock>();
        PublishClock(clockPublisher, buffer, initialTime);
        await WaitForClockAsync(node.Clock, initialTime);

        Assert.Same(node.TimeProvider, node.TimeProvider);
        var delay = Task.Delay(timeout, node.TimeProvider);
        var never = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var wait = never.Task.WaitAsync(timeout, node.TimeProvider);
        using var periodicTimer = new PeriodicTimer(timeout, node.TimeProvider);
        var nextTick = periodicTimer.WaitForNextTickAsync().AsTask();
        using var cts = new CancellationTokenSource(timeout, node.TimeProvider);
        var canceled = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        using var cancellationRegistration = cts.Token.Register(() => canceled.TrySetResult());
        var callback = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        using var timer = node.TimeProvider.CreateTimer(
            _ => callback.TrySetResult(context.IsCurrent), null, timeout, System.Threading.Timeout.InfiniteTimeSpan);

        var beforeDeadline = initialTime + timeout - TimeSpan.FromMilliseconds(1);
        PublishClock(clockPublisher, buffer, beforeDeadline);
        await WaitForClockAsync(node.Clock, beforeDeadline);
        Assert.False(delay.IsCompleted);
        Assert.False(wait.IsCompleted);
        Assert.False(nextTick.IsCompleted);
        Assert.False(cts.IsCancellationRequested);
        Assert.False(callback.Task.IsCompleted);

        PublishClock(clockPublisher, buffer, initialTime + timeout);
        await delay.WaitAsync(TimeSpan.FromMilliseconds(Timeout));
        await Assert.ThrowsAsync<TimeoutException>(() => wait);
        Assert.True(await nextTick.WaitAsync(TimeSpan.FromMilliseconds(Timeout)));
        await canceled.Task.WaitAsync(TimeSpan.FromMilliseconds(Timeout));
        Assert.False(await callback.Task.WaitAsync(TimeSpan.FromMilliseconds(Timeout)));
    }

    [Fact]
    public async Task TimeProviderTimerCanChangeDueTimeAndPeriod()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var publisherNode = context.CreateNode(NameGenerator.GenerateNodeName());
        using var clockPublisher = publisherNode.CreatePublisher<Clock>("/clock", new(qos: QosProfile.Clock));
        using var node = context.CreateNode(NameGenerator.GenerateNodeName(),
            options: new(arguments: new[] { "--ros-args", "-p", "use_sim_time:=true" }));

        var initialTime = TimeSpan.FromSeconds(1);
        await WaitForSubscribersAsync(clockPublisher);
        using var buffer = RosMessageBuffer.Create<Clock>();
        PublishClock(clockPublisher, buffer, initialTime);
        await WaitForClockAsync(node.Clock, initialTime);

        var first = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var second = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var calls = 0;
        using var timer = node.TimeProvider.CreateTimer(_ =>
        {
            var count = Interlocked.Increment(ref calls);
            if (count == 1) first.TrySetResult();
            if (count == 2) second.TrySetResult();
        }, null, System.Threading.Timeout.InfiniteTimeSpan, System.Threading.Timeout.InfiniteTimeSpan);

        Assert.True(timer.Change(TimeSpan.FromMilliseconds(100), TimeSpan.FromMilliseconds(50)));
        PublishClock(clockPublisher, buffer, initialTime + TimeSpan.FromMilliseconds(100));
        await first.Task.WaitAsync(TimeSpan.FromMilliseconds(Timeout));

        PublishClock(clockPublisher, buffer, initialTime + TimeSpan.FromMilliseconds(150));
        await second.Task.WaitAsync(TimeSpan.FromMilliseconds(Timeout));
        Assert.True(timer.Change(System.Threading.Timeout.InfiniteTimeSpan, System.Threading.Timeout.InfiniteTimeSpan));
        PublishClock(clockPublisher, buffer, initialTime + TimeSpan.FromSeconds(1));
        await WaitForClockAsync(node.Clock, initialTime + TimeSpan.FromSeconds(1));
        Assert.Equal(2, Volatile.Read(ref calls));
    }

    [Fact]
    public async Task DisposingNodeStopsProviderTimers()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var timer = node.TimeProvider.CreateTimer(
            _ => { }, null, TimeSpan.FromHours(1), System.Threading.Timeout.InfiniteTimeSpan);

        node.Dispose();
        Assert.False(timer.Change(TimeSpan.FromSeconds(1), System.Threading.Timeout.InfiniteTimeSpan));
        await timer.DisposeAsync();
        await context.Yield();
    }

    [Fact]
    public async Task ProviderTimerAndNodeCanBeDisposedConcurrently()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        for (var iteration = 0; iteration < 50; iteration++)
        {
            var node = context.CreateNode(NameGenerator.GenerateNodeName());
            using var timer = node.TimeProvider.CreateTimer(
                _ => { }, null, TimeSpan.FromHours(1), System.Threading.Timeout.InfiniteTimeSpan);
            var start = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
            var disposeTimer = Task.Run(async () =>
            {
                await start.Task;
                timer.Dispose();
            });
            var disposeNode = Task.Run(async () =>
            {
                await start.Task;
                node.Dispose();
            });

            start.SetResult();
            await Task.WhenAll(disposeTimer, disposeNode).WaitAsync(TimeSpan.FromMilliseconds(Timeout));
            await timer.DisposeAsync().AsTask().WaitAsync(TimeSpan.FromMilliseconds(Timeout));
            Assert.False(timer.Change(TimeSpan.FromSeconds(1), System.Threading.Timeout.InfiniteTimeSpan));
            await context.Yield();
        }
    }

    private static async Task AssertCancellationUsesRosTimeAsync(
        IRclPublisher clockPublisher,
        IRclNode timerNode,
        IRclClock clock)
    {
        var initialTime = TimeSpan.FromSeconds(1);
        var timeout = TimeSpan.FromMilliseconds(100);

        await WaitForSubscribersAsync(clockPublisher);

        using var buffer = RosMessageBuffer.Create<Clock>();
        PublishClock(clockPublisher, buffer, initialTime);
        await WaitForClockAsync(clock, initialTime);

        using var cts = new CancellationTokenSource();
#pragma warning disable CS0618 // Keep verifying the deprecated API until it is removed.
        using var timeoutRegistration = cts.CancelAfter(timeout, timerNode);
#pragma warning restore CS0618
        var cancellation = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        using var cancellationRegistration = cts.Token.Register(
            static state => ((TaskCompletionSource)state!).TrySetResult(), cancellation);

        var beforeDeadline = initialTime + timeout - TimeSpan.FromMilliseconds(1);
        PublishClock(clockPublisher, buffer, beforeDeadline);
        await WaitForClockAsync(clock, beforeDeadline);
        Assert.False(cts.IsCancellationRequested);

        PublishClock(clockPublisher, buffer, initialTime + timeout);
        await cancellation.Task.WaitAsync(TimeSpan.FromMilliseconds(Timeout));
    }

    private static void PublishClock(IRclPublisher publisher, RosMessageBuffer buffer, TimeSpan time)
        => PublishClock(publisher, buffer, time.Ticks * 100);

    private static void PublishClock(IRclPublisher publisher, RosMessageBuffer buffer, long nanoseconds)
    {
        var seconds = (int)(nanoseconds / 1_000_000_000);
        var remainder = (uint)(nanoseconds % 1_000_000_000);

        if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
        {
            ref var clock = ref buffer.AsRef<Clock.Priv>();
            clock.Clock_.Sec = seconds;
            clock.Clock_.Nanosec = remainder;
        }
        else
        {
            ref var clock = ref buffer.AsRef<Clock.PrivV2>();
            clock.Clock_.Sec = seconds;
            clock.Clock_.Nanosec = remainder;
        }

        publisher.Publish(buffer);
    }

    private static async Task WaitForSubscribersAsync(IRclPublisher publisher)
    {
        for (var retry = 0; publisher.Subscribers == 0 && retry < 500; retry++)
        {
            await Task.Delay(10);
        }

        Assert.True(publisher.Subscribers > 0, "The clock publisher did not match a subscription.");
    }

    private static async Task WaitForClockAsync(IRclClock clock, TimeSpan expected)
    {
        for (var retry = 0; clock.Elapsed != expected && retry < 500; retry++)
        {
            await Task.Delay(10);
        }

        Assert.Equal(expected, clock.Elapsed);
    }
}
