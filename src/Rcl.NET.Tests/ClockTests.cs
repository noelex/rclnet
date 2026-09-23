using Rcl.Qos;
using Rosidl.Messages.Rosgraph;
using Rosidl.Runtime;

namespace Rcl.NET.Tests;

public class ClockTests
{
    private const int Timeout = 5_000;

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
        using var timeoutRegistration = cts.CancelAfter(timeout, timerNode);
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
    {
        var nanoseconds = time.Ticks * 100;
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
