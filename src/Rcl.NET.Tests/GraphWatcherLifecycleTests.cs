using Rcl.Graph;
using Rcl.Utils;
using System.Reflection;

namespace Rcl.NET.Tests;

public class GraphWatcherLifecycleTests
{
    [Theory]
    [InlineData(-0.5)]
    [InlineData(-1.5)]
    [InlineData(4294967294.5)]
    public async Task InvalidTimeSpanTimeoutIsRejectedWithoutTruncation(double milliseconds)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var cancellation = new CancellationTokenSource();
        try
        {
            var waiting = node.Graph.TryWatchAsync(static (_, _) => false,
                TimeSpan.FromMilliseconds(milliseconds), cancellation.Token);
            var error = await Assert.ThrowsAsync<ArgumentOutOfRangeException>(() =>
                waiting);
            Assert.Equal("timeout", error.ParamName);
        }
        finally
        {
            cancellation.Cancel();
        }
    }

    [Fact]
    public async Task InfiniteAndZeroTimeSpanTimeoutsKeepTheirSemantics()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var cancellation = new CancellationTokenSource();
        await context.Yield();
        var waiting = node.Graph.TryWatchAsync(static (_, _) => false,
            TimeSpan.FromMilliseconds(-1), cancellation.Token);
        Assert.False(waiting.IsCompleted);
        cancellation.Cancel();
        var error = await Assert.ThrowsAsync<OperationCanceledException>(() =>
            waiting);
        Assert.Equal(cancellation.Token, error.CancellationToken);
        Assert.False(await node.Graph.TryWatchAsync(static (_, _) => false, TimeSpan.Zero));
    }

    [Theory]
    [InlineData(Timeout.Infinite)]
    [InlineData(10_000)]
    public async Task GraphCompletionTerminatesPendingWatch(int timeout)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        var graph = new RosGraph((Rcl.Internal.RclNodeImpl)node, static _ => true);
        await context.Yield();
        var waiting = graph.TryWatchAsync(static (_, _) => false, timeout);
        graph.Complete();
        await Assert.ThrowsAsync<ObjectDisposedException>(() => waiting);
    }

    [Fact]
    public async Task UnsubscriptionKeepsInFlightNotificationAliveAndRejectsLateNotifications()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var checkpoint = new LifecycleCheckpoint();
        var graph = node.Graph;
        var pending = PendingOperation<bool>.Rent(true, static (p, error) => p.Fail(error));
        var calls = 0;
        Func<RosGraph, RosGraphEvent?, object?, bool> predicate = (_, _, _) =>
        {
            Interlocked.Increment(ref calls);
            checkpoint.Pause();
            return true;
        };
        // Capture an observer independently of the dictionary, as an in-flight dispatch can.
        var watcherType = typeof(RosGraph).GetNestedType("GraphWatcher", BindingFlags.NonPublic)!;
        var observer = (IObserver<RosGraphEvent>)Activator.CreateInstance(watcherType, graph, predicate, null, pending)!;
        using var watcher = (IDisposable)observer;
        pending.FinishSetup();
        var change = new NodeAppearedEvent(graph, new RosNode(new NodeName("test", "/"), "/", new SnapshotPublisher()));
        var dispatch = Task.Run(() => observer.OnNext(change));

        try
        {
            await checkpoint.Entered;
            pending.Fail(new OperationCanceledException());
            await Assert.ThrowsAsync<OperationCanceledException>(async () => await pending.Task);
            watcher.Dispose();
            var next = PendingOperation<bool>.Rent(true, static (p, error) => p.Fail(error));
            Assert.NotSame(pending, next);
            checkpoint.Resume();
            await dispatch;
            observer.OnNext(change);
            observer.OnCompleted();
            Assert.Equal(1, calls);
            Assert.False(next.Task.IsCompleted);
            next.FinishSetup();
            next.Succeed(true);
            Assert.True(await next.Task);
        }
        finally
        {
            checkpoint.Resume();
            await dispatch;
        }
    }

    [Fact]
    public async Task InitialMatchZeroTimeoutAndCancellationKeepTheirSemantics()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        Assert.True(await node.Graph.TryWatchAsync(static (_, _) => true, -2, cancellation.Token));
        Assert.False(await node.Graph.TryWatchAsync(static (_, _) => false, 0));
        var error = await Assert.ThrowsAsync<OperationCanceledException>(() =>
            node.Graph.TryWatchAsync(static (_, _) => false, 0, cancellation.Token));
        Assert.Equal(cancellation.Token, error.CancellationToken);
        await Assert.ThrowsAsync<ArgumentOutOfRangeException>(() =>
            node.Graph.TryWatchAsync(static (_, _) => false, -2));
    }
}
