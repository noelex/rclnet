using Rcl.Utils;

namespace Rcl.NET.Tests;

public class PendingOperationPoolTests
{
    [Fact]
    public async Task ReuseWaitsForConsumptionAndRejectsOldValueTask()
    {
        var pending = PendingOperation<decimal>.Rent(false, static (p, error) => p.Fail(error));
        var task = pending.Task;
        pending.FinishSetup();
        pending.Succeed(42);
        var other = PendingOperation<decimal>.Rent(false, static (p, error) => p.Fail(error));
        Assert.NotSame(pending, other);
        Assert.Equal(42, await task);

        var reused = PendingOperation<decimal>.Rent(false, static (p, error) => p.Fail(error));
        Assert.Same(pending, reused);
        await Assert.ThrowsAsync<InvalidOperationException>(async () => await task);
        Assert.False(reused.Task.IsCompleted);
        reused.FinishSetup();
        reused.Succeed(43);
        Assert.Equal(43, await reused.Task);
        other.FinishSetup();
        other.Succeed(44);
        Assert.Equal(44, await other.Task);
    }

    [Fact]
    public async Task InlineConsumerCannotRecycleWhileCancellationCallbackStillUsesOperation()
    {
        using var cancellation = new CancellationTokenSource();
        using var checkpoint = new LifecycleCheckpoint();
        var pending = PendingOperation<long>.Rent(false, (p, error) =>
        {
            p.Fail(error);
            checkpoint.Pause();
            Assert.False(p.Succeed(1));
        });
        var task = pending.Task.AsTask();
        pending.SetupCancellation(cancellation.Token, Timeout.InfiniteTimeSpan);
        pending.FinishSetup();
        var canceling = Task.Run(cancellation.Cancel);

        try
        {
            await checkpoint.Entered.WaitAsync(TimeSpan.FromSeconds(10));
            await Assert.ThrowsAsync<OperationCanceledException>(() => task);
            var next = PendingOperation<long>.Rent(false, static (p, error) => p.Fail(error));
            Assert.NotSame(pending, next);
            next.FinishSetup();
            next.Succeed(2);
            Assert.Equal(2, await next.Task);
        }
        finally
        {
            checkpoint.Resume();
            await canceling.WaitAsync(TimeSpan.FromSeconds(10));
        }

        Assert.False(checkpoint.TimedOut);
    }

#if DEBUG
    [Fact(Skip = "Allocation assertions require a Release build.")]
#else
    [Fact]
#endif
    public async Task WarmOperationsDoNotAllocate()
    {
        for (int i = 0; i < 100; i++)
        {
            await Complete();
        }

        var before = GC.GetAllocatedBytesForCurrentThread();

        for (int i = 0; i < 1000; i++)
        {
            await Complete();
        }

        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);

        static ValueTask<int> Complete()
        {
            var pending = PendingOperation<int>.Rent(false, static (p, error) => p.Fail(error));
            var task = pending.Task;
            pending.FinishSetup();
            pending.Succeed(1);
            return task;
        }
    }
}
