using Rcl.Utils;

namespace Rcl.NET.Tests;

public class PendingOperationTimeoutTests
{
    [Fact]
    public async Task TimeoutDuringSetupDisposesTimerAfterSetupFinishes()
    {
        var provider = new TestTimeProvider(fireDuringCreation: true);
        var pending = new PendingOperation<int>(false, static (operation, error) => operation.Fail(error));
        var task = pending.Task.AsTask();
        pending.SetupCancellation(default, TimeSpan.Zero, provider);
        await Assert.ThrowsAsync<TimeoutException>(() => task);
        Assert.False(provider.Timer.Disposed);
        pending.FinishSetup();
        Assert.True(provider.Timer.Disposed);
    }

    [Fact]
    public async Task LateTimeoutCannotCompleteAnotherOperation()
    {
        var provider = new TestTimeProvider(fireDuringCreation: false);
        var pending = new PendingOperation<int>(false, static (operation, error) => operation.Fail(error));
        pending.SetupCancellation(default, TimeSpan.FromSeconds(1), provider);
        pending.FinishSetup();
        pending.Succeed(42);
        Assert.Equal(42, await pending.Task);
        Assert.True(provider.Timer.Disposed);

        var next = new PendingOperation<int>(false, static (operation, error) => operation.Fail(error));
        next.FinishSetup();
        // A timer callback already in flight may still arrive after Dispose.
        provider.Timer.Fire();
        Assert.False(next.Task.IsCompleted);
        next.Succeed(43);
        Assert.Equal(43, await next.Task);
    }

    private sealed class TestTimeProvider(bool fireDuringCreation) : TimeProvider
    {
        public TestTimer Timer { get; private set; } = null!;

        public override ITimer CreateTimer(TimerCallback callback, object? state, TimeSpan dueTime, TimeSpan period)
        {
            Assert.Equal(Timeout.InfiniteTimeSpan, period);
            Timer = new TestTimer(callback, state);

            if (fireDuringCreation)
            {
                Timer.Fire();
            }

            return Timer;
        }
    }

    private sealed class TestTimer(TimerCallback callback, object? state) : ITimer
    {
        public bool Disposed { get; private set; }

        public void Fire()
        {
            callback(state);
        }

        public bool Change(TimeSpan dueTime, TimeSpan period)
        {
            throw new NotSupportedException();
        }

        public void Dispose()
        {
            Disposed = true;
        }

        public ValueTask DisposeAsync()
        {
            Dispose();
            return ValueTask.CompletedTask;
        }
    }
}
