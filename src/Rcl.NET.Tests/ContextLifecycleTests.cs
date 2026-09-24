namespace Rcl.NET.Tests;

public class ContextLifecycleTests
{
    [Fact]
    public void ExternalDisposeDrainsAcceptedCallbacks()
    {
        using var context = new RclContext(TestConfig.DefaultContextArguments);
        bool executed = false;
        context.SynchronizationContext.Post(_ => executed = true, null);

        context.Dispose();

        Assert.True(executed);
        Assert.True(context.DisposeAsync().IsCompletedSuccessfully);
        context.Dispose();
    }

    [Fact]
    public async Task DisposeAsyncWaitsForActiveCallback()
    {
        using var checkpoint = new LifecycleCheckpoint();
        var context = new RclContext(TestConfig.DefaultContextArguments);
        context.SynchronizationContext.Post(_ => checkpoint.Pause(), null);
        try
        {
            await checkpoint.Entered.WaitAsync(TimeSpan.FromSeconds(10));
            var first = context.DisposeAsync().AsTask();
            var second = context.DisposeAsync().AsTask();
            Assert.False(first.IsCompleted);
            Assert.False(second.IsCompleted);
            checkpoint.Resume();
            await Task.WhenAll(first, second).WaitAsync(TimeSpan.FromSeconds(10));
            Assert.False(checkpoint.TimedOut);
        }
        finally
        {
            checkpoint.Resume();
            await context.DisposeAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(10));
        }
    }

    [Fact]
    public async Task EventLoopDisposeDoesNotWaitForItself()
    {
        var context = new RclContext(TestConfig.DefaultContextArguments);
        var returned = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
        context.SynchronizationContext.Post(_ =>
        {
            context.Dispose();
            returned.SetResult(context.IsCurrent && !context.DisposeAsync().IsCompleted);
        }, null);
        try
        {
            Assert.True(await returned.Task.WaitAsync(TimeSpan.FromSeconds(10)));
        }
        finally
        {
            await context.DisposeAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(10));
        }
    }
}
