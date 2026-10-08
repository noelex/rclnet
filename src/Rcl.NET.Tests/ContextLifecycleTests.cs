namespace Rcl.NET.Tests;

public class ContextLifecycleTests
{
    [Fact]
    public async Task ExternalDisposePreservesAcceptedCallbacks()
    {
        using var context = new RclContext(TestConfig.DefaultContextArguments);
        var executed = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        context.SynchronizationContext.Post(_ => executed.SetResult(), null);

        context.Dispose();

        await executed.Task;
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
            await checkpoint.Entered;
            var first = context.DisposeAsync().AsTask();
            var second = context.DisposeAsync().AsTask();
            Assert.False(first.IsCompleted);
            Assert.False(second.IsCompleted);
            checkpoint.Resume();
            await Task.WhenAll(first, second);
        }
        finally
        {
            checkpoint.Resume();
            await context.DisposeAsync().AsTask();
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
            Assert.True(await returned.Task);
        }
        finally
        {
            await context.DisposeAsync().AsTask();
        }
    }
}
