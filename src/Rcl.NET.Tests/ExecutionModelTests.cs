using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.NET.Tests;
public class ExecutionModelTests
{
    [Fact]
    public async Task WithSynchronizationContextEnabled()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments, useSynchronizationContext: true);
        Assert.False(context.IsCurrent);

        await context.Yield();
        Assert.True(context.IsCurrent);

        await Task.Yield();
        Assert.True(context.IsCurrent);

        await Task.Delay(1);
        Assert.True(context.IsCurrent);

        await Task.Run(() =>
        {
            Assert.False(context.IsCurrent);
        });
        Assert.True(context.IsCurrent);

        await Task.Delay(1).ConfigureAwait(false);
        Assert.False(context.IsCurrent);

        await Task.Yield();
        Assert.False(context.IsCurrent);

        await context.Yield();
        Assert.True(context.IsCurrent);
    }

    [Fact]
    public async Task WithSynchronizationContextDisabled()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments, useSynchronizationContext: false);
        Assert.False(context.IsCurrent);

        await context.Yield();
        Assert.True(context.IsCurrent);

        await Task.Yield();
        Assert.False(context.IsCurrent);

        await Task.Delay(1);
        Assert.False(context.IsCurrent);

        await Task.Run(() =>
        {
            Assert.False(context.IsCurrent);
        });
        Assert.False(context.IsCurrent);

        await Task.Delay(1).ConfigureAwait(false);
        Assert.False(context.IsCurrent);

        await Task.Yield();
        Assert.False(context.IsCurrent);

        await context.Yield();
        Assert.True(context.IsCurrent);
    }

    [Fact]
    public Task WaitOneShouldResumeOnEventLoopIfSynchronizationContextIsNotCaptured()
    {
        // Run the test in a thread pool thread so that we won't have any SynchronizationContext.
        return Task.Run(async () =>
        {
            await using var context = new RclContext(TestConfig.DefaultContextArguments);
            using var gc = context.CreateGuardCondition();

            // This will resume execution on the event loop.
            var t1 = gc.WaitOneAsync(false).ConfigureAwait(false);
            _ = Task.Delay(1).ContinueWith(t => gc.Trigger());
            await t1;

            Assert.True(context.IsCurrent);
        });
    }

    [Fact]
    public Task WaitOneShouldNotResumeOnEventLoopIfSynchronizationContextIsNotCapturedAndRunContinuationAsynchronouslyIsTrue()
    {
        // Run the test in a thread pool thread so that we won't have any SynchronizationContext.
        return Task.Run(async () =>
        {
            await using var context = new RclContext(TestConfig.DefaultContextArguments);
            using var gc = context.CreateGuardCondition();

            // This prevents us from resuming on the event loop.
            var t1 = gc.WaitOneAsync(true);
            _ = Task.Delay(1).ContinueWith(t => gc.Trigger());
            await t1;

            Assert.False(context.IsCurrent);

            // Doesn't matter whether we ConfigureAwait(false) or not as no SynchronizationContext is captured.
            var t2 = gc.WaitOneAsync(true).ConfigureAwait(false);
            _ = Task.Delay(1).ContinueWith(t => gc.Trigger());
            await t2;

            Assert.False(context.IsCurrent);
        });
    }

    [Fact]
    public async Task WaitOneWithSynchronizationContext()
    {
        await using var context1 = new RclContext(TestConfig.DefaultContextArguments, useSynchronizationContext: true);

        await context1.Yield();

        // Now we're on the event loop of context1, with SynchronizationContext.Current set to its SynchronizationContext.

        await using var context2 = new RclContext(TestConfig.DefaultContextArguments);
        using var gc = context2.CreateGuardCondition();

        // This will keep us on the event loop of context1 due to the captured sync context.
        var t1 = gc.WaitOneAsync(false);
        _ = Task.Delay(1).ContinueWith(t => gc.Trigger());
        await t1;

        Assert.True(context1.IsCurrent);

        // This will bring us back to the event loop of context2 because we explicitly ignore the sync context.
        var t2 = gc.WaitOneAsync(false).ConfigureAwait(false);
        _ = Task.Delay(1).ContinueWith(t => gc.Trigger());
        await t2;

        // No sync context at this point.
        Assert.True(context2.IsCurrent);

        // Get back to context1, we have sync context of context1 again.
        await context1.Yield();

        // Doesn't matter whether runContinuationAsynchronously is true or false,
        // as when a sync context is captured, continuations will always
        // execute asynchronously on captured sync context.
        var t3 = gc.WaitOneAsync(true);
        _ = Task.Delay(1).ContinueWith(t => gc.Trigger());
        await t3;

        // Thus still on context1.
        Assert.True(context1.IsCurrent);

        // Does not capture the context, with runContinuationAsynchronously set to true.
        // This will break us out from both context1 and context2, and resume execution
        // on a thread pool thread.
        var t4 = gc.WaitOneAsync(true).ConfigureAwait(false);
        _ = Task.Delay(1).ContinueWith(t => gc.Trigger());
        await t4;

        Assert.False(context1.IsCurrent);
        Assert.False(context2.IsCurrent);
    }

    [Fact]
    public async Task ContinuationOfYieldIfNotCurrent()
    {
        await using var ctx1 = new RclContext(
            TestConfig.DefaultContextArguments, useSynchronizationContext: true);
        await using var ctx2 = new RclContext(
            TestConfig.DefaultContextArguments);

        await ctx2.YieldIfNotCurrent();
        Assert.True(ctx2.IsCurrent);

        await ctx1.Yield();
        Assert.True(ctx1.IsCurrent);

        await ctx2.YieldIfNotCurrent();
        Assert.True(ctx2.IsCurrent);

        await ctx2.YieldIfNotCurrent();
        Assert.True(ctx2.IsCurrent);
    }

    [Fact]
    public async Task YieldAndYieldBackground()
    {
        await using var ctx1 = new RclContext(
            TestConfig.DefaultContextArguments, useSynchronizationContext: true);
        await using var ctx2 = new RclContext(
            TestConfig.DefaultContextArguments);

        await ctx1.Yield();
        Assert.True(ctx1.IsCurrent);

        // Yield ignores the sync context.
        await ctx2.Yield();
        Assert.True(ctx2.IsCurrent);

        await ctx1.Yield();
        Assert.True(ctx1.IsCurrent);

        // YieldBackground also ignores the sync context.
        await RclContext.YieldBackground();
        Assert.False(ctx1.IsCurrent);
        Assert.False(ctx2.IsCurrent);
    }
}
