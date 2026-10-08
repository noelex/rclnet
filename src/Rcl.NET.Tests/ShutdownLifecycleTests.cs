using Rcl.Internal;
using Rcl.Interop;
using Rcl.SafeHandles;
using System.Reflection;
using static Rcl.Interop.RclCommon;

namespace Rcl.NET.Tests;

public class ShutdownLifecycleTests
{
    [Fact]
    public async Task ConcurrentSynchronousDisposeWaitsForSharedCompletion()
    {
        var context = NewContext();
        using var active = new LifecycleCheckpoint();
        context.SynchronizationContext.Post(_ => active.Pause(), null);
        await active.Entered;
        var closing = context.DisposeAsync().AsTask();
        var started = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var synchronous = Task.Run(() =>
        {
            started.SetResult();
            context.Dispose();
        });

        try
        {
            await started.Task;
            Assert.False(closing.IsCompleted);
            Assert.False(synchronous.IsCompleted);
        }
        finally
        {
            active.Resume();
        }

        await Task.WhenAll(closing, synchronous);
        Assert.True(context.Handle.IsClosed);
    }

    [Fact]
    public async Task QueuedContinuationDoesNotHoldShutdownOpen()
    {
        var context = NewContext();
        using var active = new LifecycleCheckpoint();
        using var fallback = new LifecycleCheckpoint();
        var finished = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        bool ranOnLoop = true;
        context.SynchronizationContext.Post(_ => active.Pause(), null);
        await active.Entered;
        context.SynchronizationContext.Post(_ =>
        {
            ranOnLoop = context.IsCurrent;
            fallback.Pause();
            finished.SetResult();
        }, null);

        try
        {
            var closing = context.DisposeAsync().AsTask();
            active.Resume();
            await closing;
            await fallback.Entered;
            Assert.False(ranOnLoop);
            Assert.False(finished.Task.IsCompleted);
        }
        finally
        {
            active.Resume();
            fallback.Resume();
            await finished.Task;
            await context.DisposeAsync();
        }
    }

    [Fact]
    public async Task FaultedLoopPreservesQueuedSendCompletionAndTerminatesWaiters()
    {
        var context = NewContext();
        using var guard = context.CreateGuardCondition();
        var waiting = guard.WaitOneAsync().AsTask();
        using var active = new LifecycleCheckpoint();
        var fault = new InvalidOperationException("Injected event-loop failure.");
        context.SynchronizationContext.Post(_ =>
        {
            active.Pause();
            throw fault;
        }, null);
        await active.Entered;
        bool ranOnLoop = true;
        var send = SendAsync(context, _ => ranOnLoop = context.IsCurrent);
        active.Resume();

        Assert.Same(fault, await Assert.ThrowsAsync<InvalidOperationException>(
            () => context.DisposeAsync().AsTask()));
        await send.AsTask();
        await Assert.ThrowsAsync<ObjectDisposedException>(() => waiting);
        Assert.False(ranOnLoop);
        Assert.False(context.Handle.IsClosed);
        guard.Dispose();
        Assert.True(context.Handle.IsClosed);
        Assert.Same(fault, Assert.Throws<InvalidOperationException>(context.Dispose));
    }

    [Fact]
    public async Task FeatureFailureStillCleansOtherFeaturesAndFaultsAllClosers()
    {
        var context = NewContext();
        var failure = new InvalidOperationException("Injected feature cleanup failure.");
        int calls = 0;
        context.GetOrAddFeature("failing", _ => new Feature(() =>
        {
            Interlocked.Increment(ref calls);
            throw failure;
        }));
        context.GetOrAddFeature("remaining", _ => new Feature(() =>
        {
            context.Dispose();
            Interlocked.Increment(ref calls);
            context.ScheduleCleanup(_ => Interlocked.Increment(ref calls), null);
        }));

        var first = context.DisposeAsync().AsTask();
        var second = context.DisposeAsync().AsTask();
        Assert.Same(first, second);
        Assert.Same(failure, await Assert.ThrowsAsync<InvalidOperationException>(
            () => first));
        Assert.Equal(3, calls);
        Assert.True(context.Handle.IsClosed);
        Assert.Same(failure, Assert.Throws<InvalidOperationException>(context.Dispose));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void ConstructionFailureReleasesInfrastructureAndLogging(bool threadStartup)
    {
        int before = SafeContextHandle.LoggingReferences;
        var handle = new SafeContextHandle(TestConfig.DefaultContextArguments);
        var failure = new InvalidOperationException("Injected infrastructure startup failure.");
        SafeWaitSetHandle? waitSet = null;
        var error = Assert.Throws<InvalidOperationException>(() => new RclContext(handle,
            createWaitSet: context =>
            {
                if (!threadStartup)
                {
                    throw failure;
                }

                return waitSet = new SafeWaitSetHandle(context);
            },
            startThread: _ => throw failure));

        Assert.Same(failure, error);
        Assert.True(handle.IsClosed);
        Assert.Equal(before, SafeContextHandle.LoggingReferences);

        if (threadStartup)
        {
            Assert.True(waitSet!.IsClosed);
        }
    }

    [Fact]
    public async Task NativeShutdownFailureFaultsCompletionWithoutRetryingAtFini()
    {
        int before = SafeContextHandle.LoggingReferences;
        var handle = new FailingShutdownHandle();
        var context = new RclContext(handle);
        using var guard = new SafeGuardConditionHandle(handle);
        await Assert.ThrowsAsync<InvalidOperationException>(
            () => context.DisposeAsync().AsTask());
        Assert.Equal(1, handle.Attempts);
        Assert.False(handle.IsClosed);
        Assert.Equal(before + 1, SafeContextHandle.LoggingReferences);
        Assert.Throws<InvalidOperationException>(context.Dispose);
        guard.Dispose();
        Assert.True(handle.IsClosed);
        Assert.Equal(1, handle.Attempts);
        Assert.Equal(before, SafeContextHandle.LoggingReferences);
        Assert.Contains(HandleReleaseDiagnostics.Snapshot(), e =>
            e?.HandleType == nameof(FailingShutdownHandle) && e.Api == "rcl_shutdown");
    }

    [Fact]
    public async Task NativeWaitCallbackFailureStillDetachesAndShutsDown()
    {
        var context = NewContext();
        using var checkpoint = new LifecycleCheckpoint();
        using var probe = new FaultingGuard(context, checkpoint);
        probe.Trigger();
        await checkpoint.Entered;
        checkpoint.Resume();
        await Assert.ThrowsAsync<InvalidOperationException>(
            () => context.DisposeAsync().AsTask());
        Assert.True(probe.Detached);
        probe.Dispose();
        Assert.True(probe.Handle.IsClosed);
        Assert.True(context.Handle.IsClosed);
    }

    [Fact]
    public async Task CloseCompletesBeforeSynchronousWaiterContinuationReturns()
    {
        var context = NewContext();
        using var guard = context.CreateGuardCondition();
        using var continuation = new LifecycleCheckpoint();
        var finished = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var wait = guard.WaitOneAsync(false);
        await Task.Run(() => wait.GetAwaiter().UnsafeOnCompleted(() =>
        {
            try
            {
                ConsumeClosedWait(wait);
                continuation.Pause();
                finished.SetResult();
            }
            catch (Exception error)
            {
                finished.SetException(error);
            }
        }));

        try
        {
            await context.DisposeAsync().AsTask();
            await continuation.Entered;
            Assert.False(finished.Task.IsCompleted);
        }
        finally
        {
            continuation.Resume();
            await finished.Task;
        }
    }

    [Fact]
    public async Task PostsAndInternalCleanupRacingCloseAreNotLost()
    {
        var context = NewContext();
        int posts = 0, cleanup = 0;
        var allPosts = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        await Task.WhenAll(Task.Run(async () =>
        {
            for (int i = 0; i < 100; i++)
            {
                context.SynchronizationContext.Post(_ =>
                {
                    if (Interlocked.Increment(ref posts) == 100)
                    {
                        allPosts.TrySetResult();
                    }
                }, null);
                context.ScheduleCleanup(_ => Interlocked.Increment(ref cleanup), null);
                await Task.Yield();
            }
        }), Task.Run(async () => await context.DisposeAsync()));
        await allPosts.Task;
        Assert.Equal(100, cleanup);
        Assert.True(context.Handle.IsClosed);
    }

    [Fact]
    public async Task RetainedChildrenKeepEachPhysicalContextsLoggingReference()
    {
        int before = SafeContextHandle.LoggingReferences;
        var first = NewContext();
        var second = NewContext();
        using var child1 = new SafeNodeHandle(first.Handle, NameGenerator.GenerateNodeName(), "/", NodeOptions.Default);
        using var child2 = new SafeNodeHandle(second.Handle, NameGenerator.GenerateNodeName(), "/", NodeOptions.Default);
        await Task.WhenAll(first.DisposeAsync().AsTask(), second.DisposeAsync().AsTask());
        Assert.Equal(before + 2, SafeContextHandle.LoggingReferences);
        Assert.False(first.Handle.IsClosed);
        Assert.False(second.Handle.IsClosed);
        child1.Dispose();
        Assert.True(first.Handle.IsClosed);
        Assert.Equal(before + 1, SafeContextHandle.LoggingReferences);
        child2.Dispose();
        Assert.True(second.Handle.IsClosed);
        Assert.Equal(before, SafeContextHandle.LoggingReferences);
    }

    [Fact]
    public async Task InvalidNativeWaitSetFaultsAndReleasesInfrastructure()
    {
        var started = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var handle = new SafeContextHandle(TestConfig.DefaultContextArguments);
        var context = new RclContext(handle, createWaitSet: nativeContext =>
        {
            var waitSet = new SafeWaitSetHandle(nativeContext);

            unsafe
            {
                RclException.ThrowIfNonSuccess(rcl_wait_set_fini(waitSet.DangerousObject));
            }

            return waitSet;
        }, startThread: thread =>
        {
            thread.Start();
            started.SetResult();
        });
        await started.Task;
        SpinWait.SpinUntil(() => handle.IsClosing);
        await Assert.ThrowsAsync<RclException>(() => context.DisposeAsync().AsTask());
        Assert.True(handle.IsClosed);
    }

    [Fact]
    public async Task SendFailureCompletesCallerWithoutFaultingContext()
    {
        var context = NewContext();
        var failure = new InvalidOperationException("Injected Send callback failure.");
        var send = SendAsync(context, _ => throw failure);
        Assert.Same(failure, await Assert.ThrowsAsync<InvalidOperationException>(() => send.AsTask()));
        await context.DisposeAsync();
        Assert.True(context.Handle.IsClosed);
        int calls = 0;
        context.SynchronizationContext.Send(_ => calls++, null);
        Assert.Equal(1, calls);
    }

    [Fact]
    public async Task WaitSetCleanupFailureFaultsAllClosersAndContinuesCleanup()
    {
        var handle = new SafeContextHandle(TestConfig.DefaultContextArguments);
        FailingWaitSet? waitSet = null;
        var context = new RclContext(handle, createWaitSet: owner => waitSet = new FailingWaitSet(owner));
        int cleanup = 0;
        context.GetOrAddFeature("queued-cleanup", _ => new Feature(() =>
            context.ScheduleCleanup(_ => cleanup++, null)));

        var first = context.DisposeAsync().AsTask();
        var second = context.DisposeAsync().AsTask();
        Assert.Same(first, second);
        var synchronous = Task.Run(() => Assert.Throws<InvalidOperationException>(context.Dispose));
        var failure = await Assert.ThrowsAsync<InvalidOperationException>(() => first);
        Assert.Contains(nameof(FailingWaitSet), failure.Message);
        Assert.Same(failure, await Assert.ThrowsAsync<InvalidOperationException>(() => second));
        Assert.Same(failure, await synchronous);
        Assert.Single(first.Exception!.InnerExceptions);
        Assert.Equal(1, cleanup);
        Assert.True(waitSet!.IsInvalid);
        Assert.True(handle.IsInvalid);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task QueuedCleanupFailureFaultsShutdownAndContinuesDraining(bool duringShutdown)
    {
        var context = NewContext();
        var failure = new InvalidOperationException("Injected queued cleanup failure.");
        int cleanup = 0;

        void QueueCleanup()
        {
            context.ScheduleCleanup(_ => throw failure, null);
            context.ScheduleCleanup(_ => cleanup++, null);
        }

        if (duringShutdown)
        {
            context.GetOrAddFeature("queued-cleanup", _ => new Feature(QueueCleanup));
        }
        else
        {
            QueueCleanup();
            await context.Yield();
        }

        var shutdown = context.DisposeAsync().AsTask();
        Assert.Same(failure, await Assert.ThrowsAsync<InvalidOperationException>(
            () => shutdown));
        Assert.Same(failure, Assert.Throws<InvalidOperationException>(context.Dispose));
        Assert.Equal(1, cleanup);
        Assert.True(context.Handle.IsInvalid);
    }

    [Fact]
    public async Task LateChildCleanupFailureDoesNotChangeShutdownOrFaultAnotherContext()
    {
        var context = NewContext();
        using var child = new FailingGuardHandle(context.Handle);
        var shutdown = context.DisposeAsync().AsTask();
        await shutdown;
        Assert.False(context.Handle.IsClosed);

        child.Dispose();
        Assert.True(child.IsInvalid);
        Assert.True(context.Handle.IsInvalid);
        Assert.Same(shutdown, context.DisposeAsync().AsTask());
        await shutdown;

        var next = NewContext();
        await next.DisposeAsync().AsTask();
    }

    private sealed class FailingWaitSet : SafeWaitSetHandle
    {
        internal FailingWaitSet(SafeContextHandle context) : base(context)
        {
        }

        protected override unsafe bool ReleaseHandleCore(rcl_wait_set_t* ptr)
        {
            // Free real native resources before injecting the fini failure result.
            base.ReleaseHandleCore(ptr);
            return CheckReleaseResult(rcl_ret_t.RCL_RET_ERROR, nameof(rcl_wait_set_fini));
        }
    }

    private sealed class FailingGuardHandle : SafeGuardConditionHandle
    {
        internal FailingGuardHandle(SafeContextHandle context) : base(context)
        {
        }

        protected override unsafe bool ReleaseHandleCore(rcl_guard_condition_t* ptr)
        {
            base.ReleaseHandleCore(ptr);
            return CheckReleaseResult(rcl_ret_t.RCL_RET_ERROR, nameof(rcl_guard_condition_fini));
        }
    }

    private static RclContext NewContext() => new(TestConfig.DefaultContextArguments);

    private static ValueTask SendAsync(RclContext context, SendOrPostCallback callback)
        => (ValueTask)context.SynchronizationContext.GetType().GetMethod("SendAsync", BindingFlags.Public | BindingFlags.Instance)!
            .Invoke(context.SynchronizationContext, new object?[] { callback, null })!;

    private static void ConsumeClosedWait(ValueTask wait)
        => Assert.Throws<ObjectDisposedException>(() => wait.GetAwaiter().GetResult());

    private sealed class Feature(Action dispose) : IDisposable
    {
        public void Dispose() => dispose();
    }

    private sealed class FailingShutdownHandle : SafeContextHandle
    {
        internal int Attempts;

        internal FailingShutdownHandle() : base(TestConfig.DefaultContextArguments)
        {
        }

        protected override rcl_ret_t ShutdownCore()
        {
            Attempts++;
            RclException.ThrowIfNonSuccess(base.ShutdownCore());
            return rcl_ret_t.RCL_RET_ERROR;
        }
    }

    private sealed class FaultingGuard : RclWaitObject<SafeGuardConditionHandle>
    {
        private readonly LifecycleCheckpoint _checkpoint;
        internal bool Detached;

        internal FaultingGuard(RclContext context, LifecycleCheckpoint checkpoint)
            : base(context, new SafeGuardConditionHandle(context.Handle))
        {
            _checkpoint = checkpoint;
            RegisterWaitHandle();
        }

        internal unsafe void Trigger()
        {
            using var lease = Handle.Acquire();
            RclException.ThrowIfNonSuccess(rcl_trigger_guard_condition(lease.Object));
        }

        protected override void OnWaitCompleted()
        {
            _checkpoint.Pause();
            throw new InvalidOperationException("Injected native callback failure.");
        }

        protected override void OnDetached()
        {
            Detached = true;
        }
    }
}
