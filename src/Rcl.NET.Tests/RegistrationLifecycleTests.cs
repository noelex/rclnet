using Rcl.Internal;
using Rcl.Internal.Services;
using Rcl.Internal.Subscriptions;
using Rcl.SafeHandles;
using Rcl.Utils;
using Rosidl.Messages.Builtin;
using Rosidl.Messages.Rcl;
using System.Collections.Concurrent;
using System.Reflection;
using System.Text;
using static Rcl.Interop.RclCommon;

namespace Rcl.NET.Tests;

public class RegistrationLifecycleTests : IDisposable
{
    private readonly HandleReleaseError?[] _errorsBefore = HandleReleaseDiagnostics.Snapshot();

    public void Dispose() => Assert.Empty(HandleReleaseDiagnostics.Snapshot().Except(_errorsBefore));

    [Fact]
    public async Task FailedBatchPublicationRollsBackBeforeDispatch()
    {
        await using var context = NewContext();
        using var first = new Probe(context);
        using var closed = new Probe(context);
        var callbacks = 0;
        first.Callback = () => Interlocked.Increment(ref callbacks);
        Trigger(first.Handle);
        closed.Dispose();

        Assert.Throws<ObjectDisposedException>(() =>
            RclWaitObject<SafeGuardConditionHandle>.RegisterWaitHandles(context, first, null, closed));

        first.Dispose();
        Assert.True(first.Handle.IsClosed);
        Assert.Equal(1, first.NativeReleases);
        Assert.Equal(1, first.DetachCount);
        await context.Yield();
        Assert.Equal(0, Volatile.Read(ref callbacks));
    }

    [Fact]
    public async Task BatchPublicationIsVisibleToImmediateContextClose()
    {
        var context = NewContext();
        using var first = new Probe(context);
        using var second = new Probe(context);

        try
        {
            RclWaitObject<SafeGuardConditionHandle>.RegisterWaitHandles(context, first, null, second);
            var shutdown = context.DisposeAsync().AsTask();

            await shutdown.WaitAsync(TimeSpan.FromSeconds(10));
            Assert.Equal(1, first.DetachCount);
            Assert.Equal(1, second.DetachCount);
        }
        finally
        {
            await context.DisposeAsync();
        }
    }

    [Fact]
    public async Task CloseBeforePublicationRejectsRegistrationAndReturnsRefs()
    {
        await using var context = NewContext();
        using var probe = new Probe(context);
        probe.Dispose();
        Assert.IsType<ObjectDisposedException>(Record.Exception(probe.Publish));
        await probe.Detached.Task.WaitAsync(TimeSpan.FromSeconds(10));
        await probe.NativeReleased.WaitAsync(TimeSpan.FromSeconds(10));
        Assert.True(probe.Handle.IsClosed);
        Assert.Equal(1, probe.DetachCount);
        Assert.Equal(1, probe.NativeReleases);
    }

    [Fact]
    public async Task PublishedRegistrationIsVisibleToImmediateClose()
    {
        await using var context = NewContext();
        using var probe = new Probe(context);

        probe.Publish();
        probe.Dispose();
        probe.Handle.Dispose();

        await probe.Detached.Task.WaitAsync(TimeSpan.FromSeconds(10));
        await probe.NativeReleased.WaitAsync(TimeSpan.FromSeconds(10));
        Assert.True(probe.Handle.IsClosed);
        Assert.Equal(1, probe.NativeReleases);
        Assert.Equal(1, probe.DetachCount);
    }

    [Fact]
    public async Task RegistrationPinsNativeStorageAcrossWaitIterations()
    {
        await using var context = NewContext();
        using var handle = new TrackedGuard(context.Handle);
        WaitHandleRegistration registration = default;
        context.Register(handle, static (_, _) =>
        {
        }, null, ref registration);
        handle.Dispose();

        for (var i = 0; i < 3; i++)
        {
            await context.Yield();
            Assert.False(handle.IsClosed);
        }

        registration.Dispose();
        registration.Dispose();
        // This continuation is on the event loop. Await release instead of blocking its cleanup.
        await handle.Released.Task.WaitAsync(TimeSpan.FromSeconds(10));
        Assert.True(handle.IsClosed);
        Assert.Equal(1, handle.Releases);
    }

    [Fact]
    public async Task DuplicateNativeEntryIsRejectedWithoutDroppingTheOriginalRef()
    {
        await using var context = NewContext();
        using var handle = new TrackedGuard(context.Handle);
        WaitHandleRegistration first = default, duplicate = default;
        context.Register(handle, static (_, _) =>
        {
        }, null, ref first);
        Assert.Throws<InvalidOperationException>(() => context.Register(handle, static (_, _) =>
        {
        }, null, ref duplicate));
        Assert.True(duplicate.IsEmpty);
        handle.Dispose();
        Assert.False(handle.IsClosed);
        first.Dispose();
        await handle.Released.Task.WaitAsync(TimeSpan.FromSeconds(10));
        Assert.True(handle.IsClosed);
        Assert.Equal(1, handle.Releases);
    }

    [Fact]
    public async Task OldTokenCannotRemoveNewRegistrationAtTheSameAddress()
    {
        await using var context = NewContext();
        using var handle = new TrackedGuard(context.Handle);
        using var checkpoint = new LifecycleCheckpoint();
        WaitHandleRegistration oldRegistration = default, newRegistration = default;
        context.Register(handle, (_, _) => checkpoint.Pause(), null, ref oldRegistration);
        var received = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        Trigger(handle);

        try
        {
            await checkpoint.Entered.WaitAsync(TimeSpan.FromSeconds(10));
            oldRegistration.Dispose();
            context.Register(handle, (_, _) => received.TrySetResult(), null, ref newRegistration);
            Assert.NotEqual(oldRegistration.Entry!.Token, newRegistration.Entry!.Token);
            oldRegistration.Dispose();
            Assert.False(oldRegistration.Entry.Detached);
            Assert.False(newRegistration.Entry.RemoveRequested);
            Trigger(handle);
        }
        finally
        {
            checkpoint.Resume();
        }

        await received.Task.WaitAsync(TimeSpan.FromSeconds(10));
        newRegistration.Dispose();
        await LifecycleAssert.EventuallyAsync(() =>
            Volatile.Read(ref oldRegistration.Entry!.Detached) && Volatile.Read(ref newRegistration.Entry!.Detached));
        Assert.False(checkpoint.TimedOut);
    }

    [Fact]
    public async Task RegistrationRejectsAnotherContextWithoutRetainingTheHandle()
    {
        await using var first = NewContext();
        await using var second = NewContext();
        using var handle = new TrackedGuard(first.Handle);
        WaitHandleRegistration registration = default;
        Assert.Throws<InvalidOperationException>(() => second.Register(handle, static (_, _) =>
        {
        }, null, ref registration));
        Assert.True(registration.IsEmpty);
        handle.Dispose();
        Assert.True(handle.IsClosed);
        Assert.Equal(1, handle.Releases);
    }

    [Fact]
    public async Task CallbackCanDisposeItselfWithoutReleasingItsStorageOrResources()
    {
        await using var context = NewContext();
        using var checkpoint = new LifecycleCheckpoint();
        using var probe = new Probe(context);
        probe.Callback = () =>
        {
            probe.Dispose();
            checkpoint.Pause();
        };
        probe.Publish();
        Trigger(probe.Handle);

        try
        {
            await checkpoint.Entered.WaitAsync(TimeSpan.FromSeconds(10));
            Assert.True(probe.Handle.IsClosing);
            Assert.False(probe.Handle.IsClosed);
            Assert.Equal(0, probe.DetachCount);
            Assert.Equal(0, probe.NativeReleases);
        }
        finally
        {
            checkpoint.Resume();
        }

        await probe.Detached.Task.WaitAsync(TimeSpan.FromSeconds(10));
        await probe.NativeReleased.WaitAsync(TimeSpan.FromSeconds(10));
        Assert.True(probe.Handle.IsClosed);
        Assert.Equal(1, probe.NativeReleases);
        Assert.Equal(1, probe.DetachCount);
    }

    [Fact]
    public async Task MultipleWaitersShareOneNativeRegistration()
    {
        await using var context = NewContext();
        using var guard = context.CreateGuardCondition();
        var first = guard.WaitOneAsync().AsTask();
        var second = guard.WaitOneAsync().AsTask();
        guard.Trigger();
        await Task.WhenAll(first, second).WaitAsync(TimeSpan.FromSeconds(10));
    }

    [Fact]
    public async Task PreCanceledWaitsDoNotLoseCancellationOrPoisonLaterWaits()
    {
        await using var context = NewContext();
        using var guard = context.CreateGuardCondition();
        using var canceled = new CancellationTokenSource();
        canceled.Cancel();

        for (var i = 0; i < 100; i++)
        {
            var wait = guard.WaitOneAsync(canceled.Token).AsTask();
            var error = await Assert.ThrowsAsync<OperationCanceledException>(() => wait.WaitAsync(TimeSpan.FromSeconds(10)));
            Assert.Equal(canceled.Token, error.CancellationToken);
        }

        var next = guard.WaitOneAsync().AsTask();
        guard.Trigger();
        await next.WaitAsync(TimeSpan.FromSeconds(10));
    }

    [Fact]
    public async Task ContextCloseTerminatesRetainedWaitersAndRequests()
    {
        await using var context = NewContext();
        using var guard = context.CreateGuardCondition();
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = node.CreateClient<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(NameGenerator.GenerateServiceName());
        var wait = guard.WaitOneAsync().AsTask();
        var request = client.InvokeAsync(new ListParametersServiceRequest(), Timeout.Infinite);
        await context.DisposeAsync();
        await Assert.ThrowsAsync<ObjectDisposedException>(() => wait.WaitAsync(TimeSpan.FromSeconds(10)));
        await Assert.ThrowsAsync<ObjectDisposedException>(() => request.WaitAsync(TimeSpan.FromSeconds(10)));
    }

    [Fact]
    public async Task WaitSignalCancellationAndCloseHaveOneTerminalWinner()
    {
        await using var context = NewContext();

        for (var i = 0; i < 40; i++)
        {
            using var guard = context.CreateGuardCondition();
            using var cancellation = new CancellationTokenSource();
            var wait = guard.WaitOneAsync(cancellation.Token).AsTask();
            var start = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
            var signal = Task.Run(async () =>
            {
                await start.Task;

                try
                {
                    guard.Trigger();
                }
                catch (ObjectDisposedException)
                {
                }
            });
            var cancel = Task.Run(async () =>
            {
                await start.Task;
                cancellation.Cancel();
            });
            var close = Task.Run(async () =>
            {
                await start.Task;
                guard.Dispose();
            });
            start.SetResult();
            await Task.WhenAll(signal, cancel, close).WaitAsync(TimeSpan.FromSeconds(10));

            try
            {
                await wait.WaitAsync(TimeSpan.FromSeconds(10));
            }
            catch (Exception error) when (error is OperationCanceledException or ObjectDisposedException)
            {
            }
        }
    }

    [Fact]
    public async Task CompletionSourceIsNotRecycledDuringSynchronousCancellationSetup()
    {
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        var pending = new PendingOperation<int>(false, (p, error) => p.Fail(error));
        var source = Source(pending);
        var version = source.Version;
        var task = pending.Task.AsTask();
        pending.SetupCancellation(cancellation.Token, Timeout.InfiniteTimeSpan);
        await Assert.ThrowsAsync<OperationCanceledException>(() => task);
        Assert.Equal(version, source.Version);
        pending.FinishSetup();
        Assert.NotEqual(version, source.Version);
        Assert.False(pending.Fail(new Exception("Late callback")));
    }

    [Fact]
    public async Task RequestCancellationCanRaceWithResponsesWithoutPoisoningLaterRequests()
    {
        await using var context = NewContext();
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        var name = NameGenerator.GenerateServiceName();
        int calls = 0;
        using var service = node.CreateService<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(
            name, (request, state) =>
            {
                Interlocked.Increment(ref calls);
                return new ListParametersServiceResponse();
            });
        using var client = node.CreateClient<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(name);
        Assert.True(await client.TryWaitForServerAsync(10_000));
        using var canceled = new CancellationTokenSource();
        canceled.Cancel();
        await Assert.ThrowsAsync<OperationCanceledException>(() => client.InvokeAsync(new ListParametersServiceRequest(), canceled.Token));
        await Assert.ThrowsAsync<ArgumentOutOfRangeException>(() => client.InvokeAsync(new ListParametersServiceRequest(), -2));
        await client.InvokeAsync(new ListParametersServiceRequest(), 10_000);
        Assert.Equal(1, Volatile.Read(ref calls));

        for (var i = 0; i < 40; i++)
        {
            using var cancellation = new CancellationTokenSource();
            var response = client.InvokeAsync(new ListParametersServiceRequest(), 10_000, cancellation.Token);
            await Task.Run(cancellation.Cancel);

            try
            {
                await response.WaitAsync(TimeSpan.FromSeconds(10));
            }
            catch (OperationCanceledException)
            {
            }
        }

        await client.InvokeAsync(new ListParametersServiceRequest(), 10_000);
    }

    [Fact]
    public async Task CompletionSourceIsNotRecycledBeforeWinningProducerReturns()
    {
        using var checkpoint = new LifecycleCheckpoint();
        var pending = new PendingOperation<int>(false, (p, error) => p.Fail(error));
        var source = Source(pending);
        var version = source.Version;
        int value = 0;
        // Register without the test runner's synchronization context so completion is inline.
        await Task.Run(() => pending.Task.GetAwaiter().UnsafeOnCompleted(() =>
        {
            value = ReadCompleted(pending);
            checkpoint.Pause();
        }));
        pending.FinishSetup();
        var completing = Task.Run(() => pending.Succeed(42));

        try
        {
            await checkpoint.Entered.WaitAsync(TimeSpan.FromSeconds(10));
            Assert.Equal(version, source.Version);
            Assert.False(pending.Fail(new OperationCanceledException()));
        }
        finally
        {
            checkpoint.Resume();
        }

        Assert.True(await completing.WaitAsync(TimeSpan.FromSeconds(10)));
        Assert.Equal(42, value);
        Assert.NotEqual(version, source.Version);
    }

    [Fact]
    public async Task LateCallbacksCannotCompleteAReusedSource()
    {
        var old = new PendingOperation<Guid>(false, (p, error) => p.Fail(error));
        var source = Source(old);
        old.FinishSetup();
        old.Succeed(Guid.NewGuid());
        await old.Task;
        var current = new PendingOperation<Guid>(false, (p, error) => p.Fail(error));
        Assert.Same(source, Source(current));
        current.FinishSetup();
        Assert.False(old.Fail(new OperationCanceledException()));
        Assert.False(current.Task.IsCompleted);
        var expected = Guid.NewGuid();
        current.Succeed(expected);
        Assert.Equal(expected, await current.Task);
    }

    [Fact]
    public async Task CleanupEnqueuedWhileDrainingIsCompletedBeforeStop()
    {
        await using var context = NewContext();
        var calls = new ConcurrentQueue<int>();
        context.ScheduleCleanup(_ =>
        {
            calls.Enqueue(1);
            context.ScheduleCleanup(_ => calls.Enqueue(2), null);
        }, null);
        await context.DisposeAsync();
        Assert.Equal(new[] { 1, 2 }, calls);
        context.ScheduleCleanup(_ => calls.Enqueue(3), null);
        Assert.Equal(new[] { 1, 2, 3 }, calls);
    }

    [Fact]
    public async Task TypedSubscriptionCompletionWaitsForItsSelfDisposingObserver()
    {
        await using var context = NewContext();
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        var topic = NameGenerator.GenerateTopicName();
        using var publisher = node.CreatePublisher<Time>(topic);
        using var subscription = node.CreateSubscription<Time>(topic);
        using var checkpoint = new LifecycleCheckpoint();
        var completed = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        using var observer = subscription.Subscribe(new Observer<Time>(value =>
        {
            subscription.Dispose();
            checkpoint.Pause();
        }, () => completed.TrySetResult()));
        Assert.True(SpinWait.SpinUntil(() => publisher.Subscribers > 0, TimeSpan.FromSeconds(10)));
        publisher.Publish(new Time(1, 2));

        try
        {
            await checkpoint.Entered.WaitAsync(TimeSpan.FromSeconds(10));
            Assert.False(completed.Task.IsCompleted);
            Assert.False(((RclSubscription<Time>)subscription).Handle.IsClosed);
        }
        finally
        {
            checkpoint.Resume();
        }

        await completed.Task.WaitAsync(TimeSpan.FromSeconds(10));
        Assert.False(checkpoint.TimedOut);
    }

    [Fact]
    public async Task NativeSubscriptionBufferWaitsForDispatchDuringContextClose()
    {
        await using var context = NewContext();
        using var node = (RclNodeImpl)context.CreateNode(NameGenerator.GenerateNodeName());
        var topic = NameGenerator.GenerateTopicName();
        using var checkpoint = new LifecycleCheckpoint();
        using var subscription = new PausedSubscription(node, topic, checkpoint);
        using var publisher = node.CreatePublisher<Time>(topic);
        Assert.True(SpinWait.SpinUntil(() => publisher.Subscribers > 0, TimeSpan.FromSeconds(10)));
        publisher.Publish(new Time(3, 4));
        Task shutdown;

        try
        {
            await checkpoint.Entered.WaitAsync(TimeSpan.FromSeconds(10));
            subscription.Dispose();
            shutdown = context.DisposeAsync().AsTask();
            Assert.False(shutdown.IsCompleted);
            Assert.Equal(0, subscription.Destroyed);
        }
        finally
        {
            checkpoint.Resume();
        }

        await shutdown.WaitAsync(TimeSpan.FromSeconds(10));
        Assert.Equal(1, subscription.Destroyed);
        Assert.False(checkpoint.TimedOut);
    }

    [Fact]
    public async Task AsynchronousServiceOwnsBuffersAndCancellationUntilHandlerExits()
    {
        await using var context = NewContext();
        using var node = (RclNodeImpl)context.CreateNode(NameGenerator.GenerateNodeName());
        var name = NameGenerator.GenerateServiceName();
        var entered = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var resume = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var checkedBuffers = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        using var service = new TrackingService(node, name, new DelegateConcurrentNativeServiceCallHandler(async (request, response, _, token) =>
        {
            entered.TrySetResult();
            await resume.Task;

            try
            {
                Assert.True(token.WaitHandle.WaitOne(0));
                _ = ListParametersServiceRequest.CreateFrom(request.Data, Encoding.UTF8);
                new ListParametersServiceResponse().WriteTo(response.Data, Encoding.UTF8);
                checkedBuffers.SetResult();
            }
            catch (Exception error)
            {
                checkedBuffers.SetException(error);
            }
        }, null));
        using var client = node.CreateClient<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(name);
        Assert.True(await client.TryWaitForServerAsync(10_000));
        var pending = client.InvokeAsync(new ListParametersServiceRequest(), Timeout.Infinite);

        try
        {
            await entered.Task.WaitAsync(TimeSpan.FromSeconds(10));
            await context.DisposeAsync();
            await Assert.ThrowsAsync<ObjectDisposedException>(() => pending);
            Assert.Equal(0, service.Destroyed);
        }
        finally
        {
            resume.TrySetResult();
        }

        await checkedBuffers.Task.WaitAsync(TimeSpan.FromSeconds(10));
        await LifecycleAssert.EventuallyAsync(() => Volatile.Read(ref service.Destroyed) == 2);
    }

    [Fact]
    public async Task TimerDisposeAsyncWaitsForAdmittedThreadPoolCallback()
    {
        await using var context = NewContext();
        using var provider = new RclTimeProvider(context, RclClock.SteadyClock);
        using var checkpoint = new LifecycleCheckpoint();
        using var timer = provider.CreateTimer(_ => checkpoint.Pause(), null, TimeSpan.Zero, Timeout.InfiniteTimeSpan);
        Task disposing;

        try
        {
            await checkpoint.Entered.WaitAsync(TimeSpan.FromSeconds(10));
            disposing = timer.DisposeAsync().AsTask();
            await context.Yield();
            Assert.False(disposing.IsCompleted);
        }
        finally
        {
            checkpoint.Resume();
        }

        await disposing.WaitAsync(TimeSpan.FromSeconds(10));
        Assert.False(checkpoint.TimedOut);
    }

    private static RclContext NewContext() => new(TestConfig.DefaultContextArguments);

    private static T ReadCompleted<T>(PendingOperation<T> pending) => pending.Task.GetAwaiter().GetResult();

    private static ManualResetValueTaskSource<T> Source<T>(PendingOperation<T> pending)
        => (ManualResetValueTaskSource<T>)typeof(PendingOperation<T>).GetField("_source", BindingFlags.Instance | BindingFlags.NonPublic)!.GetValue(pending)!;

    private static unsafe void Trigger(SafeGuardConditionHandle handle)
    {
        using var lease = handle.Acquire();
        RclException.ThrowIfNonSuccess(rcl_trigger_guard_condition(lease.Object));
    }

    private sealed class TrackedGuard : SafeGuardConditionHandle
    {
        internal int Releases;
        internal readonly TaskCompletionSource Released = new(TaskCreationOptions.RunContinuationsAsynchronously);

        internal TrackedGuard(SafeContextHandle context) : base(context)
        {
        }

        protected override unsafe bool ReleaseHandleCore(rcl_guard_condition_t* ptr)
        {
            Interlocked.Increment(ref Releases);
            var result = base.ReleaseHandleCore(ptr);
            Released.TrySetResult();
            return result;
        }
    }

    private sealed class Probe : RclWaitObject<SafeGuardConditionHandle>
    {
        internal Action? Callback;
        internal int DetachCount;

        internal int NativeReleases => ((TrackedGuard)Handle).Releases;

        internal Task NativeReleased => ((TrackedGuard)Handle).Released.Task;

        internal readonly TaskCompletionSource Detached = new(TaskCreationOptions.RunContinuationsAsynchronously);

        internal Probe(RclContext context) : base(context, new TrackedGuard(context.Handle))
        {
        }

        internal void Publish() => RegisterWaitHandle();

        protected override void OnWaitCompleted() => Callback?.Invoke();

        protected override void OnDetached()
        {
            Interlocked.Increment(ref DetachCount);
            Detached.TrySetResult();
        }
    }

    private sealed class Observer<T>(Action<T> next, Action completed) : IObserver<T>
    {
        public void OnNext(T value) => next(value);

        public void OnCompleted() => completed();

        public void OnError(Exception error) => throw error;
    }

    private sealed class PausedSubscription : NativeSubscription<Time>
    {
        private readonly LifecycleCheckpoint _checkpoint;
        internal int Destroyed;

        internal PausedSubscription(RclNodeImpl node, string topic, LifecycleCheckpoint checkpoint)
            : base(node, topic, SubscriptionOptions.Default) => _checkpoint = checkpoint;

        protected override RosMessageBuffer TakeMessage()
        {
            var buffer = base.TakeMessage();

            if (buffer.IsEmpty)
            {
                return buffer;
            }

            _checkpoint.Pause();
            return new(buffer.Data, (_, _) =>
            {
                buffer.Dispose();
                Interlocked.Increment(ref Destroyed);
            });
        }
    }

    private sealed class TrackingService : ConcurrentIntrospectionService
    {
        internal int Destroyed;

        internal TrackingService(RclNodeImpl node, string name, IConcurrentNativeServiceHandler handler)
            : base(node, name, handler, ListParametersService.GetTypeSupportHandle(), ServerOptions.Default)
        {
        }

        private RosMessageBuffer Track(RosMessageBuffer buffer)
            => new(buffer.Data, (_, _) =>
            {
                buffer.Dispose();
                Interlocked.Increment(ref Destroyed);
            });

        protected override RosMessageBuffer CreateRequestBuffer() => Track(base.CreateRequestBuffer());

        protected override RosMessageBuffer CreateResponseBuffer() => Track(base.CreateResponseBuffer());
    }
}
