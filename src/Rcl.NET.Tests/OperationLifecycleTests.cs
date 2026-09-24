using Rcl.Internal;
using Rcl.Internal.Clients;
using Rcl.Internal.Publishers;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Messages.Builtin;
using Rosidl.Messages.Rcl;
using System.Collections.Concurrent;
using System.Runtime.InteropServices;
using static Rcl.Interop.RclCommon;

namespace Rcl.NET.Tests;

public class OperationLifecycleTests : IDisposable
{
    private readonly HandleReleaseError?[] _errorsBefore = HandleReleaseDiagnostics.Snapshot();

    public void Dispose()
        => Assert.Empty(HandleReleaseDiagnostics.Snapshot().Except(_errorsBefore));

    [Fact]
    public async Task WrapperClosesAdmissionBeforeSubclassCleanup()
    {
        var releases = new ConcurrentQueue<string>();
        using var checkpoint = new LifecycleCheckpoint();
        using var wrapper = new PausingObject(new FakeRclHandle(releases), checkpoint);
        var disposing = Task.Run(wrapper.Dispose);

        try
        {
            await checkpoint.Entered.WaitAsync(TimeSpan.FromSeconds(10));
            Assert.True(wrapper.Handle.IsClosing);
            Assert.False(wrapper.Handle.IsReleaseRequested);
            Assert.Throws<ObjectDisposedException>(() =>
{
    using var lease = wrapper.Handle.Acquire();
});
            wrapper.Dispose();
            Assert.Empty(releases);
        }
        finally
        {
            checkpoint.Resume();
            await disposing.WaitAsync(TimeSpan.FromSeconds(10));
        }

        Assert.Equal(new[] { "handle:enter", "handle:exit" }, releases);
        Assert.False(checkpoint.TimedOut);
    }

    [Fact]
    public async Task BorrowedPublisherDataCanBeCopiedAfterConcurrentClose()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var publisher = node.CreatePublisher<Time>("/operation_borrowed");
        var handle = ((RclNativePublisher)publisher).Handle;
        using var checkpoint = new LifecycleCheckpoint();
        var copying = Task.Run(() =>
        {
            using var lease = handle.Acquire();

            unsafe
            {
                var name = rcl_publisher_get_topic_name(lease.Object);
                var qos = rcl_publisher_get_actual_qos(lease.Object);
                checkpoint.Pause();
                Assert.Equal(publisher.Name, Marshal.PtrToStringUTF8((IntPtr)name));
                Assert.Equal(publisher.ActualQos, QosProfile.Create(in *qos));
                using var buffer = RosMessageBuffer.Create<Time>();
                RclException.ThrowIfNonSuccess(rcl_publish(lease.Object, buffer.Data.ToPointer(), null));
            }
        });

        try
        {
            await checkpoint.Entered.WaitAsync(TimeSpan.FromSeconds(10));
            publisher.Dispose();
            await context.Yield();
            Assert.True(handle.IsReleaseRequested);
            Assert.False(handle.IsClosed);
            Assert.Throws<ObjectDisposedException>(() => publisher.Publish(new Time()));
        }
        finally
        {
            checkpoint.Resume();
            await copying.WaitAsync(TimeSpan.FromSeconds(10));
        }

        Assert.True(handle.IsClosed);
        Assert.False(checkpoint.TimedOut);
    }

    [Fact]
    public async Task WrappersRejectNativeCallsBeforeQueuedReleaseRuns()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var publisher = node.CreatePublisher<Time>("/operation_close");
        using var subscription = node.CreateSubscription<Time>("/operation_close");
        using var nativeSubscription = node.CreateNativeSubscription<Time>("/operation_close");
        using var guard = context.CreateGuardCondition();
        using var timer = context.CreateTimer(TimeSpan.FromHours(1));
        using var client = node.CreateClient<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>("/operation_close");
        using var service = node.CreateService<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(
            "/operation_close", (request, state) => new ListParametersServiceResponse());
        using var checkpoint = new LifecycleCheckpoint();
        context.SynchronizationContext.Post(_ => checkpoint.Pause(), null);

        try
        {
            await checkpoint.Entered.WaitAsync(TimeSpan.FromSeconds(10));
            publisher.Dispose();
            subscription.Dispose();
            nativeSubscription.Dispose();
            guard.Dispose();
            timer.Dispose();
            client.Dispose();
            service.Dispose();
            node.Dispose();
            Assert.False(((RclNativePublisher)publisher).Handle.IsReleaseRequested);
            Assert.Throws<ObjectDisposedException>(() => _ = publisher.Subscribers);
            Assert.Throws<ObjectDisposedException>(() => publisher.AssertLiveliness());
            Assert.Throws<ObjectDisposedException>(() => _ = subscription.Publishers);
            Assert.Throws<ObjectDisposedException>(() => _ = nativeSubscription.Publishers);
            Assert.Throws<ObjectDisposedException>(guard.Trigger);
            Assert.Throws<ObjectDisposedException>(timer.Resume);
            Assert.Throws<ObjectDisposedException>(() => _ = client.IsServerAvailable);
            Assert.Throws<ObjectDisposedException>(() => _ = service.IsValid);
            Assert.Throws<ObjectDisposedException>(() => _ = node.DomaindId);
            Assert.Throws<ObjectDisposedException>(() => _ = node.Clock.Now);
        }
        finally
        {
            checkpoint.Resume();
        }

        await context.Yield();
        Assert.False(checkpoint.TimedOut);
    }

    [Fact]
    public async Task ContextCloseRejectsOperationsOnRetainedChildren()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var publisher = node.CreatePublisher<Time>("/operation_domain");
        using var guard = context.CreateGuardCondition();
        using var timer = context.CreateTimer(TimeSpan.FromHours(1));
        await context.DisposeAsync();
        Assert.Throws<ObjectDisposedException>(() => publisher.Publish(new Time()));
        Assert.Throws<ObjectDisposedException>(guard.Trigger);
        Assert.Throws<ObjectDisposedException>(timer.Pause);
        Assert.Throws<ObjectDisposedException>(() => _ = node.InstanceId);
    }

    [Fact]
    public async Task ClientCloseRejectsSendWaitingToEnterEventLoop()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = node.CreateClient<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(
            NameGenerator.GenerateServiceName());
        using var checkpoint = new LifecycleCheckpoint();
        context.SynchronizationContext.Post(_ => checkpoint.Pause(), null);
        Task<ListParametersServiceResponse> pending;

        try
        {
            await checkpoint.Entered.WaitAsync(TimeSpan.FromSeconds(10));
            pending = client.InvokeAsync(new ListParametersServiceRequest(), Timeout.Infinite);
            client.Dispose();
        }
        finally
        {
            checkpoint.Resume();
        }

        await Assert.ThrowsAsync<ObjectDisposedException>(() => pending.WaitAsync(TimeSpan.FromSeconds(10)));
        Assert.False(checkpoint.TimedOut);
    }

    [SkippableFact]
    public async Task ParentClosePreservesQueriesButRejectsIntrospectionCreation()
    {
        Skip.If(!RosEnvironment.IsSupported(RosEnvironment.Iron));
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var clock = new RclClock(RclClockType.Ros);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName(), clockOverride: clock);
        using var publisher = node.CreatePublisher<Time>("/operation_parent");
        using var client = node.CreateClient<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>("/operation_parent");
        using var service = node.CreateService<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(
            "/operation_parent", (request, state) => new ListParametersServiceResponse());
        using var timer = context.CreateTimer(clock, TimeSpan.FromHours(1));
        node.Dispose();
        clock.Dispose();
        publisher.Publish(new Time());
        _ = client.IsServerAvailable;
        Assert.True(service.IsValid);
        timer.Pause();
        timer.Resume();
        Assert.Throws<ObjectDisposedException>(() => _ = clock.Now);
        Assert.Throws<ObjectDisposedException>(() => context.CreateTimer(clock, TimeSpan.FromSeconds(1)));
        Assert.Throws<ObjectDisposedException>(() => node.CreatePublisher<Time>("/rejected"));
        Assert.Throws<ObjectDisposedException>(() => client.ConfigureIntrospection(ServiceIntrospectionState.Full));
        Assert.Throws<ObjectDisposedException>(() => service.ConfigureIntrospection(ServiceIntrospectionState.Full));
    }

    [SkippableTheory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task IntrospectionConfigurationCanRaceWithRequestsAndResponses(bool concurrentService)
    {
        Skip.If(!RosEnvironment.IsSupported(RosEnvironment.Iron));
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        var name = NameGenerator.GenerateServiceName();
        using var service = concurrentService
            ? node.CreateConcurrentService<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(
                name, async (request, state, token) =>
{
    await Task.Yield();
    return new ListParametersServiceResponse();
}, null)
            : node.CreateService<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(
                name, (request, state) => new ListParametersServiceResponse());
        using var client = node.CreateClient<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(name);
        Assert.True(await client.TryWaitForServerAsync(10_000));
        using var start = new ManualResetEventSlim();

        Task Configure(Action<ServiceIntrospectionState> configure) => Task.Run(() =>
        {
            Assert.True(start.Wait(TimeSpan.FromSeconds(10)));

            for (var i = 0; i < 40; i++)
            {
                configure((ServiceIntrospectionState)(i % 3));
            }
        });

        var configuringClient = Configure(state => client.ConfigureIntrospection(state));
        var configuringService = Configure(state => service.ConfigureIntrospection(state));
        var requests = Task.Run(async () =>
        {
            Assert.True(start.Wait(TimeSpan.FromSeconds(10)));

            for (var i = 0; i < 40; i++)
            {
                await client.InvokeAsync(new ListParametersServiceRequest(), 10_000);
            }
        });
        start.Set();
        await Task.WhenAll(configuringClient, configuringService, requests).WaitAsync(TimeSpan.FromSeconds(30));
    }

    [SkippableFact]
    public async Task AdmittedConfigurationFinishesAfterClientClose()
    {
        Skip.If(!RosEnvironment.IsSupported(RosEnvironment.Iron));
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = node.CreateClient<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(
            NameGenerator.GenerateServiceName());
        var handle = ((RclClientBase)client).Handle;
        using var started = new ManualResetEventSlim();
        Exception? failure = null;
        var configuring = new Thread(() =>
        {
            try
            {
                started.Set();
                client.ConfigureIntrospection(ServiceIntrospectionState.Full);
            }
            catch (Exception error)
            {
                failure = error;
            }
        });
        await Task.Run(() =>
        {
            try
            {
                lock (handle.NativeGate)
                {
                    configuring.Start();
                    Assert.True(started.Wait(TimeSpan.FromSeconds(10)));
                    // The dedicated thread has passed admission and is blocked on the native gate.
                    Assert.True(SpinWait.SpinUntil(() =>
                        (configuring.ThreadState & ThreadState.WaitSleepJoin) != 0, TimeSpan.FromSeconds(10)));
                    client.Dispose();
                    Assert.True(SpinWait.SpinUntil(() => handle.IsReleaseRequested, TimeSpan.FromSeconds(10)));
                    Assert.False(handle.IsClosed);
                }
            }
            finally
            {
                Assert.True(configuring.Join(TimeSpan.FromSeconds(10)));
            }
        });
        Assert.Null(failure);
        Assert.True(handle.IsClosed);
    }

    private sealed class PausingObject : RclObject<FakeRclHandle>
    {
        private readonly LifecycleCheckpoint _checkpoint;

        public PausingObject(FakeRclHandle handle, LifecycleCheckpoint checkpoint) : base(handle)
            => _checkpoint = checkpoint;

        protected override void DisposeCore()
        {
            _checkpoint.Pause();
            base.DisposeCore();
        }
    }
}
