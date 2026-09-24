using Rcl.Internal;
using Rcl.Interop;
using Rcl.Logging;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Messages.Builtin;
using Rosidl.Messages.Rcl;
using static Rcl.Interop.RclCommon;

namespace Rcl.NET.Tests;

public class DependencyLifecycleTests : IDisposable
{
    private readonly HandleReleaseError?[] _errorsBefore = HandleReleaseDiagnostics.Snapshot();

    public void Dispose()
        => Assert.Empty(HandleReleaseDiagnostics.Snapshot().Except(_errorsBefore));

    [Fact]
    public unsafe void NodeCloseKeepsPublisherAndSubscriptionNativeParentsAlive()
    {
        using var context = new SafeContextHandle(TestConfig.DefaultContextArguments);
        using var node = NewNode(context);
        using var publisher = new SafePublisherHandle(node, Time.GetTypeSupportHandle(), "/lifecycle_pub", PublisherOptions.Default);
        using var subscription = new SafeSubscriptionHandle(node, Time.GetTypeSupportHandle(), "/lifecycle_pub", SubscriptionOptions.Default);
        node.Dispose();
        Assert.False(node.IsClosed);
        Assert.Throws<ObjectDisposedException>(() => new SafePublisherEventHandle(publisher,
            rcl_publisher_event_type_t.RCL_PUBLISHER_LIVELINESS_LOST));

        using (var lease = publisher.Acquire())
        {
            using (var buffer = RosMessageBuffer.Create<Time>())
            {
                RclException.ThrowIfNonSuccess(rcl_publish(lease.Object, buffer.Data.ToPointer(), null));
            }
        }

        using (var lease = subscription.Acquire())
        {
            Assert.True(rcl_subscription_is_valid(lease.Object));
        }

        Assert.Throws<ObjectDisposedException>(() => new SafePublisherHandle(node, Time.GetTypeSupportHandle(), "/rejected", PublisherOptions.Default));
        publisher.Dispose();
        Assert.False(node.IsClosed);
        subscription.Dispose();
        Assert.True(node.IsClosed);
    }

    [Fact]
    public void ClientAndServicePinBothNodeAndClock()
    {
        using var context = new SafeContextHandle(TestConfig.DefaultContextArguments);
        using var node = NewNode(context);
        using var clock = new SafeClockHandle(RclClockType.Ros);
        var type = ListParametersService.GetTypeSupportHandle();
        using var client = new SafeClientHandle(node, clock, type, "/lifecycle_service", QosProfile.ServicesDefault);
        using var service = new SafeServiceHandle(node, clock, type, "/lifecycle_service", QosProfile.ServicesDefault);
        node.Dispose();
        clock.Dispose();
        Assert.False(node.IsClosed);
        Assert.False(clock.IsClosed);

        using (var lease = client.Acquire())
        {
        }

        using (var lease = service.Acquire())
        {
        }

        client.Dispose();
        Assert.False(clock.IsClosed);
        service.Dispose();
        Assert.True(node.IsClosed);
        Assert.True(clock.IsClosed);
    }

    [Fact]
    public void EventsPinTheirEntities()
    {
        using var context = new SafeContextHandle(TestConfig.DefaultContextArguments);
        using var node = NewNode(context);
        using var publisher = new SafePublisherHandle(node, Time.GetTypeSupportHandle(), "/lifecycle_events", PublisherOptions.Default);
        using var subscription = new SafeSubscriptionHandle(node, Time.GetTypeSupportHandle(), "/lifecycle_events", SubscriptionOptions.Default);
        using var publisherEvent = new SafePublisherEventHandle(publisher, rcl_publisher_event_type_t.RCL_PUBLISHER_LIVELINESS_LOST);
        using var subscriptionEvent = new SafeSubscriptionEventHandle(subscription, rcl_subscription_event_type_t.RCL_SUBSCRIPTION_LIVELINESS_CHANGED);
        publisher.Dispose();
        subscription.Dispose();
        node.Dispose();
        Assert.False(publisher.IsClosed);
        Assert.False(subscription.IsClosed);
        publisherEvent.Dispose();
        Assert.True(publisher.IsClosed);
        Assert.False(node.IsClosed);
        subscriptionEvent.Dispose();
        Assert.True(subscription.IsClosed);
        Assert.True(node.IsClosed);
    }

    [Fact]
    public unsafe void BorrowedHandlesPinTheirActualOwners()
    {
        using var context = new SafeContextHandle(TestConfig.DefaultContextArguments);
        using var node = NewNode(context);
        using var graph = SafeGuardConditionHandle.BorrowGraphGuard(node);
        using var localArguments = SafeArgumentsHandle.Borrow(node);
        using var globalArguments = SafeArgumentsHandle.Borrow(context);
        node.Dispose();
        Assert.False(node.IsClosed);

        using (var lease = localArguments.Acquire())
        {
            Assert.Equal(0, rcl_arguments_get_count_unparsed(lease.Object));
        }

        graph.Dispose();
        Assert.False(node.IsClosed);
        localArguments.Dispose();
        Assert.True(node.IsClosed);
        context.Dispose();
        Assert.False(context.IsClosed);
        globalArguments.Dispose();
        Assert.True(context.IsClosed);
    }

    [Fact]
    public void WaitSetAndGuardPinContext()
    {
        using var context = new SafeContextHandle(TestConfig.DefaultContextArguments);
        using var waitSet = new SafeWaitSetHandle(context);
        using var guard = new SafeGuardConditionHandle(context);
        context.Dispose();
        Assert.False(context.IsClosed);
        Assert.Throws<ObjectDisposedException>(() => new SafeGuardConditionHandle(context));
        waitSet.Dispose();
        Assert.False(context.IsClosed);
        guard.Dispose();
        Assert.True(context.IsClosed);
    }

    [Fact]
    public unsafe void SharedClockRemainsDomainNeutral()
    {
        using var first = new SafeContextHandle(TestConfig.DefaultContextArguments);
        using var second = new SafeContextHandle(TestConfig.DefaultContextArguments);
        using var clock = new SafeClockHandle(RclClockType.Steady);
        using var firstTimer = new SafeTimerHandle(first, clock, 1000000);
        using var secondTimer = new SafeTimerHandle(second, clock, 1000000);
        first.Dispose();
        Assert.Throws<ObjectDisposedException>(() =>
{
    using var lease = firstTimer.Acquire();
});
        using var thirdTimer = new SafeTimerHandle(second, clock, 1000000);
        clock.Dispose();
        Assert.Throws<ObjectDisposedException>(() => new SafeTimerHandle(second, clock, 1000000));

        using (var lease = secondTimer.Acquire())
        {
            bool ready;
            RclException.ThrowIfNonSuccess(rcl_timer_is_ready(lease.Object, &ready));
        }

        firstTimer.Dispose();
        Assert.True(first.IsClosed);
        Assert.False(clock.IsClosed);
        secondTimer.Dispose();
        thirdTimer.Dispose();
        Assert.True(clock.IsClosed);
    }

    [Theory]
    [InlineData("node")]
    [InlineData("publisher")]
    [InlineData("subscription")]
    [InlineData("client")]
    [InlineData("service")]
    [InlineData("timer")]
    public void NativeInitFailureReturnsEveryParentRef(string kind)
    {
        using var context = new SafeContextHandle(TestConfig.DefaultContextArguments);
        using var node = NewNode(context);
        using var clock = new SafeClockHandle(RclClockType.Steady);
        Assert.Throws<RclException>(() =>
        {
            using RclObjectHandle handle = kind switch
            {
                "node" => new SafeNodeHandle(context, "invalid name", "/", NodeOptions.Default),
                "publisher" => new SafePublisherHandle(node, Time.GetTypeSupportHandle(), "invalid name", PublisherOptions.Default),
                "subscription" => new SafeSubscriptionHandle(node, Time.GetTypeSupportHandle(), "invalid name", SubscriptionOptions.Default),
                "client" => new SafeClientHandle(node, clock, ListParametersService.GetTypeSupportHandle(), "invalid name", QosProfile.ServicesDefault),
                "service" => new SafeServiceHandle(node, clock, ListParametersService.GetTypeSupportHandle(), "invalid name", QosProfile.ServicesDefault),
                _ => new SafeTimerHandle(context, clock, -1)
            };
        });
        node.Dispose();
        clock.Dispose();
        context.Dispose();
        Assert.True(node.IsClosed);
        Assert.True(clock.IsClosed);
        Assert.True(context.IsClosed);
    }

    [Fact]
    public void FailedSecondParentReturnsContextRef()
    {
        using var context = new SafeContextHandle(TestConfig.DefaultContextArguments);
        using var clock = new SafeClockHandle(RclClockType.Steady);
        clock.Dispose();
        Assert.Throws<ObjectDisposedException>(() => new SafeTimerHandle(context, clock, 1000000));
        context.Dispose();
        Assert.True(context.IsClosed);
    }

    [Fact]
    public async Task ContextCloseBeforeInitRejectsConstruction()
    {
        using var context = new SafeContextHandle(TestConfig.DefaultContextArguments);
        Task construction;
        using var started = new ManualResetEventSlim();

        lock (context.LifecycleGate)
        {
            construction = Task.Run(() =>
            {
                started.Set();
                Assert.Throws<ObjectDisposedException>(() => new SafeGuardConditionHandle(context));
            });
            Assert.True(started.Wait(TimeSpan.FromSeconds(10)));
            context.TryBeginClose();
        }

        await construction.WaitAsync(TimeSpan.FromSeconds(10));
        context.Dispose();
        Assert.True(context.IsClosed);
    }

    [Fact]
    public async Task ContextCloseWaitsForAdmittedNativeInit()
    {
        using var context = new SafeContextHandle(TestConfig.DefaultContextArguments);
        using var entered = new LifecycleCheckpoint();
        SafeGuardConditionHandle? guard = null;
        var construction = Task.Run(() =>
        {
            lock (context.LifecycleGate)
            {
                guard = new SafeGuardConditionHandle(context);
                entered.Pause();
                Assert.False(context.IsClosing);
            }
        });
        await entered.Entered.WaitAsync(TimeSpan.FromSeconds(10));
        var closing = Task.Run(() => context.TryBeginClose());

        try
        {
            entered.Resume();
            await Task.WhenAll(construction, closing).WaitAsync(TimeSpan.FromSeconds(10));
        }
        finally
        {
            entered.Resume();
        }

        Assert.False(entered.TimedOut);
        Assert.NotNull(guard);
        context.Dispose();
        Assert.False(context.IsClosed);
        guard.Dispose();
        Assert.True(context.IsClosed);
    }

    [Fact]
    public async Task PhysicalContextRetainsLoggingAfterEventLoopStops()
    {
        int before = SafeContextHandle.LoggingReferences;
        var context = new RclContext(TestConfig.DefaultContextArguments);
        using var guard = new SafeGuardConditionHandle(context.Handle);
        await context.DisposeAsync();
        Assert.False(context.Handle.IsClosed);
        Assert.Equal(before + 1, SafeContextHandle.LoggingReferences);
        guard.Dispose();
        Assert.True(context.Handle.IsClosed);
        Assert.Equal(before, SafeContextHandle.LoggingReferences);
    }

    [Fact]
    public void ContextManagedConstructionFailureReturnsLogging()
    {
        int before = SafeContextHandle.LoggingReferences;
        Assert.Throws<InvalidOperationException>(() => new RclContext(
            TestConfig.DefaultContextArguments, new CallbackLoggerFactory(_ => throw new InvalidOperationException("Injected logger failure."))));
        Assert.Equal(before, SafeContextHandle.LoggingReferences);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task NodeManagedConstructionFailureReleasesContext(bool closeDuringConstruction)
    {
        int before = SafeContextHandle.LoggingReferences;
        RclContext? context = null;
        var factory = new CallbackLoggerFactory(name =>
        {
            if (name == "rclnet")
            {
                return;
            }

            if (closeDuringConstruction)
            {
                context!.Dispose();
            }
            else
            {
                throw new InvalidOperationException("Injected node logger failure.");
            }
        });
        context = new RclContext(TestConfig.DefaultContextArguments, factory);

        try
        {
            if (closeDuringConstruction)
            {
                Assert.Throws<ObjectDisposedException>(() => context.CreateNode(NameGenerator.GenerateNodeName()));
            }
            else
            {
                Assert.Throws<InvalidOperationException>(() => context.CreateNode(NameGenerator.GenerateNodeName()));
            }
        }
        finally
        {
            await context.DisposeAsync();
        }

        Assert.True(SpinWait.SpinUntil(() => context.Handle.IsClosed && SafeContextHandle.LoggingReferences == before,
            TimeSpan.FromSeconds(10)), "Constructor rollback left a native or logging reference outstanding.");
    }

    [Fact]
    public async Task NodeDisposePreservesUserPublisherAndRejectsNewChildren()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = (RclNodeImpl)context.CreateNode(NameGenerator.GenerateNodeName());
        using var publisher = node.CreatePublisher<Time>("/lifecycle_wrapper");
        node.Dispose();
        node.Dispose();
        await context.Yield();
        Assert.False(node.Handle.IsClosed);
        using var buffer = RosMessageBuffer.Create<Time>();
        publisher.Publish(buffer);
        Assert.Throws<ObjectDisposedException>(() => node.CreatePublisher<Time>("/rejected"));
        publisher.Dispose();
        await LifecycleAssert.EventuallyAsync(() => node.Handle.IsClosed);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task SubscriptionManagedConstructionFailureReturnsNativeRefs(bool native)
    {
        var context = new RclContext(TestConfig.DefaultContextArguments);
        var node = (RclNodeImpl)context.CreateNode(NameGenerator.GenerateNodeName());
        var options = new SubscriptionOptions(queueSize: -1);

        try
        {
            Assert.Throws<ArgumentOutOfRangeException>(() =>
            {
                using var subscription = native
                    ? (IDisposable)node.CreateNativeSubscription<Time>("/lifecycle_bad_channel", options)
                    : node.CreateSubscription<Time>("/lifecycle_bad_channel", options);
            });
        }
        finally
        {
            node.Dispose();
            await context.Yield();
            await context.DisposeAsync();
        }

        Assert.True(node.Handle.IsClosed);
        Assert.True(context.Handle.IsClosed);
    }

    [Fact]
    public async Task NativeChildrenCanReleaseAfterContextShutdown()
    {
        int before = SafeContextHandle.LoggingReferences;
        var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = NewNode(context.Handle);
        using var publisher = new SafePublisherHandle(node, Time.GetTypeSupportHandle(), "/lifecycle_late_release", PublisherOptions.Default);
        node.Dispose();
        await context.DisposeAsync();
        Assert.False(node.IsClosed);
        Assert.False(context.Handle.IsClosed);
        Assert.Equal(before + 1, SafeContextHandle.LoggingReferences);
        publisher.Dispose();
        Assert.True(node.IsClosed);
        Assert.True(context.Handle.IsClosed);
        Assert.Equal(before, SafeContextHandle.LoggingReferences);
    }

    [Fact]
    public async Task RosClockTimerInitAndFiniSerializeWithTimeJumps()
    {
        using var first = new SafeContextHandle(TestConfig.DefaultContextArguments);
        using var second = new SafeContextHandle(TestConfig.DefaultContextArguments);
        using var clock = new RclClock(RclClockType.Ros);
        var tasks = Enumerable.Range(0, 4).Select(index => Task.Run(() =>
        {
            for (int i = 0; i < 100; i++)
            {
                using var timer = new SafeTimerHandle(index % 2 == 0 ? first : second, clock.Impl.Handle, 1000000);
            }
        })).Append(Task.Run(() =>
        {
            for (int i = 0; i < 100; i++)
            {
                clock.Impl.ToggleRosTimeOverride(true);
                clock.Impl.SetRosTimeOverride(i * 1000000);
                clock.Impl.ToggleRosTimeOverride(false);
            }
        }));
        await Task.WhenAll(tasks).WaitAsync(TimeSpan.FromSeconds(20));
    }

    [Fact]
    public void FailedContextInitializationDoesNotAcquireLogging()
    {
        int before = SafeContextHandle.LoggingReferences;
        Assert.Throws<RclException>(() => new SafeContextHandle(new[] { "--ros-args", "-r" }));
        Assert.Equal(before, SafeContextHandle.LoggingReferences);
    }

    [Fact]
    public async Task RegistrationFailureAfterNativeInitRollsBackHandle()
    {
        int before = SafeContextHandle.LoggingReferences;
        var context = new RclContext(TestConfig.DefaultContextArguments);
        using var handle = new SafeGuardConditionHandle(context.Handle);
        await context.DisposeAsync();
        Assert.Throws<ObjectDisposedException>(() => new RclGuardConditionImpl(context, handle));
        Assert.True(SpinWait.SpinUntil(() => handle.IsClosed && context.Handle.IsClosed
            && SafeContextHandle.LoggingReferences == before, TimeSpan.FromSeconds(10)));
    }

    private static SafeNodeHandle NewNode(SafeContextHandle context)
        => new(context, NameGenerator.GenerateNodeName(), "/", NodeOptions.Default);

    private sealed class CallbackLoggerFactory(Action<string> callback) : IRclLoggerFactory
    {
        public IRclLogger CreateLogger(string name)
        {
            callback(name);
            return new SilentLogger(name);
        }
    }

    private sealed class SilentLogger(string name) : IRclLogger
    {
        public string Name => name;

        public void Log(LogSeverity severity, string? message, string file = "", string functionName = "", int lineNumber = 0)
        {
        }
    }
}
