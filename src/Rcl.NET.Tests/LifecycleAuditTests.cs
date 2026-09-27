using Rcl.Internal;
using Rcl.Internal.Publishers;
using Rcl.Logging;
using Rcl.SafeHandles;
using Rosidl.Messages.Builtin;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Rcl.NET.Tests;

public class LifecycleAuditTests : IDisposable
{
    private readonly HandleReleaseError?[] _errorsBefore = HandleReleaseDiagnostics.Snapshot();
    private static bool s_outputHeldLoggingGate;

    public void Dispose() => Assert.Empty(HandleReleaseDiagnostics.Snapshot().Except(_errorsBefore));

    [Fact]
    public unsafe void NativeLogOutputUsesTheLoggingLifetimeGate()
    {
        using var context = NewContext();
        var logger = context.CreateLogger("lifecycle_logging_gate");
        nint original;

        lock (SafeContextHandle.LoggingGate)
        {
            original = rcutils_logging_get_output_handler();
            s_outputHeldLoggingGate = false;
            rcutils_logging_set_output_handler((nint)(delegate* unmanaged[Cdecl]<nint, int, nint, long, nint, nint, void>)&CaptureLog);
        }

        try
        {
            logger.LogWarning("lifecycle audit");
            Assert.True(s_outputHeldLoggingGate);
        }
        finally
        {
            lock (SafeContextHandle.LoggingGate)
            {
                rcutils_logging_set_output_handler(original);
            }
        }
    }

    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) })]
    private static void CaptureLog(nint location, int severity, nint name, long timestamp, nint format, nint args)
    {
        s_outputHeldLoggingGate = Monitor.IsEntered(SafeContextHandle.LoggingGate);
    }

    [DllImport("rcutils", CallingConvention = CallingConvention.Cdecl)]
    private static extern nint rcutils_logging_get_output_handler();

    [DllImport("rcutils", CallingConvention = CallingConvention.Cdecl)]
    private static extern void rcutils_logging_set_output_handler(nint handler);

    [Fact]
    public async Task OwnedPublishBufferIsReleasedBeforeCompletionEvenWithoutConsumer()
    {
        await using var context = NewContext();
        using var node = (RclNodeImpl)context.CreateNode(NameGenerator.GenerateNodeName());
        using var publisher = new OwnedPublisher(node);
        using var releasing = new LifecycleCheckpoint();
        int released = 0;
        var buffer = RosMessageBuffer.Create<Time>();
        var tracked = new RosMessageBuffer(buffer.Data, (_, _) =>
        {
            releasing.Pause();
            buffer.Dispose();
            Interlocked.Increment(ref released);
        });
        var publish = publisher.PublishOwned(tracked);

        try
        {
            await releasing.Entered.WaitAsync(TimeSpan.FromSeconds(10));
            Assert.False(publish.IsCompleted);
            Assert.Equal(0, released);
        }
        finally
        {
            releasing.Resume();
        }

        Assert.True(SpinWait.SpinUntil(() => publish.IsCompleted, TimeSpan.FromSeconds(10)));
        Assert.Equal(1, released);
        await publish;
    }

    [Fact]
    public async Task FailedPublishReturnsOwnedBufferAndLeavesNextPublishUsable()
    {
        await using var context = NewContext();
        using var node = (RclNodeImpl)context.CreateNode(NameGenerator.GenerateNodeName());
        using var publisher = new OwnedPublisher(node);
        publisher.Dispose();
        int released = 0;
        var buffer = RosMessageBuffer.Create<Time>();
        var tracked = new RosMessageBuffer(buffer.Data, (_, _) =>
        {
            buffer.Dispose();
            Interlocked.Increment(ref released);
        });
        await Assert.ThrowsAsync<ObjectDisposedException>(() => publisher.PublishOwned(tracked).AsTask());
        Assert.Equal(1, released);

        using var next = new OwnedPublisher(node);

        for (int i = 0; i < 100; i++)
        {
            await next.PublishOwned(RosMessageBuffer.Create<Time>());
        }
    }

    [Fact]
    public async Task AsyncPublishDoesNotTakeOwnershipOfCallerBuffer()
    {
        await using var context = NewContext();
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var publisher = node.CreatePublisher<Time>("/audit_borrowed_publish");
        int released = 0;
        var buffer = RosMessageBuffer.Create<Time>();
        using var tracked = new RosMessageBuffer(buffer.Data, (_, _) =>
        {
            buffer.Dispose();
            Interlocked.Increment(ref released);
        });
        await publisher.PublishAsync(tracked);
        Assert.Equal(0, released);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void AbandonedNativeDependencyChainsReleaseWithoutExplicitDispose(bool timer)
    {
        int logging = SafeContextHandle.LoggingReferences;
        var handles = AbandonChain(timer);
        CollectUntil(() => handles.All(handle => !handle.IsAlive) && SafeContextHandle.LoggingReferences == logging);
        Assert.All(handles, handle => Assert.False(handle.IsAlive));
        Assert.Equal(logging, SafeContextHandle.LoggingReferences);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static WeakReference[] AbandonChain(bool timer)
    {
        var context = new SafeContextHandle(TestConfig.DefaultContextArguments);

        if (timer)
        {
            var clock = new SafeClockHandle(RclClockType.Steady);
            var child = new SafeTimerHandle(context, clock, 1000000);
            return [new(context), new(clock), new(child)];
        }

        var node = new SafeNodeHandle(context, NameGenerator.GenerateNodeName(), "/", NodeOptions.Default);
        var borrowed = SafeArgumentsHandle.Borrow(node);
        return [new(context), new(node), new(borrowed)];
    }

    [Fact]
    public async Task ActiveRegistrationRootsWrapperUntilContextDetach()
    {
        int logging = SafeContextHandle.LoggingReferences;
        var context = NewContext();
        var wrapper = AbandonRegisteredTimer(context);
        GC.Collect();
        GC.WaitForPendingFinalizers();
        Assert.True(wrapper.IsAlive);
        await context.DisposeAsync();
        CollectUntil(() => !wrapper.IsAlive && context.Handle.IsClosed);
        Assert.False(wrapper.IsAlive);
        Assert.True(context.Handle.IsClosed);
        Assert.Equal(logging, SafeContextHandle.LoggingReferences);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static WeakReference AbandonRegisteredTimer(RclContext context)
        => new(context.CreateTimer(TimeSpan.FromHours(1)));

    private static void CollectUntil(Func<bool> complete)
    {
        for (int i = 0; i < 10 && !complete(); i++)
        {
            GC.Collect();
            GC.WaitForPendingFinalizers();
        }
    }

    private static RclContext NewContext() => new(TestConfig.DefaultContextArguments);

    private sealed class OwnedPublisher(RclNodeImpl node)
        : RclNativePublisher(node, "/audit_owned_publish", Time.GetTypeSupportHandle(), PublisherOptions.Default)
    {
        internal ValueTask PublishOwned(RosMessageBuffer buffer) => PublishAsync(buffer, true);
    }
}
