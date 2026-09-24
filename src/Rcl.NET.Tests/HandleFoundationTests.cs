using Rcl.Interop;
using Rcl.SafeHandles;
using System.Collections.Concurrent;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Rcl.NET.Tests;

public class HandleFoundationTests
{
    [Fact]
    public unsafe void ClosePreservesAdmittedOperation()
    {
        var events = new ConcurrentQueue<string>();
        using var handle = new FoundationHandle(events);
        var lease = handle.Acquire();
        *lease.Object = 42;
        Assert.True(handle.TryBeginClose());
        Assert.False(handle.TryBeginClose());
        Assert.False(handle.IsReleaseRequested);
        Assert.Throws<ObjectDisposedException>(() => { using var rejected = handle.Acquire(); });
        handle.RequestRelease();
        handle.RequestRelease();
        Assert.True(handle.IsReleaseRequested);
        Assert.False(handle.IsClosed);
        Assert.Equal(42, *lease.Object);
        Assert.Empty(events);
        lease.Dispose();
        lease.Dispose();
        Assert.Equal(new[] { "handle:fini" }, events);
        Assert.Equal(IntPtr.Zero, handle.DangerousGetHandle());
        Assert.True(handle.ReleaseResult);
    }

    [Fact]
    public unsafe void EmptyLeaseRejectsAccess()
    {
        RclHandleLease<int> lease = default;
        try { _ = lease.Object; Assert.Fail("Default lease returned a pointer."); }
        catch (ObjectDisposedException) { }
        lease.Dispose();
        using var handle = new FoundationHandle(new());
        lease = handle.Acquire();
        lease.Dispose();
        try { _ = lease.Object; Assert.Fail("Disposed lease returned a pointer."); }
        catch (ObjectDisposedException) { }
    }

    [Fact]
    public void RejectedAdmissionReturnsItsRef()
    {
        var events = new ConcurrentQueue<string>();
        using var handle = new FoundationHandle(events);
        bool added = false;
        handle.DangerousAddRef(ref added);
        try
        {
            handle.Dispose();
            Assert.Throws<ObjectDisposedException>(() => { using var rejected = handle.Acquire(); });
            Assert.Empty(events);
        }
        finally { if (added) handle.DangerousRelease(); }
        Assert.Single(events);
    }

    [Fact]
    public async Task ConcurrentCloseWaitsForAdmittedOperation()
    {
        var events = new ConcurrentQueue<string>();
        using var handle = new FoundationHandle(events);
        using var checkpoint = new LifecycleCheckpoint();
        var operation = Task.Run(() =>
        {
            using var lease = handle.Acquire();
            checkpoint.Pause();
            unsafe { *lease.Object = 7; }
        });
        try
        {
            await checkpoint.Entered.WaitAsync(TimeSpan.FromSeconds(10));
            handle.Dispose();
            Assert.Empty(events);
            Assert.Throws<ObjectDisposedException>(() => { using var rejected = handle.Acquire(); });
        }
        finally
        {
            checkpoint.Resume();
            await operation.WaitAsync(TimeSpan.FromSeconds(10));
        }
        Assert.False(checkpoint.TimedOut);
        Assert.Single(events);
    }

    [Fact]
    public void DependenciesReleaseInReverseOrder()
    {
        var events = new ConcurrentQueue<string>();
        using var first = new FoundationHandle(events, "first");
        using var second = new FoundationHandle(events, "second");
        using var child = new FoundationHandle(events, "child", first, second);
        first.OnRelease = () => Assert.Equal(IntPtr.Zero, child.DangerousGetHandle());
        second.OnRelease = first.OnRelease;
        first.Dispose();
        second.Dispose();
        child.Dispose();
        Assert.Equal(new[] { "child:fini", "second:fini", "first:fini" }, events);
        Assert.True(first.ReleaseResult);
        Assert.True(second.ReleaseResult);
    }

    [Fact]
    public void ClosedAncestorRejectsNewDescendantOnly()
    {
        var events = new ConcurrentQueue<string>();
        using var parent = new FoundationHandle(events, "parent");
        using var child = new FoundationHandle(events, "child", parent);
        parent.Dispose();
        using (var lease = child.Acquire()) { }
        Assert.Throws<ObjectDisposedException>(() => new FoundationHandle(events, "rejected", child));
        Assert.Empty(events);
        child.Dispose();
        Assert.Equal(new[] { "child:fini", "parent:fini" }, events);
    }

    [Fact]
    public unsafe void DomainCloseRejectsNewOperations()
    {
        var events = new ConcurrentQueue<string>();
        using var domain = new FoundationHandle(events, "domain", domainRoot: true);
        using var child = new FoundationHandle(events, "child", domain);
        var lease = child.Acquire();
        domain.Dispose();
        Assert.Throws<ObjectDisposedException>(() => { using var rejected = child.Acquire(); });
        *lease.Object = 8;
        lease.Dispose();
        child.Dispose();
        Assert.Equal(new[] { "child:fini", "domain:fini" }, events);
    }

    [Fact]
    public void FailedSecondDependencyRollsBackFirst()
    {
        var events = new ConcurrentQueue<string>();
        using var first = new FoundationHandle(events, "first");
        using var second = new FoundationHandle(events, "second");
        second.Dispose();
        Assert.Throws<ObjectDisposedException>(() => new FoundationHandle(events, "child", first, second));
        first.Dispose();
        Assert.Equal(new[] { "second:fini", "first:fini" }, events);
    }

    [Fact]
    public void IncompatibleDomainsRollBackBothRefs()
    {
        var events = new ConcurrentQueue<string>();
        using var first = new FoundationHandle(events, "first", domainRoot: true);
        using var second = new FoundationHandle(events, "second", domainRoot: true);
        Assert.Throws<InvalidOperationException>(() => new FoundationHandle(events, "child", first, second));
        second.Dispose();
        first.Dispose();
        Assert.Equal(new[] { "second:fini", "first:fini" }, events);
    }

    [Fact]
    public void DependencySlotsAreFixedAndImmutable()
    {
        using var first = new FoundationHandle(new());
        using var second = new FoundationHandle(new());
        using var child = new FoundationHandle(new(), initialized: false);
        Assert.Throws<InvalidOperationException>(() => child.Attach(child));
        Assert.Throws<InvalidOperationException>(() => child.Attach(first, first));
        child.Attach(first, second);
        Assert.Throws<InvalidOperationException>(() => child.Attach(first));
        Assert.Throws<InvalidOperationException>(() => first.Attach(child));
    }

    [Fact]
    public unsafe void BorrowedHandleReturnsOwnerWithoutFreeingStorage()
    {
        var events = new ConcurrentQueue<string>();
        using var owner = new FoundationHandle(events, "owner");
        IntPtr storage = Marshal.AllocHGlobal(sizeof(int));
        try
        {
            using var borrowed = new BorrowedFoundationHandle(storage, owner);
            owner.Dispose();
            using (var lease = borrowed.Acquire()) { *lease.Object = 123; }
            borrowed.Dispose();
            Assert.Equal(0, borrowed.FiniCalls);
            Assert.Equal(123, *(int*)storage);
            Assert.Equal(new[] { "owner:fini" }, events);
        }
        finally { Marshal.FreeHGlobal(storage); }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    public void InvalidBorrowedPointerDoesNotPinOwner(int pointer)
    {
        var events = new ConcurrentQueue<string>();
        using var owner = new FoundationHandle(events);
        Assert.Throws<ObjectDisposedException>(() => new BorrowedFoundationHandle(new IntPtr(pointer), owner));
        owner.Dispose();
        Assert.Single(events);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void InitializationControlsCleanup(bool partial)
    {
        var events = new ConcurrentQueue<string>();
        using var handle = new FoundationHandle(events, initialized: false);
        if (partial) handle.RequireCleanup();
        Assert.Throws<ObjectDisposedException>(() => { using var rejected = handle.Acquire(); });
        handle.Dispose();
        Assert.Equal(partial ? 1 : 0, events.Count);
        Assert.Equal(IntPtr.Zero, handle.DangerousGetHandle());
    }

    [Fact]
    public void ConstructionRollbackReturnsTransferredRefs()
    {
        var events = new ConcurrentQueue<string>();
        using var first = new FoundationHandle(events, "first");
        using var second = new FoundationHandle(events, "second");
        using var child = new FoundationHandle(events, "child", first, second, initialized: false);
        first.Dispose();
        second.Dispose();
        child.Dispose();
        child.Dispose();
        Assert.Equal(IntPtr.Zero, child.DangerousGetHandle());
        Assert.Equal(new[] { "second:fini", "first:fini" }, events);
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(false, true)]
    [InlineData(true, false)]
    [InlineData(true, true)]
    public void ReleaseFailureStillCleansUp(bool throwCleanup, bool throwDiagnostics)
    {
        var events = new ConcurrentQueue<string>();
        using var first = new FoundationHandle(events, "first");
        using var second = new FoundationHandle(events, "second");
        using var child = new FoundationHandle(events, "child", first, second)
        {
            ThrowCleanup = throwCleanup,
            ThrowDiagnostics = throwDiagnostics,
            ErrorCode = 1
        };
        first.Dispose();
        second.Dispose();
        var lease = child.Acquire();
        child.Dispose();
        lease.Dispose();
        child.Dispose();
        Assert.False(child.ReleaseResult);
        Assert.Equal(IntPtr.Zero, child.DangerousGetHandle());
        Assert.Equal(new[] { "child:fini", "second:fini", "first:fini" }, events);
        Assert.NotNull(child.Error);
        Assert.Equal(nameof(FoundationHandle), child.Error.HandleType);
        Assert.Equal(throwCleanup ? "native cleanup" : "fake_fini", child.Error.Api);
        Assert.Equal(throwCleanup ? (int?)null : 1, child.Error.ReturnCode);
        if (!throwDiagnostics) Assert.Contains(child.Error, HandleReleaseDiagnostics.Snapshot());
    }

    [Fact]
    public void FinalizerFailureDoesNotEscape()
    {
        var events = new ConcurrentQueue<string>();
        var weak = AbandonHandle(events);
        for (int i = 0; i < 10 && events.Count < 2; i++)
        {
            GC.Collect();
            GC.WaitForPendingFinalizers();
        }
        Assert.False(weak.IsAlive);
        Assert.Equal(new[] { "child:fini", "parent:fini" }, events);
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static WeakReference AbandonHandle(ConcurrentQueue<string> events)
    {
        var parent = new FoundationHandle(events, "parent");
        var child = new FoundationHandle(events, "child", parent)
        {
            ThrowCleanup = true, ThrowDiagnostics = true
        };
        parent.Dispose();
        return new WeakReference(child);
    }

    [Fact]
    public void OperationLeaseDoesNotAllocate()
    {
        using var handle = new FoundationHandle(new());
        for (int i = 0; i < 10000; i++) { using var lease = handle.Acquire(); }
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 10000; i++) { using var lease = handle.Acquire(); }
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
    }

    [Fact]
    public unsafe void NativeErrorIsRecordedAndReset()
    {
        RclCommon.rcutils_reset_error();
        using var handle = new FoundationHandle(new()) { NativeError = true };
        handle.Dispose();
        Assert.False(handle.ReleaseResult);
        Assert.NotNull(handle.Error);
        Assert.Equal("rcl_event_fini", handle.Error.Api);
        Assert.NotEqual(0, handle.Error.ReturnCode);
        Assert.False(string.IsNullOrEmpty(handle.Error.Message));
        Assert.False(RclCommon.rcutils_error_is_set());
        string message = handle.Error.Message;
        RclCommon.rcl_event_fini(null);
        RclCommon.rcutils_reset_error();
        Assert.Equal(message, handle.Error.Message);
    }

    [Fact]
    public void FailedTimerInitializationLeavesClockUsable()
    {
        using var context = new SafeContextHandle(TestConfig.DefaultContextArguments);
        using var clock = new TrackingClockHandle();
        using var timer = new SafeTimerHandle(context, clock, 1000000);
        using var closedContext = new SafeContextHandle(TestConfig.DefaultContextArguments);
        closedContext.Dispose();
        Assert.Throws<ObjectDisposedException>(() => new SafeTimerHandle(closedContext, clock, 1000000));
        timer.Dispose();
        clock.Dispose();
        Assert.True(clock.ReleaseResult);
    }

    [Fact]
    public void FailedArgumentsInitializationThrows()
    {
        Assert.Throws<RclException>(() => new SafeArgumentsHandle(new[] { "--ros-args", "-r" }));
        using var arguments = new SafeArgumentsHandle(Array.Empty<string>());
        using var lease = arguments.Acquire();
    }

    private sealed unsafe class FoundationHandle : RclObjectHandle<int>
    {
        private readonly ConcurrentQueue<string> _events;
        private readonly string _name;
        internal bool ThrowCleanup, ThrowDiagnostics, NativeError;
        internal int ErrorCode;
        internal bool? ReleaseResult;
        internal HandleReleaseError? Error;
        internal Action? OnRelease;

        internal FoundationHandle(ConcurrentQueue<string> events, string name = "handle",
            RclObjectHandle? first = null, RclObjectHandle? second = null,
            bool initialized = true, bool domainRoot = false)
        {
            _events = events;
            _name = name;
            try
            {
                if (domainRoot) SetShutdownDomain(this);
                if (first != null) SetDependencies(first, second);
                if (initialized) MarkInitialized();
            }
            catch { Dispose(); throw; }
        }

        internal void Attach(RclObjectHandle first, RclObjectHandle? second = null) => SetDependencies(first, second);
        internal void RequireCleanup() => MarkCleanupRequired();

        protected override bool ReleaseHandleCore(int* ptr)
        {
            _events.Enqueue($"{_name}:fini");
            OnRelease?.Invoke();
            if (ThrowCleanup) throw new InvalidOperationException("Injected fini failure.");
            if (NativeError) return CheckReleaseResult(RclCommon.rcl_event_fini(null), "rcl_event_fini");
            return CheckReleaseResult((rcl_ret_t)ErrorCode, "fake_fini");
        }

        protected override void WriteReleaseError(HandleReleaseError error)
        {
            Error = error;
            if (ThrowDiagnostics) throw new InvalidOperationException("Injected diagnostic failure.");
            base.WriteReleaseError(error);
        }

        protected override bool ReleaseHandle()
        {
            bool result = base.ReleaseHandle();
            ReleaseResult = result;
            return result;
        }
    }

    private sealed unsafe class BorrowedFoundationHandle : RclObjectHandle<int>
    {
        internal int FiniCalls;
        internal BorrowedFoundationHandle(IntPtr storage, RclObjectHandle owner) : base(storage, owner) { }
        protected override bool ReleaseHandleCore(int* ptr)
        {
            FiniCalls++;
            throw new InvalidOperationException("Borrowed fini was called.");
        }
    }

    private sealed class TrackingClockHandle : SafeClockHandle
    {
        internal bool? ReleaseResult;
        internal TrackingClockHandle() : base(RclClockType.Steady) { }
        protected override bool ReleaseHandle()
        {
            bool result = base.ReleaseHandle();
            ReleaseResult = result;
            return result;
        }
    }
}
