namespace Rcl.SafeHandles;

unsafe class SafeClockHandle : RclObjectHandle<rcl_clock_t>
{
    private int _refCount;
    internal SpinLock SyncRoot = new();

    public SafeClockHandle(RclClockType clockType)
    {
        *Object = new rcl_clock_t();
        var alloc = RclAllocator.Default.Object;

        try
        {
            RclException.ThrowIfNonSuccess(
                rcl_clock_init((rcl_clock_type_t)clockType, Object, &alloc));
            MarkInitialized();
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    internal void AddTimerRef() => _refCount++;

    internal void ReleaseTimerRef() => _refCount--;

    protected override bool ReleaseHandleCore(rcl_clock_t* ptr)
    {
        using (ScopedLock.Lock(ref SyncRoot))
        {
            if (_refCount != 0)
            {
                throw new InvalidOperationException(
                    $"Unable to release a SafeClockHandle with active timer references. (type = {(RclClockType)ptr->type}, count = {_refCount})");
            }

            return CheckReleaseResult(rcl_clock_fini(ptr), nameof(rcl_clock_fini));
        }
    }
}