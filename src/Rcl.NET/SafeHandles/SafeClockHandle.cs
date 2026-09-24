namespace Rcl.SafeHandles;

unsafe class SafeClockHandle : RclObjectHandle<rcl_clock_t>
{
    internal SpinLock SyncRoot = new();

    public SafeClockHandle(RclClockType clockType)
    {
        try
        {
            *DangerousObject = new rcl_clock_t();
            var alloc = RclAllocator.Default.Object;
            RclException.ThrowIfNonSuccess(
                rcl_clock_init((rcl_clock_type_t)clockType, DangerousObject, &alloc));
            MarkInitialized();
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    protected override bool ReleaseHandleCore(rcl_clock_t* ptr)
    {
        using (ScopedLock.Lock(ref SyncRoot))
        {
            return CheckReleaseResult(rcl_clock_fini(ptr), nameof(rcl_clock_fini));
        }
    }
}