namespace Rcl.SafeHandles;

unsafe class SafeClockHandle : RclObjectHandle<rcl_clock_t>
{
    public SafeClockHandle(RclClockType clockType)
    {
        *Object = new rcl_clock_t();
        var alloc = RclAllocator.Default.Object;

        try
        {
            RclException.ThrowIfNonSuccess(
                rcl_clock_init((rcl_clock_type_t)clockType, Object, &alloc));
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    protected override void ReleaseHandleCore(rcl_clock_t* ptr)
    {
        rcl_clock_fini(ptr);
    }
}