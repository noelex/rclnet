namespace Rcl.SafeHandles;

unsafe class SafeTimerHandle : RclObjectHandle<rcl_timer_t>
{
    public SafeTimerHandle(
        SafeContextHandle context, SafeClockHandle clock, long period)
    {
        *Object = rcl_get_zero_initialized_timer();
        try
        {
            RclException.ThrowIfNonSuccess(
                rcl_timer_init(Object, clock.Object, context.Object,
                  period, null, RclAllocator.Default.Object));
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    protected override void ReleaseHandleCore(rcl_timer_t* ptr)
    {
        rcl_timer_fini(ptr);
    }
}