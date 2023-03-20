using Rosidl.Messages.Rosgraph;

namespace Rcl.SafeHandles;

unsafe class SafeTimerHandle : RclObjectHandle<rcl_timer_t>
{
    private readonly SafeClockHandle _clock;

    public SafeTimerHandle(
        SafeContextHandle context, SafeClockHandle clock, long period)
    {
        _clock = clock;
        * Object = rcl_get_zero_initialized_timer();

        try
        {
            using (ScopedLock.Lock(ref _clock.SyncRoot))
            {
                RclException.ThrowIfNonSuccess(
                    rcl_timer_init(Object, clock.Object, context.Object,
                      period, null, RclAllocator.Default.Object));
            }
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    protected override void ReleaseHandleCore(rcl_timer_t* ptr)
    {
        using (ScopedLock.Lock(ref _clock.SyncRoot))
        {
            rcl_timer_fini(ptr);
        }
    }
}