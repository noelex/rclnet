using Rcl.Interop;

namespace Rcl.SafeHandles;

unsafe class SafeTimerHandle : RclObjectHandle<rcl_timer_t>
{
    private readonly SafeClockHandle _clock;

    public SafeTimerHandle(
        SafeContextHandle context, SafeClockHandle clock, long period)
    {
        _clock = clock;

        try
        {
            lock (context.LifecycleGate)
            {
                SetDependencies(context, clock);
                *DangerousObject = rcl_get_zero_initialized_timer();

                lock (_clock.SyncRoot)
                {
                    if (RosEnvironment.IsSupported(RosEnvironment.Jazzy))
                    {
                        RclException.ThrowIfNonSuccess(
                            RclJazzy.rcl_timer_init2(DangerousObject, clock.DangerousObject, context.DangerousObject,
                              period, null, RclAllocator.Default.Object, true));
                    }
                    else
                    {
                        RclException.ThrowIfNonSuccess(
                            rcl_timer_init(DangerousObject, clock.DangerousObject, context.DangerousObject,
                              period, null, RclAllocator.Default.Object));
                    }
                }

                MarkInitialized();
            }
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    protected override bool ReleaseHandleCore(rcl_timer_t* ptr)
    {
        lock (_clock.SyncRoot)
        {
            return CheckReleaseResult(rcl_timer_fini(ptr), nameof(rcl_timer_fini));
        }
    }
}
