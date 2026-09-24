using Rcl.SafeHandles;

namespace Rcl.Internal;

internal class RclTimer : RclWaitObject<SafeTimerHandle>, IRclTimer
{
    public RclTimer(
        RclContext context,
        RclClockImpl clock,
        TimeSpan period)
        : base(context, new(context.Handle, clock.Handle, (long)period.TotalNanoseconds))
    {
        RegisterWaitHandle();
    }

    protected override unsafe void OnWaitCompleted()
    {
        using var lease = Handle.Acquire();
        rcl_timer_call(lease.Object);
    }

    public unsafe bool IsPaused
    {
        get
        {
            using var lease = Handle.Acquire();
            bool ret;
            rcl_timer_is_canceled(lease.Object, &ret);
            return ret;
        }
    }

    public unsafe void Pause()
    {
        using var lease = Handle.Acquire();
        rcl_timer_cancel(lease.Object);
    }

    public unsafe void Resume()
    {
        using var lease = Handle.Acquire();
        rcl_timer_reset(lease.Object);
    }
}
