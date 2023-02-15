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

    }

    protected override unsafe void OnWaitCompleted()
    {
        rcl_timer_reset(Handle.Object);
    }

    public unsafe bool IsPaused
    {
        get
        {
            bool ret;
            rcl_timer_is_canceled(Handle.Object, &ret);
            return ret;
        }
    }

    public unsafe void Pause()
    {
        rcl_timer_cancel(Handle.Object);
    }

    public unsafe void Resume()
    {
        rcl_timer_reset(Handle.Object);
    }
}