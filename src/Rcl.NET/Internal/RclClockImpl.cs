using Rcl.SafeHandles;

namespace Rcl.Internal;

class RclClockImpl : RclObject<SafeClockHandle>
{
    public RclClockImpl(RclClockType type) : base(new(type))
    {
    }

    public unsafe TimeSpan Elapsed
    {
        get
        {
            rcl_time_point_value_t t;
            rcl_clock_get_now(Handle.Object, &t);
            return TimeSpan.FromMicroseconds(t.Value / 1000.0);
        }
    }

    public DateTimeOffset Now => DateTimeOffset.UnixEpoch + Elapsed;
}