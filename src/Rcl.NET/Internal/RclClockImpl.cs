using Rcl.SafeHandles;

namespace Rcl.Internal;

class RclClockImpl : RclObject<SafeClockHandle>
{
    public RclClockImpl(RclClockType type) : base(new(type))
    {
        Type = type;
    }

    public RclClockType Type { get; }

    public unsafe bool IsRosTimeOverrideEnabled
    {
        get
        {
            bool enabled;
            rcl_is_enabled_ros_time_override(Handle.Object, &enabled);
            return enabled;
        }
    }

    public unsafe void ToggleRosTimeOverride(bool enabled)
    {
        if (enabled)
        {
            rcl_enable_ros_time_override(Handle.Object);
        }
        else
        {
            rcl_disable_ros_time_override(Handle.Object);
        }
    }

    public unsafe void SetRosTimeOverride(long nanoseconds)
    {
        RclException.ThrowIfNonSuccess(
            rcl_set_ros_time_override(Handle.Object, nanoseconds));
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