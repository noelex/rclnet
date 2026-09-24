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
            using var lease = Handle.Acquire();

            lock (Handle.SyncRoot)
            {
                bool enabled;
                rcl_is_enabled_ros_time_override(lease.Object, &enabled);
                return enabled;
            }
        }
    }

    public unsafe void ToggleRosTimeOverride(bool enabled)
    {
        using var lease = Handle.Acquire();

        lock (Handle.SyncRoot)
        {
            if (enabled)
            {
                rcl_enable_ros_time_override(lease.Object);
            }
            else
            {
                rcl_disable_ros_time_override(lease.Object);
            }
        }
    }

    public unsafe void SetRosTimeOverride(long nanoseconds)
    {
        using var lease = Handle.Acquire();

        lock (Handle.SyncRoot)
        {
            RclException.ThrowIfNonSuccess(rcl_set_ros_time_override(lease.Object, nanoseconds));
        }
    }

    internal unsafe long Nanoseconds
    {
        get
        {
            using var lease = Handle.Acquire();
            rcl_time_point_value_t t;
            rcl_clock_get_now(lease.Object, &t);
            return t.Value;
        }
    }

    public TimeSpan Elapsed => TimeSpan.FromMicroseconds(Nanoseconds / 1000.0);

    public DateTimeOffset Now => DateTimeOffset.UnixEpoch + Elapsed;
}
