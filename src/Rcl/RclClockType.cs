
namespace Rcl;

public enum RclClockType
{
    Unspecified = rcl_clock_type_t.RCL_CLOCK_UNINITIALIZED,

    Ros = rcl_clock_type_t.RCL_ROS_TIME,

    System = rcl_clock_type_t.RCL_SYSTEM_TIME,

    Steady = rcl_clock_type_t.RCL_STEADY_TIME,
}