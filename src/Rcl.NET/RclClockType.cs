
namespace Rcl;

/// <summary>
/// Time source type, used to indicate the source of a time measurement.
/// </summary>
public enum RclClockType
{
    /// <summary>
    /// The type of the clock is not specified.
    /// </summary>
    Unspecified = rcl_clock_type_t.RCL_CLOCK_UNINITIALIZED,

    /// <summary>
    /// The clock reports the latest time value reported by a ROS time source, or
    /// if a ROS time source is not active it reports the same as <see cref="System"/> clock.
    /// </summary>
    /// <remarks>
    /// See <a href="http://design.ros2.org/articles/clock_and_time.html#RCL_SYSTEM_TIME"/> for more
    /// information about ROS time sources.
    /// </remarks>
    Ros = rcl_clock_type_t.RCL_ROS_TIME,

    /// <summary>
    /// The clock reports the latest time value from the wall clock of the operating system.
    /// </summary>
    System = rcl_clock_type_t.RCL_SYSTEM_TIME,

    /// <summary>
    /// The clock reports the latest time value from a monotonically increasing clock.
    /// </summary>
    Steady = rcl_clock_type_t.RCL_STEADY_TIME,
}