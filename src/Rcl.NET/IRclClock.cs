namespace Rcl;

/// <summary>
/// Represents an abstraction over various kind of clocks.
/// </summary>
public interface IRclClock: IRclObject
{
    /// <summary>
    /// Type of the clock.
    /// </summary>
    RclClockType Type { get; }

    /// <summary>
    /// Time elasped since the clock's epoch time.
    /// </summary>
    TimeSpan Elapsed { get; }

    /// <summary>
    /// <see cref="Elapsed"/> time plus UNIX epoch.
    /// The returned value is meaningless if the clock is a <see cref="RclClockType.Steady"/> clock.
    /// </summary>
    /// <remarks>
    /// Also, if the clock is a <see cref="RclClockType.Ros"/> clock, the meaning this property
    /// depends on the underlying ROS time source. 
    /// </remarks>
    DateTimeOffset Now { get; }
}