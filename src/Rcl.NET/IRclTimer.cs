namespace Rcl;

/// <summary>
/// Represents an ROS timer.
/// </summary>
public interface IRclTimer : IRclWaitObject
{
    /// <summary>
    /// Gets whether the timer is paused.
    /// </summary>
    bool IsPaused { get; }

    /// <summary>
    /// Pause the timer.
    /// </summary>
    void Pause();

    /// <summary>
    /// Resume the timer.
    /// </summary>
    void Resume();
}