namespace Rcl;

/// <summary>
/// Exposes various operations can be performed an <see cref="RclContext"/>.
/// </summary>
public interface IRclContext : IDisposable
{
    /// <summary>
    /// A context for scheduling executions on current <see cref="IRclContext"/> event loop.
    /// </summary>
    SynchronizationContext SynchronizationContext { get; }

    /// <summary>
    /// Creates a guard condition that allows manual signaling.
    /// </summary>
    /// <returns>An <see cref="IRclGuardCondition"/> object.</returns>
    IRclGuardCondition CreateGuardCondition();

    /// <summary>
    /// Creates an ROS node.
    /// </summary>
    /// <param name="name">Name of the node.</param>
    /// <param name="namespace">Namespace of the node</param>
    /// <param name="options">Other options to be used when creating the node.</param>
    /// <returns>An <see cref="IRclNode"/> object.</returns>
    IRclNode CreateNode(string name, string @namespace = "/", NodeOptions? options = null);

    /// <summary>
    /// Creates a timer with specified period.
    /// </summary>
    /// <param name="clock">Clock to be used.</param>
    /// <param name="period">Period of the timer.</param>
    /// <returns>An <see cref="IRclTimer"/> object can be used for listening and controlling the timer.</returns>
    IRclTimer CreateTimer(RclClock clock, TimeSpan period);

    /// <summary>
    /// Creates a timer with specified period using <see cref="RclClock.Ros"/>.
    /// </summary>
    /// <param name="period">Period of the timer.</param>
    /// <returns>An <see cref="IRclTimer"/> object can be used for listening and controlling the timer.</returns>
    IRclTimer CreateTimer(TimeSpan period);

    /// <summary>
    /// Creates an awaitable that asynchronously yields back to current <see cref="RclContext"/> when awaited.
    /// </summary>
    /// <returns>
    /// A context that, when awaited, will asynchronously transition back into the current <see cref="RclContext"/> at the
    /// time of the await.
    /// </returns>
    YieldAwaiter Yield();

    /// <summary>
    /// Checks whether the caller is on the event loop of current <see cref="IRclContext"/>.
    /// </summary>
    bool IsCurrent { get; }
}