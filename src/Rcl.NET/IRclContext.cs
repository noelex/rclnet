using Rcl.Logging;

namespace Rcl;

/// <summary>
/// Represents a context for hosting nodes and wait primitives. 
/// </summary>
public interface IRclContext : IDisposable, IAsyncDisposable
{
    /// <summary>
    /// A context for scheduling executions on current <see cref="IRclContext"/> event loop.
    /// </summary>
    SynchronizationContext SynchronizationContext { get; }

    /// <summary>
    /// Creates a logger with specific name.
    /// </summary>
    /// <param name="loggerName">The name of the logger.</param>
    /// <returns>A <see cref="IRclLogger"/> instance for writing logs.</returns>
    IRclLogger CreateLogger(string loggerName);

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
    /// Creates an ROS node with specified <see cref="RclClock"/>.
    /// </summary>
    /// <remarks>
    /// <see cref="IRclNode"/> created by this method will use the clock specified by <paramref name="clockOverride"/> as its internal clock,
    /// and the <see cref="NodeOptions.Clock"/> and <see cref="NodeOptions.ClockQos"/> options are ignored.
    /// <para>
    /// The node will become a 'consumer' of the clock, that is, no subscription of /clock topic will be created and the node will never try to
    /// set time override of the clock, regardless of the type of the clock. 
    /// </para>
    /// <para>
    /// This can be useful if you want your application to have a single node to be the producer of the clock, so that other nodes can also use the
    /// same external time source without subscribing to the /clock topic. 
    /// </para>
    /// </remarks>
    /// <param name="name">Name of the node.</param>
    /// <param name="clockOverride">The <see cref="RclClock"/> to be used to override clock settings.</param>
    /// <param name="namespace">Namespace of the node.</param>
    /// <param name="options">Other options to be used when creating the node.</param>
    /// <returns></returns>
    IRclNode CreateNode(string name, RclClock clockOverride, string @namespace = "/", NodeOptions? options = null);

    /// <summary>
    /// Creates a timer with specified period.
    /// </summary>
    /// <param name="clock">Clock to be used. The clock must be created by current context.</param>
    /// <param name="period">Period of the timer.</param>
    /// <returns>An <see cref="IRclTimer"/> object can be used for listening and controlling the timer.</returns>
    IRclTimer CreateTimer(RclClock clock, TimeSpan period);

    /// <summary>
    /// Creates a timer with specified period using <see cref="SteadyClock"/>.
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