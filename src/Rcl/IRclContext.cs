namespace Rcl;

public interface IRclContext : IDisposable
{
    /// <summary>
    /// A context for scheduling executions on current <see cref="IRclContext"/> event loop.
    /// </summary>
    SynchronizationContext SynchronizationContext { get; }

    IRclGuardCondition CreateGuardCondition();

    IRclNode CreateNode(string name, string @namespace = "/", NodeOptions? options = null);

    IRclTimer CreateTimer(RclClock clock, TimeSpan period);

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