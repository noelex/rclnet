namespace Rcl;

/// <summary>
/// Represents a wait object which allows manual signaling.
/// </summary>
public interface IRclGuardCondition : IRclWaitObject
{
    /// <summary>
    /// Trigger the <see cref="IRclGuardCondition"/> once, allowing all current asynchronous waits to complete.
    /// </summary>
    /// <remarks>
    /// If the guard condition is currently being waited, then all current asynchronous waits will complete.
    /// <para>
    /// Otherwise the next call to <see cref="IRclWaitObject.WaitOneAsync(CancellationToken)"/> will complete 
    /// immediately, and all subsequent <see cref="IRclWaitObject.WaitOneAsync(CancellationToken)"/> calls will have to
    /// wait asynchronously until next call to <see cref="Trigger"/>.
    /// </para>
    /// </remarks>
    void Trigger();
}