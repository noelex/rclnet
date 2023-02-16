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
    /// All subsequent <see cref="IRclWaitObject.WaitOneAsync(CancellationToken)"/> calls will have to
    /// wait asynchronously until next call to <see cref="Trigger"/>.
    /// </remarks>
    void Trigger();
}