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
    /// All subsequent <see cref="IRclWaitObject.WaitOneAsync(CancellationToken)"/> calls will asynchronously
    /// wait until next call to <see cref="Trigger"/> to compelete.
    /// </remarks>
    void Trigger();
}