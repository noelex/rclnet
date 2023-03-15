using Rosidl.Runtime;

namespace Rcl.Actions;

/// <summary>
/// Represents the result of an action goal.
/// </summary>
/// <remarks>
/// The ownership of the <see cref="RosMessageBuffer"/> is transfered to the caller. It's the caller's
/// responsibility to make sure that the buffer is disposed when no longer needed, if the result is successful.
/// </remarks>
/// <param name="Status">Status of the action goal.</param>
/// <param name="Result">
/// A <see cref="RosMessageBuffer"/> containing the native message structure of the action result.
/// If <see cref="IsSuccessful"/> is <see langword="false"/>, this will be <see cref="RosMessageBuffer.Empty"/>.
/// </param>
public record struct ActionResult(ActionGoalStatus Status, RosMessageBuffer Result)
{
    /// <summary>
    /// Gets whether current <see cref="ActionResult"/> contains a successful status.
    /// </summary>
    public bool IsSuccessful => Status == ActionGoalStatus.Succeeded;
}

/// <summary>
/// Represents the result of an action goal.
/// </summary>
/// <typeparam name="T">Type of the result,</typeparam>
/// <param name="Status">Status of the action goal.</param>
/// <param name="Result">
/// Result of the action goal.
/// If <see cref="IsSuccessful"/> is <see langword="false"/>, this will be <see langword="null"/>.
/// </param>
public record struct ActionResult<T>(ActionGoalStatus Status, T? Result) where T : IActionResult
{
    /// <summary>
    /// Gets whether current <see cref="ActionResult{T}"/> contains a successful status.
    /// </summary>
    public bool IsSuccessful => Status == ActionGoalStatus.Succeeded;
}
