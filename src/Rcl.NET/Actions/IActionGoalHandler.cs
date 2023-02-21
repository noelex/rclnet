using Rosidl.Runtime;

namespace Rcl.Actions;

/// <summary>
/// A handler to process action goals sent by clients.
/// </summary>
/// <typeparam name="TGoal">Type of the goal message.</typeparam>
/// <typeparam name="TResult">Type of the result message.</typeparam>
/// <typeparam name="TFeedback">Type of the feedback message.</typeparam>
public interface IActionGoalHandler<TGoal, TResult, TFeedback>
    where TGoal : IActionGoal
    where TResult : IActionResult
    where TFeedback : IActionFeedback
{
    /// <summary>
    /// Checks whether the goal can be accepted.
    /// </summary>
    /// <remarks>
    /// This method will never be called by the action server concurrently.
    /// </remarks>
    /// <param name="id">Id of the goal.</param>
    /// <param name="goal"><typeparamref name="TGoal"/> object to be verified.</param>
    /// <returns><see langword="true"/> if the goal can be accepted, otherwise <see langword="false"/>.</returns>
    bool CanAccept(Guid id, TGoal goal);

    /// <summary>
    /// Notify the handler that a goal was accepted.
    /// </summary>
    /// <remarks>
    /// This method is non-reentrant.
    /// </remarks>
    /// <param name="controller"></param>
    void OnAccepted(IActionGoalController<TFeedback> controller);

    /// <summary>
    /// Notify the handler that a goal was completed.
    /// </summary>
    /// <remarks>
    /// This method is non-reentrant.
    /// </remarks>
    /// <param name="controller"></param>
    void OnCompleted(IActionGoalController<TFeedback> controller);

    /// <summary>
    /// Executes the action goal.
    /// </summary>
    /// <remarks>
    /// This method may be called by the action server concurrently if there are multiple goals sent by action clients.
    /// </remarks>
    /// <param name="goal"><typeparamref name="TGoal"/> object sent by action client.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> will be canceled when client sends cancel goal request.</param>
    /// <returns><typeparamref name="TResult"/> to be sent to the action client.</returns>
    Task<TResult> ExecuteAsync(IActionGoalController<TFeedback> controller, TGoal goal, CancellationToken cancellationToken);
}

public interface IActionGoalController
{
    /// <summary>
    /// Get the ID of the goal.
    /// </summary>
    Guid GoalId { get; }

    /// <summary>
    /// Get the status of the goal.
    /// </summary>
    ActionGoalStatus Status { get; }

    /// <summary>
    /// Abort the action goal. If the goal cannot be aborted, the call to this method would be no-op.
    /// </summary>
    void Abort();
}

/// <summary>
/// Control goal status on the server side.
/// </summary>
public interface INativeActionGoalController : IActionGoalController, IProgress<RosMessageBuffer>
{
}

/// <summary>
/// Control goal status on the server side.
/// </summary>
public interface IActionGoalController<T> : IActionGoalController, IProgress<T>
    where T : IActionFeedback
{
}

/// <summary>
/// Same as <see cref="IActionGoalHandler{TGoal, TResult, TFeedback}"/>, but sends and receives messages via native
/// message buffers.
/// </summary>
public interface INativeActionGoalHandler
{
    /// <summary>
    /// Checks whether the goal can be accepted.
    /// </summary>
    /// <remarks>
    /// This method is non-reentrant.
    /// </remarks>
    /// <param name="id">Id of the goal.</param>
    /// <param name="goal">A <see cref="RosMessageBuffer"/> containing the goal message to be verified.</param>
    /// <returns><see langword="true"/> if the goal can be accepted, otherwise <see langword="false"/>.</returns>
    bool CanAccept(Guid id, RosMessageBuffer goal);

    /// <summary>
    /// Notify the handler that a goal was accepted.
    /// </summary>
    /// <remarks>
    /// This method is non-reentrant.
    /// </remarks>
    /// <param name="controller"></param>
    void OnAccepted(INativeActionGoalController controller);

    /// <summary>
    /// Notify the handler that a goal was completed.
    /// </summary>
    /// <remarks>
    /// This method is non-reentrant.
    /// </remarks>
    /// <param name="controller"></param>
    void OnCompleted(INativeActionGoalController controller);

    /// <summary>
    /// Executes the action goal.
    /// </summary>
    /// <remarks>
    /// This method may be called by the action server concurrently if there are multiple goals sent by action clients.
    /// <para>
    /// Ownership of the <see cref="RosMessageBuffer"/>s is defined as follows:
    /// </para>
    /// <list type="bullet">
    /// <item>
    /// The action server allocates and owns the <paramref name="goal"/> and <paramref name="result"/> buffer.
    /// </item>
    /// <item>
    /// The implementation is responsibile for allocating and disposing the <see cref="RosMessageBuffer"/> for reporting feedback.
    /// </item>
    /// </list>
    /// </remarks>
    /// <param name="result">A <see cref="RosMessageBuffer"/> containing the result message to be filled by the implementation.</param>
    /// <param name="goal">A <see cref="RosMessageBuffer"/> containing the goal message sent by action client.</param>
    /// <param name="controller">Used for reporting feedbacks and control the status of the goal.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> will be canceled when client sends cancel goal request.</param>
    /// <returns></returns>
    Task ExecuteAsync(INativeActionGoalController controller, RosMessageBuffer goal, RosMessageBuffer result, CancellationToken cancellationToken);
}