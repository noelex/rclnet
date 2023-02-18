namespace Rcl.Actions;

/// <summary>
/// A handler to process action goals sent by clients.
/// </summary>
/// <typeparam name="TGoal">Type of the goal message.</typeparam>
/// <typeparam name="TResult">Type of the result message.</typeparam>
/// <typeparam name="TFeedback">Type of the feedback message.</typeparam>
public interface IActionGoalHandler<TGoal, TResult, TFeedback>
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
    /// Executes the action goal.
    /// </summary>
    /// <remarks>
    /// This method may be called by the action server concurrently if there are multiple goals sent by action clients.
    /// </remarks>
    /// <param name="id">Id of the goal.</param>
    /// <param name="goal"><typeparamref name="TGoal"/> object sent by action client.</param>
    /// <param name="feeback">Used for reporting feedbacks.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> will be canceled when client sends cancel goal request.</param>
    /// <returns><typeparamref name="TResult"/> to be sent to the action client.</returns>
    Task<TResult> ExecuteAsync(Guid id, TGoal goal, IProgress<TFeedback> feeback, CancellationToken cancellationToken);
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
    /// This method will never be called by the action server concurrently.
    /// </remarks>
    /// <param name="id">Id of the goal.</param>
    /// <param name="goal">A <see cref="RosMessageBuffer"/> containing the goal message to be verified.</param>
    /// <returns><see langword="true"/> if the goal can be accepted, otherwise <see langword="false"/>.</returns>
    bool CanAccept(Guid id, RosMessageBuffer goal);

    /// <summary>
    /// Executes the action goal.
    /// </summary>
    /// <remarks>
    /// This method may be called by the action server concurrently if there are multiple goals sent by action clients.
    /// <para>
    /// Ownership of <see cref="RosMessageBuffer"/>s:
    /// </para>
    /// <list type="bullet">
    /// <item>
    /// The action server allocates and owns the <paramref name="goal"/> buffer.
    /// </item>
    /// <item>
    /// The implementation is responsibile for allocating and disposing the <see cref="RosMessageBuffer"/> for reporting feedback.
    /// </item>
    /// <item>
    /// The implementation is responsibile for allocating the result buffer and the ownership is transferred to the action server
    /// after return.
    /// </item>
    /// </list>
    /// </remarks>
    /// <param name="id">Id of the goal.</param>
    /// <param name="goal">A <see cref="RosMessageBuffer"/> containing the goal message sent by action client.</param>
    /// <param name="feeback">Used for reporting feedbacks.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> will be canceled when client sends cancel goal request.</param>
    /// <returns></returns>
    Task<bool> ExecuteAsync(Guid id, RosMessageBuffer goal, IProgress<RosMessageBuffer> feeback, CancellationToken cancellationToken);
}