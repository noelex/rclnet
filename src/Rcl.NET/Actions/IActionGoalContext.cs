using Rosidl.Runtime;

namespace Rcl.Actions;

/// <summary>
/// A context object can be used for tracking
/// the state of an action goal sent by the action client. 
/// </summary>
public interface IActionGoalContext : IRclObject
{
    /// <summary>
    /// Report changes of status to the action goal.
    /// </summary>
    event Action<ActionGoalStatus>? StatusChanged;

    /// <summary>
    /// A <see cref="Task"/> object which will complete when the goal reaches terminal state.
    /// </summary>
    /// <remarks>
    /// If the <see cref="IActionGoalContext"/> is disposed before completion, <see langword="await"/>ing
    /// the <see cref="Task"/> returned by this property will receive an <see cref="ObjectDisposedException"/>.
    /// </remarks>
    Task<ActionGoalStatus> Completion { get; }

    /// <summary>
    /// Id of the goal.
    /// </summary>
    Guid GoalId { get; }

    /// <summary>
    /// Status of the goal.
    /// </summary>
    ActionGoalStatus Status { get; }

    /// <summary>
    /// Cancel the goal with specific timeout.
    /// </summary>
    /// <param name="timeout">Timeout of the request.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    Task CancelAsync(TimeSpan timeout, CancellationToken cancellationToken = default);

    /// <summary>
    /// Cancel the goal.
    /// </summary>
    /// <param name="timeoutMilliseconds">Timeout of the request, in milliseconds.</param>
    /// <param name="cancellationToken">
    /// A <see cref="CancellationToken"/> for canceling the operation.
    /// </param>
    /// <returns></returns>
    Task CancelAsync(int timeoutMilliseconds, CancellationToken cancellationToken = default);

    /// <summary>
    /// Cancel the goal.
    /// </summary>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    Task CancelAsync(CancellationToken cancellationToken = default);
}

/// <summary>
/// An <see cref="IActionGoalContext"/> which allows receiving result and feedbacks using native message buffers.
/// </summary>
public interface INativeActionGoalContext : IActionGoalContext
{
    /// <summary>
    /// Gets the result of the goal.
    /// </summary>
    /// <remarks>
    /// If the goal didn't complete with <see cref="ActionGoalStatus.Succeeded"/> status, an exception will be thrown.
    /// <para>To wait for completion without throwing, use <see cref="IActionGoalContext.Completion"/>.</para>
    /// <para>
    /// Though calling this method after completion is allowed.
    /// Action servers may or may not return result according to their configured result caching policy.
    /// </para>
    /// <para>
    /// Additionally, caller takes the ownership of the returned <see cref="RosMessageBuffer"/>.
    /// </para>
    /// </remarks>
    /// <param name="cancellationToken"></param>
    /// <returns></returns>
    Task<RosMessageBuffer> GetResultAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets the result of the goal.
    /// </summary>
    /// <remarks>
    /// If the goal didn't complete with <see cref="ActionGoalStatus.Succeeded"/> status, an exception will be thrown.
    /// <para>To wait for completion without throwing, use <see cref="IActionGoalContext.Completion"/>.</para>
    /// <para>
    /// Though calling this method after completion is allowed.
    /// Action servers may or may not return result according to their configured result caching policy.
    /// </para>
    /// <para>
    /// Additionally, caller takes the ownership of the returned <see cref="RosMessageBuffer"/>.
    /// </para>
    /// </remarks>
    /// <param name="cancellationToken"></param>
    /// <param name="timeoutMilliseconds">Request timeout in milliseconds.</param>
    /// <returns></returns>
    Task<RosMessageBuffer> GetResultAsync(int timeoutMilliseconds, CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets the result of the goal.
    /// </summary>
    /// <remarks>
    /// If the goal didn't complete with <see cref="ActionGoalStatus.Succeeded"/> status, an exception will be thrown.
    /// <para>To wait for completion without throwing, use <see cref="IActionGoalContext.Completion"/>.</para>
    /// <para>
    /// Though calling this method after completion is allowed.
    /// Action servers may or may not return result according to their configured result caching policy.
    /// </para>
    /// <para>
    /// Additionally, caller takes the ownership of the returned <see cref="RosMessageBuffer"/>.
    /// </para>
    /// </remarks>
    /// <param name="cancellationToken"></param>
    /// <param name="timeout">Request timeout.</param>
    /// <returns></returns>
    Task<RosMessageBuffer> GetResultAsync(TimeSpan timeout, CancellationToken cancellationToken = default);

    /// <summary>
    /// Asynchronously read all feedback messages.
    /// </summary>
    /// <param name="cancellationToken"></param>
    /// <remarks>
    /// The returned <see cref="IAsyncEnumerable{TFeedback}"/> will complete either when the goal
    /// transitioned to terminal state (<see cref="ActionGoalStatus.Aborted"/>, 
    /// <see cref="ActionGoalStatus.Canceled"/> or <see cref="ActionGoalStatus.Succeeded"/>),
    /// or when current <see cref="INativeActionGoalContext"/> instance is disposed.
    /// <para>
    /// Additionally, caller takes the ownership of the returned <see cref="RosMessageBuffer"/>.
    /// </para>
    /// </remarks>
    /// <returns>
    /// An <see cref="IAsyncEnumerable{TFeedback}"/> for receiving feedbacks asynchronously.
    /// </returns>
    IAsyncEnumerable<RosMessageBuffer> ReadFeedbacksAsync(CancellationToken cancellationToken = default);
}

/// <summary>
/// A context object can be used for tracking
/// the state of an action goal sent by the action client. 
/// </summary>
/// <typeparam name="TFeedback">Type of the feedback message.</typeparam>
/// <typeparam name="TResult">Type of the result message.</typeparam>
public interface IActionGoalContext<TResult, TFeedback> : IActionGoalContext, IObservable<TFeedback>
    where TFeedback: IActionFeedback
    where TResult: IActionResult
{
    /// <summary>
    /// Gets the result of the goal.
    /// </summary>
    /// <remarks>
    /// If the goal didn't complete with <see cref="ActionGoalStatus.Succeeded"/> status, an exception will be thrown.
    /// <para>To wait for completion without throwing, use <see cref="IActionGoalContext.Completion"/>.</para>
    /// <para>
    /// Though calling this method after completion is allowed.
    /// Action servers may or may not return result according to their configured result caching policy.
    /// </para>
    /// </remarks>
    /// <param name="cancellationToken"></param>
    /// <returns></returns>
    Task<TResult> GetResultAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets the result of the goal.
    /// </summary>
    /// <remarks>
    /// If the goal didn't complete with <see cref="ActionGoalStatus.Succeeded"/> status, an exception will be thrown.
    /// <para>To wait for completion without throwing, use <see cref="IActionGoalContext.Completion"/>.</para>
    /// <para>
    /// Though calling this method after completion is allowed.
    /// Action servers may or may not return result according to their configured result caching policy.
    /// </para>
    /// </remarks>
    /// <param name="cancellationToken"></param>
    /// <param name="timeoutMilliseconds">Request timeout in milliseconds.</param>
    /// <returns></returns>
    Task<TResult> GetResultAsync(int timeoutMilliseconds, CancellationToken cancellationToken = default);

    /// <summary>
    /// Gets the result of the goal.
    /// </summary>
    /// <remarks>
    /// If the goal didn't complete with <see cref="ActionGoalStatus.Succeeded"/> status, an exception will be thrown.
    /// <para>To wait for completion without throwing, use <see cref="IActionGoalContext.Completion"/>.</para>
    /// <para>
    /// Though calling this method after completion is allowed.
    /// Action servers may or may not return result according to their configured result caching policy.
    /// </para>
    /// <para>
    /// Additionally, caller takes the ownership of the returned <see cref="RosMessageBuffer"/>.
    /// </para>
    /// </remarks>
    /// <param name="cancellationToken"></param>
    /// <param name="timeout">Request timeout in milliseconds.</param>
    /// <returns></returns>
    Task<TResult> GetResultAsync(TimeSpan timeout, CancellationToken cancellationToken = default);

    /// <summary>
    /// Asynchronously read all feedback messages.
    /// </summary>
    /// <param name="cancellationToken"></param>
    /// <remarks>
    /// The returned <see cref="IAsyncEnumerable{TFeedback}"/> will complete either when the goal
    /// transitioned to terminal state (<see cref="ActionGoalStatus.Aborted"/>, 
    /// <see cref="ActionGoalStatus.Canceled"/> or <see cref="ActionGoalStatus.Succeeded"/>),
    /// or when current <see cref="INativeActionGoalContext"/> instance is disposed.
    /// </remarks>
    /// <returns>
    /// An <see cref="IAsyncEnumerable{TFeedback}"/> for receiving feedbacks asynchronously.
    /// </returns>
    IAsyncEnumerable<TFeedback> ReadFeedbacksAsync(CancellationToken cancellationToken = default);
}