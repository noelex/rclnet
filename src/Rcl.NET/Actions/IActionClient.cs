﻿using Rosidl.Runtime;

namespace Rcl.Actions;

/// <summary>
/// Sends actions goals and tracks their progress.
/// </summary>
/// <typeparam name="TGoal">Type of the goal message.</typeparam>
/// <typeparam name="TResult">Type of the result message</typeparam>
/// <typeparam name="TFeedback">Type of the feedback message.</typeparam>
public interface IActionClient<TGoal, TResult, TFeedback> : IRclObject
    where TGoal : IActionGoal
    where TResult : IActionResult
    where TFeedback : IActionFeedback
{
    /// <summary>
    /// Name of the action.
    /// </summary>
    string Name { get; }

    /// <summary>
    /// Checks whether there's any action server from handling action goals sent by this client.
    /// </summary>
    bool IsServerAvailable { get; }

    /// <summary>
    /// Try wait for the server become available, or until timeout.
    /// </summary>
    /// <param name="timeoutMilliseconds">Timeout in milliseconds. Specify <see cref="Timeout.Infinite"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns><see langword="true"/> if the server is available, <see langword="false"/> if timed out.</returns>
    Task<bool> TryWaitForServerAsync(int timeoutMilliseconds, CancellationToken cancellationToken = default);

    /// <summary>
    /// Try wait for the server become available, or until timeout.
    /// </summary>
    /// <param name="timeout">Timeout of the operation. Specify <see cref="Timeout.InfiniteTimeSpan"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns><see langword="true"/> if the server is available, <see langword="false"/> if timed out.</returns>
    Task<bool> TryWaitForServerAsync(TimeSpan timeout, CancellationToken cancellationToken = default);

    /// <summary>
    /// Wait until the server become available.
    /// </summary>
    /// <param name="timeoutMilliseconds">Timeout in milliseconds. Specify <see cref="Timeout.Infinite"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    /// <exception cref="TimeoutException">The server didn't become available during the wait.</exception>
    Task WaitForServerAsync(int timeoutMilliseconds, CancellationToken cancellationToken = default);

    /// <summary>
    /// Wait until the server become available.
    /// </summary>
    /// <param name="timeout">Timeout of the operation. Specify <see cref="Timeout.InfiniteTimeSpan"/> to wait indefinitely.</param>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    /// <exception cref="TimeoutException">The server didn't become available during the wait.</exception>
    Task WaitForServerAsync(TimeSpan timeout, CancellationToken cancellationToken = default);

    /// <summary>
    /// Wait until the server become available.
    /// </summary>
    /// <param name="cancellationToken">A <see cref="CancellationToken"/> for canceling the operation.</param>
    /// <returns></returns>
    Task WaitForServerAsync(CancellationToken cancellationToken = default);

    /// <summary>
    /// Send a goal to the action server and receive result and feedbacks using native message buffers.
    /// </summary>
    /// <remarks>
    /// This method does not take the ownership of <paramref name="goalBuffer"/>.
    /// </remarks>
    /// <param name="goalBuffer">A <see cref="RosMessageBuffer"/> which contains the goal message.</param>
    /// <param name="cancellationToken"></param>
    /// <returns>An <see cref="INativeActionGoalContext"/> for tracking the status of the goal.</returns>
    Task<INativeActionGoalContext> SendGoalAsync(RosMessageBuffer goalBuffer, CancellationToken cancellationToken = default);

    /// <summary>
    /// Send a goal to the action server and receive result and feedbacks using native message buffers.
    /// </summary>
    /// <remarks>
    /// This method does not take the ownership of <paramref name="goalBuffer"/>.
    /// </remarks>
    /// <param name="goalBuffer">A <see cref="RosMessageBuffer"/> which contains the goal message.</param>
    /// <param name="timeoutMilliseconds">Request timeout in milliseconds.</param>
    /// <param name="cancellationToken"></param>
    /// <returns>An <see cref="INativeActionGoalContext"/> for tracking the status of the goal.</returns>
    Task<INativeActionGoalContext> SendGoalAsync(RosMessageBuffer goalBuffer, int timeoutMilliseconds, CancellationToken cancellationToken = default);

    /// <summary>
    /// Send a goal to the action server and receive result and feedbacks using native message buffers.
    /// </summary>
    /// <remarks>
    /// This method does not take the ownership of <paramref name="goalBuffer"/>.
    /// </remarks>
    /// <param name="goalBuffer">A <see cref="RosMessageBuffer"/> which contains the goal message.</param>
    /// <param name="timeout">Timeout of the request.</param>
    /// <param name="cancellationToken"></param>
    /// <returns>An <see cref="INativeActionGoalContext"/> for tracking the status of the goal.</returns>
    Task<INativeActionGoalContext> SendGoalAsync(RosMessageBuffer goalBuffer, TimeSpan timeout, CancellationToken cancellationToken = default);

    /// <summary>
    /// Send a goal to the action server.
    /// </summary>
    /// <param name="goal">A <typeparamref name="TGoal"/> object which contains the goal message.</param>
    /// <param name="cancellationToken"></param>
    /// <returns>An <see cref="IActionGoalContext"/> for tracking the status of the goal.</returns>
    Task<IActionGoalContext<TResult, TFeedback>> SendGoalAsync(TGoal goal, CancellationToken cancellationToken = default);

    /// <summary>
    /// Send a goal to the action server.
    /// </summary>
    /// <param name="goal">A <typeparamref name="TGoal"/> object which contains the goal message.</param>
    /// <param name="cancellationToken"></param>
    /// <param name="timeout">Timeout of the request.</param>
    /// <returns>An <see cref="IActionGoalContext"/> for tracking the status of the goal.</returns>
    Task<IActionGoalContext<TResult, TFeedback>> SendGoalAsync(TGoal goal, TimeSpan timeout, CancellationToken cancellationToken = default);

    /// <summary>
    /// Send a goal to the action server.
    /// </summary>
    /// <param name="goal">A <typeparamref name="TGoal"/> object which contains the goal message.</param>
    /// <param name="cancellationToken"></param>
    /// <param name="timeoutMilliseconds">Timeout of the request in milliseconds.</param>
    /// <returns>An <see cref="IActionGoalContext"/> for tracking the status of the goal.</returns>
    Task<IActionGoalContext<TResult, TFeedback>> SendGoalAsync(TGoal goal, int timeoutMilliseconds, CancellationToken cancellationToken = default);
}