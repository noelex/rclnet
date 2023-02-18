using Rosidl.Runtime;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

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