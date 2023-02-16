using Rosidl.Runtime;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Actions;

public interface IActionGoalContext : IRclObject
{
    event Action<ActionGoalStatus>? StatusChanged;

    Task<ActionGoalStatus> Completion { get; }

    Guid GoalId { get; }

    ActionGoalStatus Status { get; }

    Task CancelAsync(TimeSpan timeout, CancellationToken cancellationToken = default);
}

public interface INativeActionGoalContext : IActionGoalContext
{
    Task<RosMessageBuffer> GetResultAsync(CancellationToken cancellationToken = default);

    Task<RosMessageBuffer> GetResultAsync(int timeoutMilliseconds, CancellationToken cancellationToken = default);

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
    /// <returns></returns>
    IAsyncEnumerable<RosMessageBuffer> ReadFeedbacksAsync(CancellationToken cancellationToken = default);
}

public interface IActionGoalContext<TResult, TFeedback> : IActionGoalContext, IObservable<TFeedback>
    where TFeedback: IActionFeedback
    where TResult: IActionResult
{
    Task<TResult> GetResultAsync(CancellationToken cancellationToken = default);

    Task<TResult> GetResultAsync(int timeoutMilliseconds, CancellationToken cancellationToken = default);

    Task<TResult> GetResultAsync(TimeSpan timeout, CancellationToken cancellationToken = default);

    /// <summary>
    /// Asynchronously read all feedback messages.
    /// </summary>
    /// <remarks>
    /// The returned <see cref="IAsyncEnumerable{TFeedback}"/> will complete either when the goal
    /// transitioned to terminal state (<see cref="ActionGoalStatus.Aborted"/>, 
    /// <see cref="ActionGoalStatus.Canceled"/> or <see cref="ActionGoalStatus.Succeeded"/>),
    /// or when current <see cref="IActionGoalContext{TResult,TFeedback}"/> instance is disposed.
    /// </remarks>
    /// <param name="cancellationToken"></param>
    /// <returns></returns>
    IAsyncEnumerable<TFeedback> ReadFeedbacksAsync(CancellationToken cancellationToken = default);
}