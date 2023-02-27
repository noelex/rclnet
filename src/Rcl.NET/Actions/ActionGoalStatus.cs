using Rosidl.Messages.Action;

namespace Rcl.Actions;

/// <summary>
/// Represents the status of an action goal.
/// </summary>
public enum ActionGoalStatus : sbyte
{
    /// <summary>
    /// The status of the goal is unknown.
    /// </summary>
    Unknown = GoalStatus.STATUS_UNKNOWN,

    /// <summary>
    /// The goal has been accepted and is awaiting execution.
    /// </summary>
    Accepted = GoalStatus.STATUS_ACCEPTED,

    /// <summary>
    /// The goal is currently being executed by the action server.
    /// </summary>
    Executing = GoalStatus.STATUS_EXECUTING,

    /// <summary>
    /// The client has requested that the goal be canceled and the action server has
    /// accepted the cancel request.
    /// </summary>
    Canceling = GoalStatus.STATUS_CANCELING,

    /// <summary>
    /// The goal was achieved successfully by the action server.
    /// </summary>
    Succeeded = GoalStatus.STATUS_SUCCEEDED,

    /// <summary>
    /// The goal was canceled after an external request from an action client.
    /// </summary>
    Canceled = GoalStatus.STATUS_CANCELED,

    /// <summary>
    /// The goal was terminated by the action server without an external request,
    /// usually due to server shutdown, or action goal preemption.
    /// </summary>
    Aborted = GoalStatus.STATUS_ABORTED
}
