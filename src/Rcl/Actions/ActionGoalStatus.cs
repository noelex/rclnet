using Rosidl.Messages.Action;

namespace Rcl.Actions;

public enum ActionGoalStatus : sbyte
{
    Unknown = GoalStatus.STATUS_UNKNOWN,
    Accepted = GoalStatus.STATUS_ACCEPTED,
    Executing = GoalStatus.STATUS_EXECUTING,
    Canceling = GoalStatus.STATUS_CANCELING,
    Succeeded = GoalStatus.STATUS_SUCCEEDED,
    Canceled = GoalStatus.STATUS_CANCELED,
    Aborted = GoalStatus.STATUS_ABORTED
}
