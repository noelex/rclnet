using Rosidl.Runtime;

namespace Rcl.Actions;

/// <summary>
/// A base action goal handler to be inherited by application for
/// processing action goals requested by action clients.
/// </summary>
public abstract class ActionGoalHandler : INativeActionGoalHandler
{
    /// <inheritdoc/>
    public virtual bool CanAccept(Guid id, RosMessageBuffer goal)
    {
        return true;
    }

    /// <inheritdoc/>
    public abstract Task ExecuteAsync(INativeActionGoalController controller,
        RosMessageBuffer goal, RosMessageBuffer result, CancellationToken cancellationToken);

    /// <inheritdoc/>
    public virtual void OnAccepted(INativeActionGoalController controller)
    {

    }

    /// <inheritdoc/>
    public virtual void OnCompleted(INativeActionGoalController controller)
    {

    }
}

/// <summary>
/// A base action goal handler to be inherited by application for
/// processing action goals requested by action clients.
/// </summary>
/// <typeparam name="TFeedback">Type of the feedback message.</typeparam>
/// <typeparam name="TGoal">Type of the goal message.</typeparam>
/// <typeparam name="TResult">Type of the result message.</typeparam>
public abstract class ActionGoalHandler<TGoal, TResult, TFeedback> : IActionGoalHandler<TGoal, TResult, TFeedback>
    where TGoal : IActionGoal
    where TResult : IActionResult
    where TFeedback : IActionFeedback
{
    /// <inheritdoc/>
    public virtual bool CanAccept(Guid id, TGoal goal)
    {
        return true;
    }

    /// <inheritdoc/>
    public abstract Task<TResult> ExecuteAsync(
        IActionGoalController<TFeedback> controller,TGoal goal, CancellationToken cancellationToken);

    /// <inheritdoc/>
    public virtual void OnAccepted(IActionGoalController<TFeedback> controller)
    {

    }

    /// <inheritdoc/>
    public virtual void OnCompleted(IActionGoalController<TFeedback> controller)
    {

    }
}