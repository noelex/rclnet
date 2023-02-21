using Rosidl.Runtime;

namespace Rcl.Actions;

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