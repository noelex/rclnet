using Rosidl.Runtime;
using System.Text;

namespace Rcl.Actions.Server;

internal class ActionGoalHandlerWrapper<TGoal, TResult, TFeedback> : INativeActionGoalHandler
    where TGoal : IActionGoal
    where TResult : IActionResult
    where TFeedback : IActionFeedback
{
    private readonly IActionGoalHandler<TGoal, TResult, TFeedback> _innerHandler;
    private readonly Encoding _textEncoding;

    private readonly Dictionary<Guid, ActionGoalControllerWrapper> _wrappers = new();

    public ActionGoalHandlerWrapper(IActionGoalHandler<TGoal, TResult, TFeedback> innerHandler, Encoding textEncoding)
    {
        _innerHandler = innerHandler;
        _textEncoding = textEncoding;
    }

    public bool CanAccept(Guid id, RosMessageBuffer goal)
        => _innerHandler.CanAccept(id, (TGoal)TGoal.CreateFrom(goal.Data, _textEncoding));

    public async Task ExecuteAsync(INativeActionGoalController controller, RosMessageBuffer goal, RosMessageBuffer result, CancellationToken cancellationToken)
    {
        var r = await _innerHandler.ExecuteAsync(
            _wrappers[controller.GoalId], (TGoal)TGoal.CreateFrom(goal.Data, _textEncoding), cancellationToken);
        r.WriteTo(result.Data, _textEncoding);
    }

    public void OnAccepted(INativeActionGoalController controller)
    {
        var wrapper = new ActionGoalControllerWrapper(controller, _textEncoding);
        _wrappers[controller.GoalId] = wrapper;
        _innerHandler.OnAccepted(wrapper);
    }

    public void OnCompleted(INativeActionGoalController controller)
    {
        if (_wrappers.Remove(controller.GoalId, out var wrapper))
        {
            _innerHandler.OnCompleted(wrapper);
            wrapper.Dispose();
        }
    }

    internal class ActionGoalControllerWrapper : IActionGoalController<TFeedback>, IDisposable
    {
        private readonly INativeActionGoalController _nativeController;
        private readonly Encoding _textEncoding;

        private readonly RosMessageBuffer _feedbackBuffer = RosMessageBuffer.Create<TFeedback>();

        public ActionGoalControllerWrapper(INativeActionGoalController nativeController, Encoding textEncoding)
        {
            _nativeController = nativeController;
            _textEncoding = textEncoding;
        }

        public Guid GoalId => _nativeController.GoalId;

        public ActionGoalStatus Status => _nativeController.Status;

        public void Abort() => _nativeController.Abort();

        public void Dispose()
        {
            _feedbackBuffer.Dispose();
        }

        public void Report(TFeedback value)
        {
            value.WriteTo(_feedbackBuffer.Data, _textEncoding);
            _nativeController.Report(_feedbackBuffer);
        }

        public ValueTask ReportAsync(TFeedback feedback, CancellationToken cancellationToken = default)
        {
            feedback.WriteTo(_feedbackBuffer.Data, _textEncoding);
            return _nativeController.ReportAsync(_feedbackBuffer, cancellationToken);
        }
    }
}
