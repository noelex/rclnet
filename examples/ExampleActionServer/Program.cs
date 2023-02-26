// See https://aka.ms/new-console-template for more information
using Rcl;
using Rcl.Actions;
using Rcl.Logging;
using Rosidl.Messages.Turtlesim;

using var ctx = new RclContext(args);
using var node = ctx.CreateNode("example_action_server");

using var server = node.CreateActionServer<RotateAbsoluteAction>(
    "/turtle1/rotate_absolute", new MyNativeActionServer(node, preemptive: true), resultTimeout: TimeSpan.Zero);

node.Logger.LogInformation("Action server started.");
await Task.Run(()=>Console.ReadLine());

class MyNativeActionServer : ActionGoalHandler
{
    private readonly bool _preemptive;
    private readonly IRclNode _node;

    private INativeActionGoalController? _currentController;

    public MyNativeActionServer(IRclNode node, bool preemptive = false)
    {
        _node = node;
        _preemptive = preemptive;
    }

    public override void OnAccepted(INativeActionGoalController controller)
    {
        if (_preemptive)
        {
            _currentController?.Abort();
            _currentController = controller;
        }

        base.OnAccepted(controller);
    }

    public override void OnCompleted(INativeActionGoalController controller)
    {
        if (_preemptive)
        {
            _currentController = null;
        }

        base.OnCompleted(controller);
    }

    public override async Task ExecuteAsync(
        INativeActionGoalController controller,
        RosMessageBuffer goal, RosMessageBuffer result, CancellationToken cancellationToken)
    {
        using var feedback = RosMessageBuffer.Create<RotateAbsoluteActionFeedback>();
        var theta = goal.AsRef<RotateAbsoluteActionGoal.Priv>().Theta;

        _node.Logger.LogInformation("Received goal: " + theta);
        using var timer = _node.Context.CreateTimer(TimeSpan.FromMilliseconds(10));
        var current = theta;
        while (current >= 0)
        {
            current -= 0.1f;

            feedback.AsRef<RotateAbsoluteActionFeedback.Priv>().Remaining = Math.Abs(current);
            controller.Report(feedback);

            await timer.WaitOneAsync(cancellationToken);
        }

        result.AsRef<RotateAbsoluteActionResult.Priv>().Delta = theta;
    }
}
