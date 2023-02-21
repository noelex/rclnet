// See https://aka.ms/new-console-template for more information
using Rcl;
using Rcl.Actions;
using Rosidl.Messages.Turtlesim;

using var ctx = new RclContext(args);
using var node = ctx.CreateNode("example_action_server");

//using var server = node.CreateActionServer<
//    RotateAbsoluteAction,
//    RotateAbsoluteActionGoal,
//    RotateAbsoluteActionResult,
//    RotateAbsoluteActionFeedback>("/turtle1/rotate_absolute", new MyActionServer());

using var server = node.CreateActionServer<RotateAbsoluteAction>(
    "/turtle1/rotate_absolute", new MyNativeActionServer());

Console.WriteLine("Action server started.");
Console.ReadLine();

class MyActionServer : ActionGoalHandler<RotateAbsoluteActionGoal, RotateAbsoluteActionResult, RotateAbsoluteActionFeedback>
{
    public override async Task<RotateAbsoluteActionResult> ExecuteAsync(
        IActionGoalController<RotateAbsoluteActionFeedback> controller,
        RotateAbsoluteActionGoal goal, CancellationToken cancellationToken)
    {
        Console.WriteLine("Received goal: " + goal.Theta);

        var current = goal.Theta;
        var feedback = new RotateAbsoluteActionFeedback();
        while (current >= 0)
        {
            current -= 0.1f;

            feedback.Remaining = Math.Abs(current);
            controller.Report(feedback);

            await Task.Delay(10, cancellationToken);
        }

        return new RotateAbsoluteActionResult(goal.Theta);
    }
}

class MyNativeActionServer : ActionGoalHandler
{
    public override async Task ExecuteAsync(
        INativeActionGoalController controller,
        RosMessageBuffer goal, RosMessageBuffer result, CancellationToken cancellationToken)
    {
        using var feedback = RosMessageBuffer.Create<RotateAbsoluteActionFeedback>();
        var theta =  goal.AsRef<RotateAbsoluteActionGoal.Priv>().Theta;

        Console.WriteLine("Received goal: " + theta);

        var current = theta;
        while (current >= 0)
        {
            current -= 0.1f;

            feedback.AsRef<RotateAbsoluteActionFeedback.Priv>().Remaining = Math.Abs(current);
            controller.Report(feedback);

            await Task.Delay(10, cancellationToken);
        }

        result.AsRef<RotateAbsoluteActionResult.Priv>().Delta = theta;
    }
}