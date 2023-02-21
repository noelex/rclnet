using Rcl;
using Rosidl.Messages.Turtlesim;
using Rosidl.Messages.Action;
using Rcl.Actions;

var angle = 1.57f;
if (args.Length > 0 && float.TryParse(args[0], out var input))
{
    angle = input;
}

using var ctx = new RclContext(args);
using var node = ctx.CreateNode("turtle_rotate");
using var client = node.CreateActionClient<
    RotateAbsoluteAction,
    RotateAbsoluteActionGoal,
    RotateAbsoluteActionResult,
    RotateAbsoluteActionFeedback>
    ("/turtle1/rotate_absolute");

using var goal = await client.SendGoalAsync(new RotateAbsoluteActionGoal(theta: angle));
goal.StatusChanged += (s) => Console.WriteLine(s);
await foreach (var feedback in goal.ReadFeedbacksAsync())
{
    Console.WriteLine("Remaining: " + feedback.Remaining);
}

if(ActionGoalStatus.Succeeded == await goal.Completion)
{
    var result = await goal.GetResultAsync();
    Console.WriteLine("Done: delta = " + result.Delta);
}
else
{
    Console.WriteLine("Failed.");
    await goal.GetResultAsync();
}