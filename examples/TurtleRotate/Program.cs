using Rcl;
using Rosidl.Messages.Turtlesim;

using var ctx = new RclContext(args);
using var node = ctx.CreateNode("/turtle_rotate");
using var client = node.CreateActionClient<
    RotateAbsoluteAction,
    RotateAbsoluteActionGoal,
    RotateAbsoluteActionResult,
    RotateAbsoluteActionFeedback>
    ("/turtle1/rotate_absolute");

using var goal = await client.SendGoalAsync(new RotateAbsoluteActionGoal(theta: 1.57f));
await foreach (var feedback in goal.ReadFeedbacksAsync())
{
    Console.WriteLine("Remaining: " + feedback.Remaining);
}
await goal.Completion;
Console.WriteLine(goal.Status);