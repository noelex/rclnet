using Rcl;
using Rcl.Logging;
using Rosidl.Messages.Turtlesim;

var angle = 1.57f;
if (args.Length > 0 && float.TryParse(args[0], out var input))
{
    angle = input;
}

await using var ctx = new RclContext(args);

using var node = ctx.CreateNode("turtle_rotate");
using var client = node.CreateActionClient<
    RotateAbsoluteAction,
    RotateAbsoluteActionGoal,
    RotateAbsoluteActionResult,
    RotateAbsoluteActionFeedback>
    ("/turtle1/rotate_absolute");

node.Logger.LogInformation($"Waiting for action server {client.Name} ...");
if (!await client.TryWaitForServerAsync(5000))
{
    node.Logger.LogInformation("Action server is offline.");
    return;
}
node.Logger.LogInformation($"Sending goal ...");
using var goal = await client.SendGoalAsync(new RotateAbsoluteActionGoal(theta: angle));
goal.StatusChanged += (s) => node.Logger.LogInformation("Goal Status: " + s);
await foreach (var feedback in goal.ReadFeedbacksAsync())
{
    node.Logger.LogInformation("Remaining: " + feedback.Remaining);
}


var result = await goal.GetResultAsync();
node.Logger.LogInformation("Done: delta = " + result.Delta);