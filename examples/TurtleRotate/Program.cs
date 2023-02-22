﻿using Rcl;
using Rosidl.Messages.Turtlesim;

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

Console.WriteLine($"Waiting for action server {client.Name} ...");
if (!await client.TryWaitForServerAsync(5000))
{
    Console.WriteLine("Action server is offline.");
    return;
}

using var goal = await client.SendGoalAsync(new RotateAbsoluteActionGoal(theta: angle));
goal.StatusChanged += (s) => Console.WriteLine("Goal Status: " + s);
await foreach (var feedback in goal.ReadFeedbacksAsync())
{
    Console.WriteLine("Remaining: " + feedback.Remaining);
}


var result = await goal.GetResultAsync();
Console.WriteLine("Done: delta = " + result.Delta);