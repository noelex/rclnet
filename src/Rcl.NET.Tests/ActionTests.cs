using Rcl.Actions;
using Rosidl.Messages.Tf2;

namespace Rcl.NET.Tests;

public class ActionTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(50)]
    [InlineData(100)]
    [InlineData(500)]
    public async Task SendGoalAsyncTimeout(int timeoutMs)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        using var client = node.CreateActionClient<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(NameGenerator.GenerateActionName());

        await Assert.ThrowsAsync<TimeoutException>(() => client.SendGoalAsync(new LookupTransformActionGoal(), timeoutMs));
    }

    [Fact]
    public async Task AcceptActionGoal()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var actionName = NameGenerator.GenerateActionName();

        using var server = node.CreateActionServer<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName, new TestHandler());

        using var client = node.CreateActionClient<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName);

        using var goal = await client.SendGoalAsync(new LookupTransformActionGoal(), 1000);
        var result = await goal.GetResultWithStatusAsync(1000);
        Assert.True(result.IsSuccessful);
    }

    [Fact]
    public async Task RejectActionGoal()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var actionName = NameGenerator.GenerateActionName();

        using var server = node.CreateActionServer<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName, new TestHandler(acceptGoal: false));

        using var client = node.CreateActionClient<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName);

        await Assert.ThrowsAsync<RclException>(() => client.SendGoalAsync(new LookupTransformActionGoal(), 1000));
    }

    [Fact]
    public async Task AbortActionGoal()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var actionName = NameGenerator.GenerateActionName();

        using var server = node.CreateActionServer<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName, new TestHandler(throwOnExecute: true));

        using var client = node.CreateActionClient<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName);

        using var goal = await client.SendGoalAsync(new LookupTransformActionGoal(), 1000);
        var result = await goal.GetResultWithStatusAsync(1000);
        Assert.Equal(ActionGoalStatus.Aborted, result.Status);
    }

    [Fact]
    public async Task CancelActionGoal()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var actionName = NameGenerator.GenerateActionName();

        using var server = node.CreateActionServer<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName, new TestHandler(executeWaitTime: -1));

        using var client = node.CreateActionClient<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName);

        using var goal = await client.SendGoalAsync(new LookupTransformActionGoal(), 1000);
        _ = CancelGoalAfter(goal, 100);
        var result = await goal.GetResultWithStatusAsync(1000);

        Assert.Equal(ActionGoalStatus.Canceled, result.Status);

        static async Task CancelGoalAfter(IActionGoalContext goal, int milliseconds)
        {
            await Task.Delay(milliseconds);
            await goal.CancelAsync();
        }
    }

    [Fact]
    public async Task CacheGoalResults()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var actionName = NameGenerator.GenerateActionName();

        using var server = node.CreateActionServer<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName, new TestHandler());

        using var client = node.CreateActionClient<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName);

        using var goal = await client.SendGoalAsync(new LookupTransformActionGoal(), 1000);
        var result = await goal.GetResultWithStatusAsync(1000);
        Assert.True(result.IsSuccessful);

        // Goal results are cached on action server by default,
        // we can call GetResult as many times as we want.
        result = await goal.GetResultWithStatusAsync(1000);
        Assert.True(result.IsSuccessful);
    }

    [Fact]
    public async Task NoResultCache()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var actionName = NameGenerator.GenerateActionName();

        using var server = node.CreateActionServer<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName, new TestHandler(), new(resultTimeout: TimeSpan.Zero));

        using var client = node.CreateActionClient<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName);

        using var goal = await client.SendGoalAsync(new LookupTransformActionGoal(), 1000);
        var result = await goal.GetResultWithStatusAsync(1000);
        Assert.True(result.IsSuccessful);

        // Goal result is removed after the first call to GetResultWithStatusAsync,
        // now we should get a result with unknown status.
        //
        // TODO: Does this conform to the design of ROS 2 actions?
        result = await goal.GetResultWithStatusAsync(1000);
        Assert.Equal(ActionGoalStatus.Unknown, result.Status);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public async Task ActionFeedbacks(bool useAsyncFeedback)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var actionName = NameGenerator.GenerateActionName();

        using var server = node.CreateActionServer<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName, new TestHandler(
                feedbackCount: 5,
                feedbackInterval: 100,
                executeWaitTime: 100,
                asyncFeedback: useAsyncFeedback)
            );

        using var client = node.CreateActionClient<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName);

        using var goal = await client.SendGoalAsync(new LookupTransformActionGoal(), 1000);
        var feedbackTask = CountFeedbacks(goal.ReadFeedbacksAsync());

        var result = await goal.GetResultWithStatusAsync(10000);
        Assert.True(result.IsSuccessful);

        var count = await feedbackTask;
        Assert.Equal(5, count);

        static async Task<int> CountFeedbacks(IAsyncEnumerable<LookupTransformActionFeedback> items)
        {
            var count = 0;
            await foreach(var f in items)
            {
                count++;
            }
            return count;
        }
    }

    private class TestHandler : ActionGoalHandler<LookupTransformActionGoal, LookupTransformActionResult, LookupTransformActionFeedback>
    {
        private readonly bool _acceptGoal, _throwOnExecute, _asyncFeedback;
        private readonly int _executeWaitTime, _feedbackCount, _feedbackInterval;

        public TestHandler(bool acceptGoal = true, bool throwOnExecute = false,
            int executeWaitTime = 0, int feedbackCount = 0, int feedbackInterval = 1000, bool asyncFeedback = false)
        {
            _acceptGoal = acceptGoal;
            _throwOnExecute = throwOnExecute;
            _executeWaitTime = executeWaitTime;
            _feedbackCount = feedbackCount;
            _feedbackInterval = feedbackInterval;
            _asyncFeedback = asyncFeedback;
        }

        public override bool CanAccept(Guid id, LookupTransformActionGoal goal)
        {
            return _acceptGoal;
        }

        public override async Task<LookupTransformActionResult> ExecuteAsync(
            IActionGoalController<LookupTransformActionFeedback> controller,
            LookupTransformActionGoal goal,
            CancellationToken cancellationToken)
        {
            await Task.Delay(_executeWaitTime, cancellationToken);
            if (_throwOnExecute)
            {
                throw new Exception("Aborting goal");
            }

            var count = _feedbackCount;
            while (!cancellationToken.IsCancellationRequested)
            {
                if (--count < 0)
                {
                    break;
                }

                if (_asyncFeedback)
                {
                    await controller.ReportAsync(new(), cancellationToken);
                }
                else
                {
                    controller.Report(new());
                }

                await Task.Delay(_feedbackInterval, cancellationToken);
            }

            return new LookupTransformActionResult();
        }
    }
}
