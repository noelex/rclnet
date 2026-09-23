using Rcl.Actions;
using Rosidl.Messages.Tf2;

namespace Rcl.NET.Tests;

public class ActionTests
{
    private const int RequestTimeout = 10_000;
    private const int ServerOnlineTimeout = 5_000;

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

        await client.WaitForServerAsync(ServerOnlineTimeout);
        using var goal = await client.SendGoalAsync(new LookupTransformActionGoal(), RequestTimeout);
        var result = await goal.GetResultWithStatusAsync(RequestTimeout);
        Assert.True(result.IsSuccessful);
    }

    [Fact]
    public async Task AcceptNativeActionGoalWithoutExplicitTimeout()
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

        await client.WaitForServerAsync(ServerOnlineTimeout);
        using var goalBuffer = RosMessageBuffer.Create<LookupTransformActionGoal>();
        using var cts = new CancellationTokenSource(RequestTimeout);
        using var goal = await client.SendGoalAsync(goalBuffer, cts.Token);
        var result = await goal.GetResultWithStatusAsync(RequestTimeout);
        Assert.True(result.IsSuccessful);
        using var resultBuffer = result.Result;
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

        await client.WaitForServerAsync(ServerOnlineTimeout);
        await Assert.ThrowsAsync<RclException>(() =>
            client.SendGoalAsync(new LookupTransformActionGoal(), RequestTimeout));
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

        await client.WaitForServerAsync(ServerOnlineTimeout);
        using var goal = await client.SendGoalAsync(new LookupTransformActionGoal(), RequestTimeout);
        var result = await goal.GetResultWithStatusAsync(RequestTimeout);
        Assert.Equal(ActionGoalStatus.Aborted, result.Status);
    }

    [Fact]
    public async Task CancelActionGoal()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var actionName = NameGenerator.GenerateActionName();

        var handler = new TestHandler(executeWaitTime: -1);
        using var server = node.CreateActionServer<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName, handler);

        using var client = node.CreateActionClient<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName);

        await client.WaitForServerAsync(ServerOnlineTimeout);
        using var goal = await client.SendGoalAsync(new LookupTransformActionGoal(), RequestTimeout);
        await handler.ExecutionStarted.WaitAsync(TimeSpan.FromMilliseconds(RequestTimeout));
        await goal.CancelAsync(RequestTimeout);
        var result = await goal.GetResultWithStatusAsync(RequestTimeout);

        Assert.Equal(ActionGoalStatus.Canceled, result.Status);
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

        await client.WaitForServerAsync(ServerOnlineTimeout);
        using var goal = await client.SendGoalAsync(new LookupTransformActionGoal(), RequestTimeout);
        var result = await goal.GetResultWithStatusAsync(RequestTimeout);
        Assert.True(result.IsSuccessful);

        // Goal results are cached on action server by default,
        // we can call GetResult as many times as we want.
        result = await goal.GetResultWithStatusAsync(RequestTimeout);
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

        await client.WaitForServerAsync(ServerOnlineTimeout);
        using var goal = await client.SendGoalAsync(new LookupTransformActionGoal(), RequestTimeout);
        var result = await goal.GetResultWithStatusAsync(RequestTimeout);
        Assert.True(result.IsSuccessful);

        // Goal result is removed after the first call to GetResultWithStatusAsync,
        // now we should get a result with unknown status.
        //
        // TODO: Does this conform to the design of ROS 2 actions?
        result = await goal.GetResultWithStatusAsync(RequestTimeout);
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

        var handler = new TestHandler(
            feedbackCount: 5,
            feedbackInterval: 100,
            executeWaitTime: 100,
            asyncFeedback: useAsyncFeedback,
            waitForCompletion: true);
        using var server = node.CreateActionServer<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName, handler);

        using var client = node.CreateActionClient<
            LookupTransformAction,
            LookupTransformActionGoal,
            LookupTransformActionResult,
            LookupTransformActionFeedback>(actionName);

        await client.WaitForServerAsync(ServerOnlineTimeout);
        using var goal = await client.SendGoalAsync(new LookupTransformActionGoal(), RequestTimeout);
        var feedbackTask = CountFeedbacks(goal.ReadFeedbacksAsync(), 5);

        var count = await feedbackTask.WaitAsync(TimeSpan.FromMilliseconds(RequestTimeout));
        Assert.Equal(5, count);
        handler.Complete();

        var result = await goal.GetResultWithStatusAsync(RequestTimeout);
        Assert.True(result.IsSuccessful);

        static async Task<int> CountFeedbacks(
            IAsyncEnumerable<LookupTransformActionFeedback> items,
            int expectedCount)
        {
            var count = 0;
            await foreach (var f in items)
            {
                if (++count == expectedCount)
                {
                    break;
                }
            }
            return count;
        }
    }

    private class TestHandler : ActionGoalHandler<LookupTransformActionGoal, LookupTransformActionResult, LookupTransformActionFeedback>
    {
        private readonly bool _acceptGoal, _throwOnExecute, _asyncFeedback;
        private readonly int _executeWaitTime, _feedbackCount, _feedbackInterval;
        private readonly TaskCompletionSource _executionStarted = new(TaskCreationOptions.RunContinuationsAsynchronously);
        private readonly TaskCompletionSource? _completionSignal;

        public TestHandler(bool acceptGoal = true, bool throwOnExecute = false,
            int executeWaitTime = 0, int feedbackCount = 0, int feedbackInterval = 1000,
            bool asyncFeedback = false, bool waitForCompletion = false)
        {
            _acceptGoal = acceptGoal;
            _throwOnExecute = throwOnExecute;
            _executeWaitTime = executeWaitTime;
            _feedbackCount = feedbackCount;
            _feedbackInterval = feedbackInterval;
            _asyncFeedback = asyncFeedback;
            if (waitForCompletion)
            {
                _completionSignal = new(TaskCreationOptions.RunContinuationsAsynchronously);
            }
        }

        public Task ExecutionStarted => _executionStarted.Task;

        public void Complete() => _completionSignal?.TrySetResult();

        public override bool CanAccept(Guid id, LookupTransformActionGoal goal)
        {
            return _acceptGoal;
        }

        public override async Task<LookupTransformActionResult> ExecuteAsync(
            IActionGoalController<LookupTransformActionFeedback> controller,
            LookupTransformActionGoal goal,
            CancellationToken cancellationToken)
        {
            _executionStarted.TrySetResult();
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

            if (_completionSignal != null)
            {
                await _completionSignal.Task.WaitAsync(cancellationToken);
            }

            return new LookupTransformActionResult();
        }
    }
}
