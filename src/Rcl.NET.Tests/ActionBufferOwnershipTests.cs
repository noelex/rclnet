using Rcl.Actions;
using Rcl.Actions.Server;
using Rcl.Internal;
using Rcl.Introspection;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Messages.Action;
using Rosidl.Messages.Ros2csAbiTest;
using Rosidl.Runtime;
using System.Collections.Concurrent;
using System.Reflection;
using System.Text;

namespace Rcl.NET.Tests;

public class ActionBufferOwnershipTests : IDisposable
{
    private readonly HandleReleaseError?[] _errorsBefore = HandleReleaseDiagnostics.Snapshot();

    public void Dispose()
    {
        Assert.Empty(HandleReleaseDiagnostics.Snapshot().Except(_errorsBefore));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task ShutdownDuringAdmissionDoesNotNotifyUnacceptedGoal(bool closeFromCallback)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = (RclNodeImpl)context.CreateNode(NameGenerator.GenerateNodeName());
        using var admission = new LifecycleCheckpoint();
        var handler = new AdmissionHandler();
        using var server = new TrackingServer(node, handler);
        handler.CheckAdmission = () =>
        {
            if (closeFromCallback)
            {
                server.Dispose();
            }
            else
            {
                admission.Pause();
            }
        };
        var introspection = new ActionIntrospection(SequenceAction.GetTypeSupportHandle());
        using var request = introspection.GoalService.Request.CreateBuffer();
        using var response = introspection.GoalService.Response.CreateBuffer();
        var sending = InvokeAdmissionAsync(context, server, request, response);
        Exception? error;

        try
        {
            if (!closeFromCallback)
            {
                await admission.Entered;
                server.Dispose();
            }
        }
        finally
        {
            admission.Resume();
            error = await Record.ExceptionAsync(() => sending);
        }

        Assert.Empty(handler.Notifications);
        Assert.Equal(0, handler.Executions);
        Assert.Equal(1, server.GoalsReleased);
        Assert.Equal(1, server.ResultsReleased);
        Assert.Equal(1, server.FeedbacksReleased);
        Assert.False(RosidlRuntime.NativeAbi == RosidlNativeAbi.V1
            ? response.AsRef<SendGoalResponse>().Accepted
            : response.AsRef<SendGoalResponseV2>().Accepted);
        Assert.Null(error);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task AdmissionPreservesHandlerExceptionsAndPairsStartedNotifications(bool throwOnAccepted)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = (RclNodeImpl)context.CreateNode(NameGenerator.GenerateNodeName());
        var failure = new ObjectDisposedException("handler");
        var handler = new AdmissionHandler
        {
            CheckAdmission = () =>
            {
                if (!throwOnAccepted)
                {
                    throw failure;
                }
            },
            AcceptanceError = throwOnAccepted ? failure : null
        };
        using var server = new TrackingServer(node, handler);
        var introspection = new ActionIntrospection(SequenceAction.GetTypeSupportHandle());
        using var request = introspection.GoalService.Request.CreateBuffer();
        using var response = introspection.GoalService.Response.CreateBuffer();
        var error = await Record.ExceptionAsync(() => InvokeAdmissionAsync(context, server, request, response));
        server.Dispose();

        Assert.Same(failure, error);
        Assert.Equal(throwOnAccepted ? new[] { "accepted", "completed" } : Array.Empty<string>(),
            handler.Notifications);
        Assert.Equal(0, handler.Executions);
        Assert.Equal(throwOnAccepted ? 1 : 0, server.GoalsReleased);
        Assert.Equal(throwOnAccepted ? 1 : 0, server.ResultsReleased);
        Assert.Equal(throwOnAccepted ? 1 : 0, server.FeedbacksReleased);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task ShutdownKeepsHandlerBuffersAliveUntilExecutionCompletes(bool closeContext)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = (RclNodeImpl)context.CreateNode(NameGenerator.GenerateNodeName());
        var handler = new ControlledHandler();
        using var server = new TrackingServer(node, handler);
        handler.BuffersAreLive = () => server.ResultsReleased == 0;
        using var client = CreateClient(node, server.Name);
        await client.WaitForServerAsync();
        using var goalBuffer = RosMessageBuffer.Create<SequenceActionGoal>();
        using var goal = await client.SendGoalAsync(goalBuffer);
        await handler.Started.Task;

        try
        {
            server.Dispose();
            server.Dispose();

            if (closeContext)
            {
                await context.DisposeAsync();
            }

            Assert.True(handler.CancellationToken.IsCancellationRequested);
            Assert.Equal(0, server.ResultsReleased);
            Assert.Equal(0, server.FeedbacksReleased);
        }
        finally
        {
            handler.Resume.TrySetResult();
        }

        await handler.Completed.Task;
        await LifecycleAssert.EventuallyAsync(() => server.ResultsReleased == 1 && server.FeedbacksReleased == 1);
        Assert.True(handler.AccessedBuffersAfterResume);
    }

    [SkippableFact]
    public async Task ShutdownBeforeQueuedExecutionReleasesItsReservedBuffers()
    {
        TestConfig.SkipIfMultiContextEndpointTeardownCanCrash();

        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        await using var clientContext = new RclContext(TestConfig.DefaultContextArguments);
        using var node = (RclNodeImpl)context.CreateNode(NameGenerator.GenerateNodeName());
        using var clientNode = clientContext.CreateNode(NameGenerator.GenerateNodeName());
        using var queuedExecution = new LifecycleCheckpoint();
        var handler = new ControlledHandler
        {
            Accepted = () => context.SynchronizationContext.Post(_ => queuedExecution.Pause(), null)
        };
        using var server = new TrackingServer(node, handler);
        using var client = CreateClient(clientNode, server.Name);
        await client.WaitForServerAsync();
        using var goalBuffer = RosMessageBuffer.Create<SequenceActionGoal>();
        using var goal = await client.SendGoalAsync(goalBuffer);
        await queuedExecution.Entered;

        try
        {
            server.Dispose();
            Assert.False(handler.Started.Task.IsCompleted);
            Assert.Equal(0, server.ResultsReleased);
        }
        finally
        {
            queuedExecution.Resume();
            handler.Resume.TrySetResult();
        }

        await handler.Completed.Task;
        await LifecycleAssert.EventuallyAsync(() => server.ResultsReleased == 1 && server.FeedbacksReleased == 1);
        Assert.False(handler.Started.Task.IsCompleted);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task ShutdownSerializesCompletionCallbacks(bool closeContext)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = (RclNodeImpl)context.CreateNode(NameGenerator.GenerateNodeName());
        using var firstCompletion = new LifecycleCheckpoint();
        var handler = new CompletionHandler(firstCompletion);
        using var server = new TrackingServer(node, handler);
        using var client = CreateClient(node, server.Name);
        await client.WaitForServerAsync();
        using var goalBuffer = RosMessageBuffer.Create<SequenceActionGoal>();
        using var first = await client.SendGoalAsync(goalBuffer);
        using var second = await client.SendGoalAsync(goalBuffer);
        await handler.Started.Task;

        try
        {
            server.Dispose();

            if (closeContext)
            {
                await context.DisposeAsync();
            }

            handler.FirstExecution.TrySetResult();
            await firstCompletion.Entered;
            handler.SecondExecution.TrySetResult();

            // Keep the first callback active while the second goal finishes on the shutdown fallback.
            await Task.WhenAny(handler.SecondCompletion.Task, Task.Delay(1_000));
            Assert.False(handler.SecondCompletion.Task.IsCompleted);
            Assert.Equal(0, server.ResultsReleased);
        }
        finally
        {
            firstCompletion.Resume();
            handler.FirstExecution.TrySetResult();
            handler.SecondExecution.TrySetResult();
            await handler.Completed.Task;
            await LifecycleAssert.EventuallyAsync(() => server.ResultsReleased == 2 && server.FeedbacksReleased == 2);
        }
    }

    [Fact]
    public async Task FeedbackPublicationsOwnIndependentCopiesUntilCompletion()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = (RclNodeImpl)context.CreateNode(NameGenerator.GenerateNodeName());
        var handler = new ControlledHandler();
        using var server = new TrackingServer(node, handler, holdPublications: true);
        using var client = CreateClient(node, server.Name);
        await client.WaitForServerAsync();
        using var goalBuffer = RosMessageBuffer.Create<SequenceActionGoal>();
        using var goal = await client.SendGoalAsync(goalBuffer);
        await handler.Started.Task;
        Task first = Task.CompletedTask, second = Task.CompletedTask;

        try
        {
            using (var feedback = RosMessageBuffer.Create<SequenceActionFeedback>())
            {
                new SequenceActionFeedback([new(new PrimitiveSequence([1]))]).WriteTo(feedback.Data, Encoding.UTF8);
                first = handler.Controller!.ReportAsync(feedback).AsTask();
                new SequenceActionFeedback([new(new PrimitiveSequence([2]))]).WriteTo(feedback.Data, Encoding.UTF8);
                second = handler.Controller.ReportAsync(feedback).AsTask();
            }

            Assert.Equal(2, server.PublicationBuffers.Count);
            Assert.Equal(2, server.PublicationBuffers.Distinct().Count());
            server.Dispose();
            handler.Resume.TrySetResult();
            await handler.Completed.Task;
            Assert.Equal(0, server.ResultsReleased);
            Assert.Equal(0, server.FeedbacksReleased);
        }
        finally
        {
            handler.Resume.TrySetResult();
            server.ResumePublications.TrySetResult();
        }

        await Task.WhenAll(first, second);
        Assert.Equal(new byte[] { 1, 2 }, server.PublishedValues.Order().ToArray());
        await LifecycleAssert.EventuallyAsync(() => server.ResultsReleased == 1 && server.FeedbacksReleased == 3);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task FailedOrCanceledFeedbackDoesNotTakeInputOwnership(bool canceled)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = (RclNodeImpl)context.CreateNode(NameGenerator.GenerateNodeName());
        var handler = new ControlledHandler();
        using var server = new TrackingServer(node, handler)
        {
            PublicationError = new InvalidOperationException("publication failed")
        };
        using var client = CreateClient(node, server.Name);
        await client.WaitForServerAsync();
        using var goalBuffer = RosMessageBuffer.Create<SequenceActionGoal>();
        using var goal = await client.SendGoalAsync(goalBuffer);
        await handler.Started.Task;
        var buffer = RosMessageBuffer.Create<SequenceActionFeedback>();
        var inputReleased = 0;
        using var input = new RosMessageBuffer(buffer.Data, (_, _) =>
        {
            buffer.Dispose();
            Interlocked.Increment(ref inputReleased);
        });

        try
        {
            if (canceled)
            {
                using var cancellation = new CancellationTokenSource();
                cancellation.Cancel();
                Assert.Throws<OperationCanceledException>(() => handler.Controller!.ReportAsync(input, cancellation.Token));
            }
            else
            {
                await Assert.ThrowsAsync<InvalidOperationException>(() => handler.Controller!.ReportAsync(input).AsTask());
            }

            Assert.Equal(0, inputReleased);
            Assert.Equal(canceled ? 0 : 1, server.FeedbacksReleased);
        }
        finally
        {
            server.Dispose();
            handler.Resume.TrySetResult();
        }

        await handler.Completed.Task;
        await LifecycleAssert.EventuallyAsync(() => server.ResultsReleased == 1 && server.FeedbacksReleased == (canceled ? 1 : 2));
    }

    [Fact]
    public async Task UncachedResultWaitsForAllActiveReadersBeforeRelease()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = (RclNodeImpl)context.CreateNode(NameGenerator.GenerateNodeName());
        using var firstRead = new LifecycleCheckpoint();
        using var secondRead = new LifecycleCheckpoint();
        var handler = new ControlledHandler();
        using var server = new TrackingServer(node, handler, resultTimeout: TimeSpan.Zero)
        {
            FirstRead = firstRead,
            SecondRead = secondRead
        };
        using var client = CreateClient(node, server.Name);
        await client.WaitForServerAsync();
        using var goalBuffer = RosMessageBuffer.Create<SequenceActionGoal>();
        using var goal = await client.SendGoalAsync(goalBuffer);
        await handler.Started.Task;
        var first = goal.GetResultWithStatusAsync();
        var second = goal.GetResultWithStatusAsync();

        try
        {
            await server.ResultsRequested.Task;
            handler.Resume.TrySetResult();
            await Task.WhenAll(firstRead.Entered, secondRead.Entered);
            firstRead.Resume();
            var completed = await Task.WhenAny(first, second);
            Assert.True((await completed).IsSuccessful);
            Assert.Equal(0, server.ResultsReleased);
        }
        finally
        {
            handler.Resume.TrySetResult();
            firstRead.Resume();
            secondRead.Resume();
        }

        var results = await Task.WhenAll(first, second);

        foreach (var result in results)
        {
            using (result.Result)
            {
                Assert.True(result.IsSuccessful);
            }
        }

        await LifecycleAssert.EventuallyAsync(() => server.ResultsReleased == 1);
    }

    [Fact]
    public async Task ExpiredResultReleasesBuffersAndReturnsUnknown()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = (RclNodeImpl)context.CreateNode(NameGenerator.GenerateNodeName());
        var handler = new ControlledHandler();
        using var server = new TrackingServer(node, handler, resultTimeout: TimeSpan.FromMilliseconds(1));
        using var client = CreateClient(node, server.Name);
        await client.WaitForServerAsync();
        using var goalBuffer = RosMessageBuffer.Create<SequenceActionGoal>();
        using var goal = await client.SendGoalAsync(goalBuffer);
        await handler.Started.Task;
        handler.Resume.TrySetResult();
        await handler.Completed.Task;
        await LifecycleAssert.EventuallyAsync(() => server.ResultsReleased == 1 && server.FeedbacksReleased == 1);

        var result = await goal.GetResultWithStatusAsync();

        using (result.Result)
        {
            Assert.Equal(ActionGoalStatus.Unknown, result.Status);
        }
    }

    [Theory]
    [InlineData(3)]
    [InlineData(17)]
    public async Task StatusBroadcastContainsExactlyTheAcceptedGoals(int goalCount)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = (RclNodeImpl)context.CreateNode(NameGenerator.GenerateNodeName());
        var handler = new ControlledHandler();
        using var server = new TrackingServer(node, handler);
        using var subscription = node.CreateSubscription<GoalStatusArray>(server.Name + Constants.StatusTopic,
            new(qos: QosProfile.ActionStatusDefault));
        using var client = CreateClient(node, server.Name);
        await client.WaitForServerAsync();
        using var goalBuffer = RosMessageBuffer.Create<SequenceActionGoal>();
        await using var statuses = subscription.ReadAllAsync().GetAsyncEnumerator();
        var goals = new List<INativeActionGoalContext>();

        try
        {
            for (var i = 0; i < goalCount; i++)
            {
                goals.Add(await client.SendGoalAsync(goalBuffer));
            }

            while (await statuses.MoveNextAsync())
            {
                var status = statuses.Current;
                Assert.InRange(status.StatusList.Length, 1, goalCount);

                if (status.StatusList.Length == goalCount
                    && status.StatusList.All(x => x.Status == (sbyte)ActionGoalStatus.Executing))
                {
                    Assert.Equal(goals.Select(x => x.GoalId).Order(),
                        status.StatusList.Select(x => new Guid(x.GoalInfo.GoalId.Uuid)).Order());
                    return;
                }
            }

            Assert.Fail("The status subscription completed before all accepted goals were reported.");
        }
        finally
        {
            server.Dispose();
            handler.Resume.TrySetResult();

            foreach (var goal in goals)
            {
                goal.Dispose();
            }

            await LifecycleAssert.EventuallyAsync(() => server.ResultsReleased == goals.Count
                && server.FeedbacksReleased == goals.Count);
        }
    }

    private static IActionClient<SequenceActionGoal, SequenceActionResult, SequenceActionFeedback>
        CreateClient(IRclNode node, string name)
    {
        return node.CreateActionClient<SequenceAction, SequenceActionGoal,
            SequenceActionResult, SequenceActionFeedback>(name);
    }

    private static async Task InvokeAdmissionAsync(RclContext context, ActionServer server,
        RosMessageBuffer request, RosMessageBuffer response)
    {
        await context.Yield();
        // Isolate admission from the service transport, which is also closed by server.Dispose().
        typeof(ActionServer).GetMethod("HandleSendGoal", BindingFlags.Instance | BindingFlags.NonPublic)!
            .CreateDelegate<Action<RosMessageBuffer, RosMessageBuffer>>(server)(request, response);
    }

    private sealed class AdmissionHandler : ActionGoalHandler
    {
        public Action? CheckAdmission { get; set; }
        public Exception? AcceptanceError { get; init; }
        public List<string> Notifications { get; } = new();
        public int Executions { get; private set; }

        public override bool CanAccept(Guid id, RosMessageBuffer goal)
        {
            CheckAdmission?.Invoke();
            return true;
        }

        public override void OnAccepted(INativeActionGoalController controller)
        {
            Notifications.Add("accepted");

            if (AcceptanceError != null)
            {
                throw AcceptanceError;
            }
        }

        public override void OnCompleted(INativeActionGoalController controller)
        {
            Notifications.Add("completed");
        }

        public override Task ExecuteAsync(INativeActionGoalController controller, RosMessageBuffer goal,
            RosMessageBuffer result, CancellationToken cancellationToken)
        {
            Executions++;
            return Task.CompletedTask;
        }
    }

    private sealed class ControlledHandler : ActionGoalHandler
    {
        public TaskCompletionSource Started { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);
        public TaskCompletionSource Resume { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);
        public TaskCompletionSource Completed { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);
        public INativeActionGoalController? Controller { get; private set; }
        public CancellationToken CancellationToken { get; private set; }
        public Action? Accepted { get; init; }
        public bool AccessedBuffersAfterResume { get; private set; }
        public Func<bool>? BuffersAreLive { get; set; }

        public override void OnAccepted(INativeActionGoalController controller)
        {
            Accepted?.Invoke();
        }

        public override async Task ExecuteAsync(INativeActionGoalController controller, RosMessageBuffer goal,
            RosMessageBuffer result, CancellationToken cancellationToken)
        {
            Controller = controller;
            CancellationToken = cancellationToken;
            Started.TrySetResult();
            await Resume.Task.ConfigureAwait(false);

            if (BuffersAreLive?.Invoke() == false)
            {
                return;
            }

            SequenceActionGoal.CreateFrom(goal.Data, Encoding.UTF8);
            new SequenceActionResult().WriteTo(result.Data, Encoding.UTF8);
            AccessedBuffersAfterResume = true;
        }

        public override void OnCompleted(INativeActionGoalController controller)
        {
            Completed.TrySetResult();
        }
    }

    private sealed class CompletionHandler(LifecycleCheckpoint firstCompletion) : ActionGoalHandler
    {
        private int _executionsStarted, _callbacksStarted, _callbacksCompleted;

        public TaskCompletionSource FirstExecution { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);
        public TaskCompletionSource SecondExecution { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);
        public TaskCompletionSource Started { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);
        public TaskCompletionSource SecondCompletion { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);
        public TaskCompletionSource Completed { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);

        public override Task ExecuteAsync(INativeActionGoalController controller, RosMessageBuffer goal,
            RosMessageBuffer result, CancellationToken cancellationToken)
        {
            if (Interlocked.Increment(ref _executionsStarted) == 1)
            {
                return FirstExecution.Task;
            }

            Started.TrySetResult();
            return SecondExecution.Task;
        }

        public override void OnCompleted(INativeActionGoalController controller)
        {
            if (Interlocked.Increment(ref _callbacksStarted) == 1)
            {
                firstCompletion.Pause();
            }
            else
            {
                SecondCompletion.TrySetResult();
            }

            if (Interlocked.Increment(ref _callbacksCompleted) == 2)
            {
                Completed.TrySetResult();
            }
        }
    }

    private sealed class TrackingServer : ActionServer
    {
        private readonly bool _holdPublications;
        private readonly ActionIntrospection _introspection = new(SequenceAction.GetTypeSupportHandle());
        private int _goalsReleased, _resultsReleased, _feedbacksReleased, _resultReads, _resultRequests;
        private readonly ConcurrentDictionary<nint, byte> _releasedFeedbacks = new();

        public TrackingServer(RclNodeImpl node, INativeActionGoalHandler handler, bool holdPublications = false,
            TimeSpan? resultTimeout = null)
            : base(node, NameGenerator.GenerateActionName(), SequenceAction.TypeSupportName,
                SequenceAction.GetTypeSupportHandle(), handler,
                new ActionServerOptions(resultTimeout: resultTimeout ?? Timeout.InfiniteTimeSpan))
        {
            _holdPublications = holdPublications;
        }

        public int GoalsReleased => Volatile.Read(ref _goalsReleased);
        public int ResultsReleased => Volatile.Read(ref _resultsReleased);
        public int FeedbacksReleased => Volatile.Read(ref _feedbacksReleased);
        public ConcurrentQueue<nint> PublicationBuffers { get; } = new();
        public ConcurrentQueue<byte> PublishedValues { get; } = new();
        public TaskCompletionSource ResumePublications { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);
        public TaskCompletionSource ResultsRequested { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);
        public LifecycleCheckpoint? FirstRead { get; init; }
        public LifecycleCheckpoint? SecondRead { get; init; }
        public Exception? PublicationError { get; init; }

        protected override RosMessageBuffer CreateGoalBuffer()
        {
            var buffer = base.CreateGoalBuffer();
            return new RosMessageBuffer(buffer.Data, (_, _) =>
            {
                buffer.Dispose();
                Interlocked.Increment(ref _goalsReleased);
            });
        }

        protected override RosMessageBuffer CreateResultBuffer()
        {
            var buffer = base.CreateResultBuffer();
            return new RosMessageBuffer(buffer.Data, (_, _) =>
            {
                buffer.Dispose();
                Interlocked.Increment(ref _resultsReleased);
            });
        }

        protected override RosMessageBuffer CreateFeedbackBuffer()
        {
            var buffer = base.CreateFeedbackBuffer();
            _releasedFeedbacks.TryRemove(buffer.Data, out _);
            return new RosMessageBuffer(buffer.Data, (_, _) =>
            {
                _releasedFeedbacks.TryAdd(buffer.Data, 0);
                buffer.Dispose();
                Interlocked.Increment(ref _feedbacksReleased);
            });
        }

        protected override async ValueTask PublishFeedbackAsync(RosMessageBuffer buffer)
        {
            if (PublicationError != null)
            {
                throw PublicationError;
            }

            if (!_holdPublications)
            {
                await base.PublishFeedbackAsync(buffer).ConfigureAwait(false);
                return;
            }

            PublicationBuffers.Enqueue(buffer.Data);
            await ResumePublications.Task.ConfigureAwait(false);
            Assert.False(_releasedFeedbacks.ContainsKey(buffer.Data));
            var feedback = (SequenceActionFeedback)SequenceActionFeedback.CreateFrom(
                _introspection.FeedbackMessage.GetMemberPointer(buffer.Data, 1), Encoding.UTF8);
            PublishedValues.Enqueue(feedback.FeedbackValues[0].Value.Data[0]);
        }

        protected override void CopyResult(RosMessageBuffer source, RosMessageBuffer response)
        {
            var read = Interlocked.Increment(ref _resultReads);

            if (read == 1)
            {
                FirstRead?.Pause();
            }
            else if (read == 2)
            {
                SecondRead?.Pause();
            }

            base.CopyResult(source, response);
        }

        protected override Task WaitForResultAsync(Task completion, CancellationToken cancellationToken)
        {
            if (Interlocked.Increment(ref _resultRequests) == 2)
            {
                ResultsRequested.TrySetResult();
            }

            return base.WaitForResultAsync(completion, cancellationToken);
        }
    }
}
