using Rcl.Actions;
using Rcl.Actions.Client;
using Rcl.Internal;
using Rcl.Internal.Publishers;
using Rcl.Internal.Services;
using Rcl.Introspection;
using Rcl.Logging;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Messages.Action;
using Rosidl.Messages.Tf2;
using Rosidl.Messages.UniqueIdentifier;
using Rosidl.Runtime;
using System.Collections.Concurrent;
using System.Reflection;

namespace Rcl.NET.Tests;

public class ActionGoalCancellationTests : IDisposable
{
    private readonly HandleReleaseError?[] _errorsBefore = HandleReleaseDiagnostics.Snapshot();

    public void Dispose()
    {
        Assert.Empty(HandleReleaseDiagnostics.Snapshot().Except(_errorsBefore));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task LateAcceptedExecutionIsCanceledOnAnActionServer(bool native)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var admission = new LifecycleCheckpoint();
        var handler = new DelayedAcceptanceHandler(admission);
        var name = NameGenerator.GenerateActionName();
        using var server = node.CreateActionServer<LookupTransformAction, LookupTransformActionGoal,
            LookupTransformActionResult, LookupTransformActionFeedback>(name, handler);
        using var client = node.CreateActionClient<LookupTransformAction, LookupTransformActionGoal,
            LookupTransformActionResult, LookupTransformActionFeedback>(name);
        await client.WaitForServerAsync(10_000);
        using var cancellation = new CancellationTokenSource();

        try
        {
            using var buffer = RosMessageBuffer.Create<LookupTransformActionGoal>();
            Task sending = native
                ? client.SendGoalAsync(buffer, cancellation.Token)
                : client.SendGoalAsync(new LookupTransformActionGoal(), cancellation.Token);
            await admission.Entered.WaitAsync(TimeSpan.FromSeconds(5));
            cancellation.Cancel();
            await Assert.ThrowsAnyAsync<OperationCanceledException>(
                () => sending.WaitAsync(TimeSpan.FromSeconds(2)));
        }
        finally
        {
            admission.Resume();
        }

        await handler.CancellationObserved.Task.WaitAsync(TimeSpan.FromSeconds(5));
    }

    [Theory]
    [InlineData(false, Timeout.Infinite)]
    [InlineData(true, Timeout.Infinite)]
    [InlineData(false, 5000)]
    [InlineData(true, 5000)]
    public async Task CancellationBeforeAcceptanceCancelsTheLateAcceptedGoal(bool native, int timeout)
    {
        await using var fixture = new Fixture();
        await fixture.Client.WaitForServerAsync(10_000);
        using var cancellation = new CancellationTokenSource();
        var sending = fixture.SendAsync(native, timeout, cancellation.Token);
        var id = await fixture.Server.Received.Task.WaitAsync(TimeSpan.FromSeconds(5));
        cancellation.Cancel();

        var error = await Assert.ThrowsAnyAsync<OperationCanceledException>(
            () => sending.WaitAsync(TimeSpan.FromSeconds(2)));

        Assert.Equal(cancellation.Token, error.CancellationToken);
        Assert.Equal(1, fixture.PendingCount);
        Assert.Empty(fixture.Server.CancelRequests);
        fixture.Server.Acceptance.TrySetResult(true);
        Assert.Equal(id, await fixture.Server.CancellationAccepted.Task.WaitAsync(TimeSpan.FromSeconds(5)));
        await fixture.DrainAsync();
        Assert.Empty(fixture.Logs.Errors);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task SendTimeoutStillCancelsTheLateAcceptedGoal(bool native)
    {
        await using var fixture = new Fixture();
        await fixture.Client.WaitForServerAsync(10_000);
        var sending = fixture.SendAsync(native, 500, default);
        var id = await fixture.Server.Received.Task.WaitAsync(TimeSpan.FromSeconds(5));
        await Assert.ThrowsAsync<TimeoutException>(() => sending.WaitAsync(TimeSpan.FromSeconds(3)));
        Assert.Empty(fixture.Server.CancelRequests);

        fixture.Server.Acceptance.TrySetResult(true);
        Assert.Equal(id, await fixture.Server.CancellationAccepted.Task.WaitAsync(TimeSpan.FromSeconds(5)));
        await fixture.DrainAsync();
        Assert.Empty(fixture.Logs.Errors);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task LateRejectionReleasesTheGoalWithoutCanceling(bool native)
    {
        await using var fixture = new Fixture();
        await fixture.CancelPendingSendAsync(native);
        fixture.Server.Acceptance.TrySetResult(false);
        await fixture.DrainAsync();
        Assert.Empty(fixture.Server.CancelRequests);
        Assert.Empty(fixture.Logs.Errors);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task PreCanceledSendNeverReachesTheServer(bool native)
    {
        await using var fixture = new Fixture();
        await fixture.Client.WaitForServerAsync(10_000);
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();
        await Assert.ThrowsAnyAsync<OperationCanceledException>(
            () => fixture.SendAsync(native, Timeout.Infinite, cancellation.Token));

        Assert.False(fixture.Server.Received.Task.IsCompleted);
        Assert.Equal(0, fixture.PendingCount);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task CancellationAfterHandoffDoesNotCancelTheGoal(bool native)
    {
        await using var fixture = new Fixture();
        fixture.Server.Acceptance.TrySetResult(true);
        await fixture.Client.WaitForServerAsync(10_000);
        using var cancellation = new CancellationTokenSource();
        using var goal = await fixture.SendAsync(native, 5000, cancellation.Token);
        Assert.Equal(0, fixture.PendingCount);
        cancellation.Cancel();

        // Place an explicit cancellation after the token was canceled: only this request should arrive.
        await goal.CancelAsync(5000);
        Assert.Equal(new[] { goal.GoalId }, fixture.Server.CancelRequests.ToArray());
        Assert.Empty(fixture.Logs.Errors);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task UnknownGoalAndUnrelatedCancelConfirmationAreRetried(bool unrelatedGoal)
    {
        await using var fixture = new Fixture();
        fixture.Server.FirstCancelIsUnknown = !unrelatedGoal;
        fixture.Server.FirstCancelHasUnrelatedGoal = unrelatedGoal;
        var id = await fixture.CancelPendingSendAsync(false);
        fixture.Server.Acceptance.TrySetResult(true);
        Assert.Equal(id, await fixture.Server.CancellationAccepted.Task.WaitAsync(TimeSpan.FromSeconds(5)));
        await fixture.DrainAsync();
        Assert.Equal(new[] { id, id }, fixture.Server.CancelRequests.ToArray());
        Assert.Empty(fixture.Logs.Errors);
    }

    [Fact]
    public async Task ExplicitCancelRejectionIsLoggedWithoutRetrying()
    {
        await using var fixture = new Fixture();
        fixture.Server.CancelCode = CancelGoalServiceResponse.ERROR_REJECTED;
        var id = await fixture.CancelPendingSendAsync(false);
        fixture.Server.Acceptance.TrySetResult(true);
        await fixture.DrainAsync();
        Assert.Equal(new[] { id }, fixture.Server.CancelRequests.ToArray());
        var error = Assert.Single(fixture.Logs.Errors);
        Assert.Contains(id.ToString(), error);
        Assert.Contains(fixture.Server.Name, error);
        Assert.Contains("caller cancellation", error);
    }

    [Fact]
    public async Task AlreadyTerminatedGoalNeedsNoFurtherConfirmation()
    {
        await using var fixture = new Fixture();
        fixture.Server.CancelCode = CancelGoalServiceResponse.ERROR_GOAL_TERMINATED;
        await fixture.CancelPendingSendAsync(false);
        fixture.Server.Acceptance.TrySetResult(true);
        await fixture.DrainAsync();
        Assert.Single(fixture.Server.CancelRequests);
        Assert.Empty(fixture.Logs.Errors);
    }

    [Fact]
    public async Task CancelRequestTimeoutIsRetriedWithinTheCleanupDeadline()
    {
        await using var fixture = new Fixture();
        fixture.Server.StallFirstCancellation = true;
        var id = await fixture.CancelPendingSendAsync(false);
        fixture.Server.Acceptance.TrySetResult(true);
        Assert.Equal(id, await fixture.Server.CancellationAccepted.Task.WaitAsync(TimeSpan.FromSeconds(6)));
        await fixture.DrainAsync();
        Assert.Equal(new[] { id, id }, fixture.Server.CancelRequests.ToArray());
        Assert.Empty(fixture.Logs.Errors);
    }

    [Fact]
    public async Task MissingAcceptanceExpiresAndReleasesThePendingRequest()
    {
        await using var fixture = new Fixture(TimeSpan.FromMilliseconds(300));
        var id = await fixture.CancelPendingSendAsync(false);
        await fixture.DrainAsync();
        Assert.Empty(fixture.Server.CancelRequests);
        var error = Assert.Single(fixture.Logs.Errors);
        Assert.Contains("deadline expired", error);
        Assert.Contains(id.ToString(), error);
    }

    [Fact]
    public async Task MissingCancelResponseSharesTheOverallCleanupDeadline()
    {
        await using var fixture = new Fixture(TimeSpan.FromSeconds(1));
        fixture.Server.StallCancellation = true;
        await fixture.CancelPendingSendAsync(false);
        fixture.Server.Acceptance.TrySetResult(true);
        await fixture.Server.CancelReceived.Task.WaitAsync(TimeSpan.FromSeconds(5));
        await fixture.DrainAsync();
        Assert.Contains("deadline expired", Assert.Single(fixture.Logs.Errors));
    }

    [Fact]
    public async Task ClientDisposalAbortsCleanupWithoutWaitingForAcceptance()
    {
        await using var fixture = new Fixture();
        var id = await fixture.CancelPendingSendAsync(false);
        fixture.Client.Dispose();
        await fixture.DrainAsync();
        Assert.Empty(fixture.Server.CancelRequests);
        var error = Assert.Single(fixture.Logs.Errors);
        Assert.Contains("Client disposed", error);
        Assert.Contains(id.ToString(), error);
        await Assert.ThrowsAsync<ObjectDisposedException>(
            () => fixture.SendAsync(false, 1000, default));
    }

    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public async Task SendAfterNodeClosePreparesTimeoutBeforeSending(bool native, bool closeClock)
    {
        using var clock = new RclClock(RclClockType.Steady);
        await using var fixture = new Fixture(clockOverride: clock);
        await fixture.Client.WaitForServerAsync(10_000);
        fixture.CloseNode();

        if (closeClock)
        {
            clock.Dispose();
            await Assert.ThrowsAsync<ObjectDisposedException>(
                () => fixture.SendAsync(native, 1000, default));
            await Task.Delay(500);
            Assert.False(fixture.Server.Received.Task.IsCompleted);
        }
        else
        {
            fixture.Server.Acceptance.TrySetResult(true);
            using var goal = await fixture.SendAsync(native, 5000, default);
            Assert.Equal(goal.GoalId, await fixture.Server.Received.Task);
        }

        Assert.Equal(0, fixture.PendingCount);
        Assert.Empty(fixture.Server.CancelRequests);
        Assert.Empty(fixture.Logs.Errors);
    }

    private sealed class Fixture : IAsyncDisposable
    {
        private readonly RclContext _context;
        private readonly RclNodeImpl _node;

        public Fixture(TimeSpan? cleanupTimeout = null, RclClock? clockOverride = null)
        {
            Logs = new CaptureLogs();
            _context = new RclContext(TestConfig.DefaultContextArguments, Logs);
            _node = new RclNodeImpl(_context, NameGenerator.GenerateNodeName(), clockOverride: clockOverride);
            Server = new ControlledServer(_node);
            Client = new ActionClient<LookupTransformAction, LookupTransformActionGoal,
                LookupTransformActionResult, LookupTransformActionFeedback>(
                _node, Server.Name, ActionClientOptions.Default, cleanupTimeout);
        }

        public CaptureLogs Logs { get; }
        public ControlledServer Server { get; }
        public ActionClient<LookupTransformAction, LookupTransformActionGoal,
            LookupTransformActionResult, LookupTransformActionFeedback> Client { get; }

        public int PendingCount
        {
            get
            {
                var pending = Client.GetType().GetField("_pendingSends", BindingFlags.Instance | BindingFlags.NonPublic)!
                    .GetValue(Client)!;
                return (int)pending.GetType().GetProperty("Count")!.GetValue(pending)!;
            }
        }

        public void CloseNode()
        {
            _node.Dispose();
        }

        public async Task<IActionGoalContext> SendAsync(bool native, int timeout, CancellationToken cancellationToken)
        {
            if (native)
            {
                using var buffer = RosMessageBuffer.Create<LookupTransformActionGoal>();
                return await Client.SendGoalAsync(buffer, timeout, cancellationToken);
            }

            return await Client.SendGoalAsync(new LookupTransformActionGoal(), timeout, cancellationToken);
        }

        public async Task<Guid> CancelPendingSendAsync(bool native)
        {
            await Client.WaitForServerAsync(10_000);
            using var cancellation = new CancellationTokenSource();
            var sending = SendAsync(native, Timeout.Infinite, cancellation.Token);
            var id = await Server.Received.Task.WaitAsync(TimeSpan.FromSeconds(5));
            cancellation.Cancel();
            await Assert.ThrowsAnyAsync<OperationCanceledException>(
                () => sending.WaitAsync(TimeSpan.FromSeconds(2)));

            return id;
        }

        public async Task DrainAsync()
        {
            await LifecycleAssert.EventuallyAsync(() => PendingCount == 0).WaitAsync(TimeSpan.FromSeconds(5));
        }

        public async ValueTask DisposeAsync()
        {
            Client.Dispose();
            await DrainAsync();
            Server.Dispose();
            _node.Dispose();
            await _context.DisposeAsync();
        }
    }

    private sealed class ControlledServer : IDisposable
    {
        private readonly ActionIntrospection _typesupport = new(LookupTransformAction.GetTypeSupportHandle());
        private readonly IRclService _send, _cancel, _result;
        private readonly IRclPublisher _status, _feedback;

        public ControlledServer(RclNodeImpl node)
        {
            Name = "/" + NameGenerator.GenerateActionName();
            _send = new ConcurrentIntrospectionService(node, Name + Constants.SendGoalService,
                new DelegateConcurrentNativeServiceCallHandler(
                    (request, response, _, ct) => SendAsync(request, response, ct), null),
                _typesupport.GoalServiceTypeSupport, ServerOptions.Default);

            _cancel = node.CreateConcurrentService<CancelGoalService, CancelGoalServiceRequest, CancelGoalServiceResponse>(
                Name + Constants.CancelGoalService, (request, _, ct) => CancelAsync(request, ct));

            _result = new IntrospectionService(node, Name + Constants.GetResultService,
                _typesupport.ResultServiceTypeSupport, new DelegateNativeServiceCallHandler((_, _, _) =>
                {
                }, null),
                ServerOptions.Default);

            _status = node.CreatePublisher<GoalStatusArray>(Name + Constants.StatusTopic,
                new(qos: QosProfile.ActionStatusDefault));
            _feedback = new RclNativePublisher(node, Name + Constants.FeedbackTopic,
                _typesupport.FeedbackMessageTypeSupport, new(qos: QosProfile.Default));
        }

        public string Name { get; }
        public TaskCompletionSource<Guid> Received { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);
        public TaskCompletionSource<bool> Acceptance { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);
        public TaskCompletionSource<Guid> CancellationAccepted { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);
        public TaskCompletionSource CancelReceived { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);
        public ConcurrentQueue<Guid> CancelRequests { get; } = new();
        public sbyte CancelCode { get; set; } = CancelGoalServiceResponse.ERROR_NONE;
        public bool FirstCancelIsUnknown { get; set; }
        public bool FirstCancelHasUnrelatedGoal { get; set; }
        public bool StallCancellation { get; set; }
        public bool StallFirstCancellation { get; set; }

        private async Task SendAsync(RosMessageBuffer request, RosMessageBuffer response, CancellationToken ct)
        {
            var id = RosidlRuntime.NativeAbi == RosidlNativeAbi.V1
                ? _typesupport.GoalService.Request.AsRef<UUID.Priv>(request.Data, 0).ToGuid()
                : _typesupport.GoalService.Request.AsRef<UUID.PrivV2>(request.Data, 0).ToGuid();
            Received.TrySetResult(id);
            var accepted = await Acceptance.Task.WaitAsync(ct).ConfigureAwait(false);
            if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
            {
                response.AsRef<SendGoalResponse>().Accepted = accepted;
            }
            else
            {
                response.AsRef<SendGoalResponseV2>().Accepted = accepted;
            }
        }

        private async Task<CancelGoalServiceResponse> CancelAsync(CancelGoalServiceRequest request, CancellationToken ct)
        {
            var id = new Guid(request.GoalInfo.GoalId.Uuid);
            CancelRequests.Enqueue(id);
            CancelReceived.TrySetResult();
            if (StallCancellation || StallFirstCancellation && CancelRequests.Count == 1)
            {
                await Task.Delay(Timeout.Infinite, ct).ConfigureAwait(false);
            }

            if (CancelRequests.Count == 1 && FirstCancelIsUnknown)
            {
                return new CancelGoalServiceResponse { ReturnCode = CancelGoalServiceResponse.ERROR_UNKNOWN_GOAL_ID };
            }

            if (CancelRequests.Count == 1 && FirstCancelHasUnrelatedGoal)
            {
                return new CancelGoalServiceResponse
                {
                    ReturnCode = CancelGoalServiceResponse.ERROR_NONE,
                    GoalsCanceling = new[] { new GoalInfo { GoalId = new UUID { Uuid = Guid.NewGuid().ToByteArray() } } }
                };
            }

            if (CancelCode == CancelGoalServiceResponse.ERROR_NONE)
            {
                CancellationAccepted.TrySetResult(id);
                return new CancelGoalServiceResponse
                {
                    ReturnCode = CancelCode,
                    GoalsCanceling = new[] { request.GoalInfo }
                };
            }

            return new CancelGoalServiceResponse { ReturnCode = CancelCode };
        }

        public void Dispose()
        {
            _send.Dispose();
            _cancel.Dispose();
            _result.Dispose();
            _status.Dispose();
            _feedback.Dispose();
        }
    }

    private sealed class DelayedAcceptanceHandler : ActionGoalHandler<LookupTransformActionGoal,
        LookupTransformActionResult, LookupTransformActionFeedback>
    {
        private readonly LifecycleCheckpoint _admission;

        public DelayedAcceptanceHandler(LifecycleCheckpoint admission)
        {
            _admission = admission;
        }

        public TaskCompletionSource CancellationObserved { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);

        public override bool CanAccept(Guid id, LookupTransformActionGoal goal)
        {
            _admission.Pause();
            return true;
        }

        public override async Task<LookupTransformActionResult> ExecuteAsync(
            IActionGoalController<LookupTransformActionFeedback> controller,
            LookupTransformActionGoal goal, CancellationToken cancellationToken)
        {
            try
            {
                await Task.Delay(Timeout.Infinite, cancellationToken).ConfigureAwait(false);
                return new LookupTransformActionResult();
            }
            catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
            {
                CancellationObserved.TrySetResult();
                throw;
            }
        }
    }

    private sealed class CaptureLogs : IRclLoggerFactory, IRclLogger
    {
        public string Name => "action_goal_cleanup_tests";
        public ConcurrentQueue<string> Errors { get; } = new();

        public IRclLogger CreateLogger(string name)
        {
            return this;
        }

        public void Log(LogSeverity severity, string? message, string file = "", string functionName = "", int lineNumber = 0)
        {
            if (severity == LogSeverity.Error && message != null)
            {
                Errors.Enqueue(message);
            }
        }
    }
}
