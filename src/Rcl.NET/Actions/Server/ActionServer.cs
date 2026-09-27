using Rcl.Internal.Publishers;
using Rcl.Internal.Services;
using Rcl.Introspection;
using Rcl.Logging;
using Rosidl.Messages.Action;
using Rosidl.Messages.UniqueIdentifier;
using Rosidl.Runtime;
using System.Diagnostics;
using System.Text;

namespace Rcl.Actions.Server;

internal class ActionServer : IActionServer
{
    private readonly RclNodeImpl _node;
    private readonly Encoding _textEncoding;
    private readonly ActionIntrospection _typesupport;

    private readonly IRclService _sendGoalService, _getResultService, _cancelGoalService;
    private readonly IRclPublisher _statusPublisher, _feedbackPublisher;

    private readonly INativeActionGoalHandler _handler;
    private readonly MessageBufferHelper _functions;

    private readonly Dictionary<Guid, GoalContext> _goals = new();
    private readonly CancellationTokenSource _shutdownSignal = new();

    private readonly RclClock _clock;
    private readonly TimeSpan _resultTimeout;
    private readonly IRclLogger _logger;

    public ActionServer(RclNodeImpl node, string actionName,
        string typesupportName, TypeSupportHandle actionTypesupport,
        INativeActionGoalHandler handler, ActionServerOptions options)
    {
        _node = node;
        _clock = node.Clock;
        _handler = handler;
        _textEncoding = options.TextEncoding;
        _resultTimeout = options.ResultTimeout;
        _logger = _node.Context.DefaultLogger;

        var statusTopicName = actionName + Constants.StatusTopic;
        var feedbackTopicName = actionName + Constants.FeedbackTopic;
        var sendGoalServiceName = actionName + Constants.SendGoalService;
        var cancelGoalServiceName = actionName + Constants.CancelGoalService;
        var getResultServiceName = actionName + Constants.GetResultService;

        _typesupport = new ActionIntrospection(actionTypesupport);
        _functions = new MessageBufferHelper(typesupportName);

        var done = false;

        try
        {
            _sendGoalService = new IntrospectionService(node,
                sendGoalServiceName, _typesupport.GoalServiceTypeSupport,
                new DelegateNativeServiceCallHandler(static (request, response, state) =>
                ((ActionServer)state!).HandleSendGoal(request, response), this), new(qos: options.GoalServiceQos));

            _getResultService = new ConcurrentIntrospectionService(node,
                getResultServiceName,
                new DelegateConcurrentNativeServiceCallHandler((request, response, state, ct) =>
                   ((ActionServer)state!).HandleGetResult(request, response, ct), this),
                _typesupport.ResultServiceTypeSupport,
                new(qos: options.ResultServiceQos));

            _cancelGoalService = _node.CreateNativeService<CancelGoalService>(
                cancelGoalServiceName, static (request, response, state) =>
                ((ActionServer)state!).HandleCancelGoal(request, response), this, new(qos: options.CancelServiceQos));

            _statusPublisher = _node.CreatePublisher<GoalStatusArray>(statusTopicName,
                new(qos: options.StatusTopicQos, textEncoding: options.TextEncoding));

            _feedbackPublisher = new RclNativePublisher(node, feedbackTopicName,
                _typesupport.FeedbackMessageTypeSupport, new(qos: options.FeedbackTopicQos));

            // In case the given action name gets normalized.
            var sep = _feedbackPublisher.Name!.LastIndexOf(Constants.FeedbackTopic);
            Name = _feedbackPublisher.Name.Substring(0, sep);

            if (_resultTimeout > TimeSpan.Zero)
            {
                _ = ExpireResultsAsync(_shutdownSignal.Token);
            }
            else
            {
                _logger.LogDebug($"Result expiration for action server '{Name}' is disabled because result timeout is set to {_resultTimeout}.");
            }

            done = true;
        }
        finally
        {
            if (!done)
            {
                _feedbackPublisher?.Dispose();
                _statusPublisher?.Dispose();
                _cancelGoalService?.Dispose();
                _getResultService?.Dispose();
                _sendGoalService?.Dispose();
                _shutdownSignal.Dispose();
            }
        }
    }

    private async Task ExpireResultsAsync(CancellationToken cancellationToken)
    {
        var candidates = new List<GoalContext>();
        using var timer = _node.Context.CreateTimer(_clock, TimeSpan.FromSeconds(1));

        while (!cancellationToken.IsCancellationRequested)
        {
            // Ensure we wake up on the event loop.
            await timer.WaitOneAsync(false, cancellationToken).ConfigureAwait(false);

            foreach (var goal in _goals.Values)
            {
                if (goal.Completion.IsCompleted && (_clock.Elapsed - goal.CompletionTime) >= _resultTimeout)
                {
                    candidates.Add(goal);
                }
            }

            foreach (var goal in candidates)
            {
                _goals.Remove(goal.GoalId);
                goal.Dispose();
            }

            candidates.Clear();
        }
    }

    public string Name { get; }

    private unsafe void HandleSendGoal(RosMessageBuffer request, RosMessageBuffer response)
    {
        // request & response buffers are owned by service server,
        // no need to dispose here.

        var goalId = RosidlRuntime.NativeAbi switch
        {
            RosidlNativeAbi.V1 => _typesupport.GoalService.Request.AsRef<UUID.Priv>(request.Data, 0).ToGuid(),
            RosidlNativeAbi.V2 => _typesupport.GoalService.Request.AsRef<UUID.PrivV2>(request.Data, 0).ToGuid(),
            _ => throw new UnreachableException(),
        };
        var goal = _typesupport.GoalService.Request.GetMemberPointer(request.Data, 1);

        if (!_goals.ContainsKey(goalId) && _handler.CanAccept(goalId, new RosMessageBuffer(goal, static (a, b) =>
{
})))
        {
            // Make a copy of the goal because we don't own the request buffer.
            var copiedGoal = _functions.CreateGoalBuffer();

            if (!_functions.CopyGoal(goal, copiedGoal.Data))
            {
                copiedGoal.Dispose();
                throw new RclException("Unable to copy goal buffer.");
            }

            var ctx = new GoalContext(goalId, this, _clock.Elapsed);
            _goals[goalId] = ctx;

            // Send response first, then notify status change.
            ctx.Status = ActionGoalStatus.Accepted;
            _node.Context.SynchronizationContext.Post(static (state) =>
                ((ActionServer)state!).NotifyStatusChange(), this);

            if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
            {
                ref var resp = ref response.AsRef<SendGoalResponse>();
                resp.Accepted = true;
                resp.Stamp.CopyFrom(ctx.CreationTime);
            }
            else
            {
                ref var resp = ref response.AsRef<SendGoalResponseV2>();
                resp.Accepted = true;
                resp.Stamp.CopyFrom(ctx.CreationTime);
            }

            _handler.OnAccepted(ctx);

            // Spawn a coroutine to execute the goal
            _ = ExecuteGoalAsync(ctx, copiedGoal);
        }
    }

    private async Task ExecuteGoalAsync(GoalContext context, RosMessageBuffer goalBuffer)
    {
        // Make sure the following happens asynchronously.
        await _node.Context.Yield();

        using (goalBuffer)
        {
            using var cts = CancellationTokenSource.CreateLinkedTokenSource(_shutdownSignal.Token, context.CancelSignal, context.AbortSignal);

            ActionGoalStatus status;

            try
            {
                context.Status = ActionGoalStatus.Executing;
                NotifyStatusChange();

                await _handler.ExecuteAsync(context, goalBuffer, context.ResultBuffer, cts.Token);
                status = ActionGoalStatus.Succeeded;
            }
            catch (OperationCanceledException)
            {
                if (context.CancelSignal.IsCancellationRequested)
                {
                    _logger.LogDebug($"Goal '{context.GoalId}' canceled due to client cancel request.");
                    status = ActionGoalStatus.Canceled;
                }
                else
                {
                    _logger.LogDebug($"Goal '{context.GoalId}' aborted due to server shutdown or preemption.");
                    status = ActionGoalStatus.Aborted;
                }
            }
            catch (Exception e)
            {
                _logger.LogWarning($"Goal '{context.GoalId}' aborted due to unhandled exception: {e.Message}");
                status = ActionGoalStatus.Aborted;
            }

            context.Status = status;
            context.CompletionTime = _clock.Elapsed;

            await _node.Context.YieldIfNotCurrent();

            NotifyStatusChange();
            _handler.OnCompleted(context);

            context.Complete();
        }
    }

    private async Task HandleGetResult(RosMessageBuffer request, RosMessageBuffer response, CancellationToken cancellationToken)
    {
        var goalId = RosidlRuntime.NativeAbi == RosidlNativeAbi.V1
            ? request.AsRef<GetResultRequest>().GoalId.ToGuid()
            : request.AsRef<GetResultRequestV2>().GoalId.ToGuid();

        if (_goals.TryGetValue(goalId, out var ctx))
        {
            await ctx.Completion.WaitAsync(cancellationToken).ConfigureAwait(false);

            // ActionGoalStatus maps directly to the ABI-independent int8 status member.
            _typesupport.ResultService.Response.UnsafeAsRef<ActionGoalStatus>(response.Data, 0) = ctx.Status;

            if (ctx.Status == ActionGoalStatus.Succeeded)
            {
                _functions.CopyResult(ctx.ResultBuffer.Data,
                    _typesupport.ResultService.Response.GetMemberPointer(response.Data, 1));
            }

            // If the timeout is configured to have value -1,
            // then goal results will be “kept forever” (until the action server shuts down).
            //
            // If the timeout is configured to have value 0,
            // then goal results are discarded immediately (after responding to any pending result requests).
            if (_resultTimeout == TimeSpan.Zero)
            {
                await _node.Context.YieldIfNotCurrent();

                using (ctx)
                {
                    _goals.Remove(ctx.GoalId);
                }
            }
        }
        else
        {
            // ActionGoalStatus maps directly to the ABI-independent int8 status member.
            _typesupport.ResultService.Response.UnsafeAsRef<ActionGoalStatus>(response.Data, 0) = ActionGoalStatus.Unknown;
        }
    }

    private void HandleCancelGoal(RosMessageBuffer request, RosMessageBuffer response)
    {
        var (goalId, stamp) = ReadCancelRequest(request);

        if (goalId == Guid.Empty && stamp == TimeSpan.Zero)
        {
            var cancellableGoals = _goals.Values.Where(x => !x.Completion.IsCompleted).ToArray();
            CancelGoals(cancellableGoals, response);
        }
        else if (goalId == Guid.Empty)
        {
            var cancellableGoals = _goals.Values.Where(x => !x.Completion.IsCompleted && x.CreationTime <= stamp).ToArray();
            CancelGoals(cancellableGoals, response);
        }
        else if (goalId != Guid.Empty)
        {
            if (!_goals.TryGetValue(goalId, out var ctx))
            {
                _logger.LogWarning($"Unable to cancel goal [{goalId}]: Goal not found.");
                WriteCancelResponse(response, CancelGoalServiceResponse.ERROR_UNKNOWN_GOAL_ID, Array.Empty<GoalContext>());
            }
            else if (ctx.Completion.IsCompleted)
            {
                _logger.LogWarning($"Unable to cancel goal [{goalId}]: Goal is in terminal state.");
                WriteCancelResponse(response, CancelGoalServiceResponse.ERROR_GOAL_TERMINATED, Array.Empty<GoalContext>());
            }
            else
            {
                CancelGoals(new[] { ctx }, response);
            }
        }
        else
        {
            var cancellableGoals = _goals.Values
                .Where(x => !x.Completion.IsCompleted && (goalId == x.GoalId || x.CreationTime <= stamp))
                .ToArray();

            CancelGoals(cancellableGoals, response);
        }
    }

    private static (Guid GoalId, TimeSpan Stamp) ReadCancelRequest(RosMessageBuffer request)
    {
        if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
        {
            ref var value = ref request.AsRef<CancelGoalServiceRequest.Priv>();
            return (value.GoalInfo.GoalId.ToGuid(), value.GoalInfo.Stamp);
        }

        ref var valueV2 = ref request.AsRef<CancelGoalServiceRequest.PrivV2>();
        return (valueV2.GoalInfo.GoalId.ToGuid(), valueV2.GoalInfo.Stamp);
    }

    private void CancelGoals(GoalContext[] cancellableGoals, RosMessageBuffer response)
    {
        if (cancellableGoals.Length == 0)
        {
            _logger.LogWarning($"Unable to cancel goal: No matching goal found.");
            WriteCancelResponse(response, CancelGoalServiceResponse.ERROR_REJECTED, cancellableGoals);
            return;
        }

        foreach (var goal in cancellableGoals)
        {
            goal.Status = ActionGoalStatus.Canceling;
        }

        NotifyStatusChange();
        WriteCancelResponse(response, CancelGoalServiceResponse.ERROR_NONE, cancellableGoals);

        _node.Context.SynchronizationContext.Post((state) =>
        {
            foreach (var goal in (GoalContext[])state!)
            {
                goal.Cancel();
            }
        }, cancellableGoals);
    }

    private static void WriteCancelResponse(RosMessageBuffer response, sbyte returnCode, GoalContext[] goals)
    {
        if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
        {
            ref var value = ref response.AsRef<CancelGoalServiceResponse.Priv>();
            value.ReturnCode = returnCode;

            if (goals.Length == 0)
            {
                return;
            }

            Span<GoalInfo.Priv> nativeGoals = stackalloc GoalInfo.Priv[goals.Length];

            for (var i = 0; i < goals.Length; i++)
            {
                nativeGoals[i].GoalId.CopyFrom(goals[i].GoalId);
                nativeGoals[i].Stamp.CopyFrom(goals[i].CreationTime);
            }

            value.GoalsCanceling.CopyFrom(nativeGoals);
            return;
        }

        ref var valueV2 = ref response.AsRef<CancelGoalServiceResponse.PrivV2>();
        valueV2.ReturnCode = returnCode;

        if (goals.Length == 0)
        {
            return;
        }

        Span<GoalInfo.PrivV2> nativeGoalsV2 = stackalloc GoalInfo.PrivV2[goals.Length];

        for (var i = 0; i < goals.Length; i++)
        {
            nativeGoalsV2[i].GoalId.CopyFrom(goals[i].GoalId);
            nativeGoalsV2[i].Stamp.CopyFrom(goals[i].CreationTime);
        }

        valueV2.GoalsCanceling.CopyFrom(nativeGoalsV2);
    }

    private void NotifyStatusChange()
    {
        using var statusBuffer = RosMessageBuffer.Create<GoalStatusArray>();

        if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
        {
            ref var statusArray = ref statusBuffer.AsRef<GoalStatusArray.Priv>();
            Span<GoalStatus.Priv> goals = stackalloc GoalStatus.Priv[_goals.Count];
            var index = 0;

            foreach (var goal in _goals.Values)
            {
                goals[index].GoalInfo.GoalId.CopyFrom(goal.GoalId);
                goals[index].GoalInfo.Stamp.CopyFrom(goal.CreationTime);
                goals[index].Status = (sbyte)goal.Status;
                index++;
            }

            statusArray.StatusList.CopyFrom(goals);
        }
        else
        {
            ref var statusArray = ref statusBuffer.AsRef<GoalStatusArray.PrivV2>();
            Span<GoalStatus.PrivV2> goals = stackalloc GoalStatus.PrivV2[_goals.Count];
            var index = 0;

            foreach (var goal in _goals.Values)
            {
                goals[index].GoalInfo.GoalId.CopyFrom(goal.GoalId);
                goals[index].GoalInfo.Stamp.CopyFrom(goal.CreationTime);
                goals[index].Status = (sbyte)goal.Status;
                index++;
            }

            statusArray.StatusList.CopyFrom(goals);
        }

        _statusPublisher.Publish(statusBuffer);
    }

    public void Dispose()
    {
        if (!_shutdownSignal.IsCancellationRequested)
        {
            _shutdownSignal.Cancel();
            _shutdownSignal.Dispose();

            _node.Context.SynchronizationContext.Send(static (state) =>
            {
                var self = (ActionServer)state!;

                foreach (var ctx in self._goals.Values)
                {
                    ctx.Dispose();
                }

                self._goals.Clear();
            }, this);

            _feedbackPublisher?.Dispose();
            _statusPublisher?.Dispose();
            _cancelGoalService?.Dispose();
            _getResultService?.Dispose();
            _sendGoalService?.Dispose();
        }
    }

    private class GoalContext : INativeActionGoalController, IDisposable
    {
        private readonly Guid _goalId;
        private readonly ActionServer _server;
        private readonly RosMessageBuffer _feedbackMessageBuffer, _resultBuffer;

        private readonly CancellationTokenSource _abort = new(), _cancel = new();
        private readonly TaskCompletionSource _completion = new(TaskCreationOptions.RunContinuationsAsynchronously);

        public unsafe GoalContext(Guid id, ActionServer server, TimeSpan accepted)
        {
            _goalId = id;
            _server = server;
            CreationTime = accepted;

            _feedbackMessageBuffer = _server._typesupport.FeedbackMessage.CreateBuffer();

            if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
            {
                _server._typesupport.FeedbackMessage.AsRef<UUID.Priv>(_feedbackMessageBuffer.Data, 0).CopyFrom(id);
            }
            else
            {
                _server._typesupport.FeedbackMessage.AsRef<UUID.PrivV2>(_feedbackMessageBuffer.Data, 0).CopyFrom(id);
            }

            _resultBuffer = _server._functions.CreateResultBuffer();

            _server._logger.LogDebug($"Created action goal context [{GoalId}].");
        }

        public Guid GoalId => _goalId;

        public TimeSpan CreationTime { get; }

        public TimeSpan CompletionTime { get; set; }

        public ActionGoalStatus Status { get; set; }

        public CancellationToken CancelSignal => _cancel.Token;

        public CancellationToken AbortSignal => _abort.Token;

        public RosMessageBuffer ResultBuffer => _resultBuffer;

        public Task Completion => _completion.Task;

        public void Abort()
        {
            if (!_abort.IsCancellationRequested)
            {
                _abort.Cancel();
            }
        }

        public void Cancel()
        {
            if (!_cancel.IsCancellationRequested)
            {
                _cancel.Cancel();
            }
        }

        public void Complete()
        {
            _completion.TrySetResult();
        }

        public void Dispose()
        {
            _feedbackMessageBuffer.Dispose();
            _resultBuffer.Dispose();

            _abort.Dispose();
            _cancel.Dispose();

            _server._logger.LogDebug($"Action goal context [{GoalId}] disposed.");
        }

        private void CopyFeedbackFrom(RosMessageBuffer src)
        {
            _server._functions.CopyFeedback(src.Data,
                _server._typesupport.FeedbackMessage.GetMemberPointer(_feedbackMessageBuffer.Data, 1));
        }

        public void Report(RosMessageBuffer value)
        {
            CopyFeedbackFrom(value);
            _server._feedbackPublisher.Publish(_feedbackMessageBuffer);
        }

        public ValueTask ReportAsync(RosMessageBuffer buffer, CancellationToken cancellationToken = default)
        {
            CopyFeedbackFrom(buffer);
            return _server._feedbackPublisher.PublishAsync(_feedbackMessageBuffer);
        }
    }
}
