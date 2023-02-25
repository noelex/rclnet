using Rcl.Internal.Services;
using Rcl.Introspection;
using Rcl.Logging;
using Rcl.Qos;
using Rosidl.Messages.Action;
using Rosidl.Messages.UniqueIdentifier;
using Rosidl.Runtime;
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
    private readonly DynamicFunctionTable _functions;

    private readonly Dictionary<Guid, GoalContext> _goals = new();
    private readonly CancellationTokenSource _shutdownSignal = new();

    private readonly RclClock _clock;
    private readonly TimeSpan _resultTimeout;
    private readonly IRclLogger _logger;

    public ActionServer(RclNodeImpl node, string actionName,
        string typesupportName, TypeSupportHandle actionTypesupport,
        INativeActionGoalHandler handler, Encoding textEncoding,
        TimeSpan resultTimeout)
    {
        _node = node;
        _clock = node.Clock;
        _handler = handler;
        _textEncoding = textEncoding;
        _resultTimeout = resultTimeout;
        _logger = _node.Context.DefaultLogger;

        var statusTopicName = actionName + Constants.StatusTopic;
        var feedbackTopicName = actionName + Constants.FeedbackTopic;
        var sendGoalServiceName = actionName + Constants.SendGoalService;
        var cancelGoalServiceName = actionName + Constants.CancelGoalService;
        var getResultServiceName = actionName + Constants.GetResultService;

        _typesupport = new ActionIntrospection(actionTypesupport);
        _functions = new DynamicFunctionTable(typesupportName);

        var done = false;
        try
        {
            _sendGoalService = new IntrospectionService(node,
                sendGoalServiceName, _typesupport.GoalServiceTypeSupport,
                new DelegateNativeServiceCallHandler(static (request, response, state) =>
                ((ActionServer)state!).HandleSendGoal(request, response), this), QosProfile.ServicesDefault);

            _getResultService = new ConcurrentIntrospectionService(node,
                getResultServiceName,
                new DelegateConcurrentNativeServiceCallHandler((request, response, state, ct) =>
                   ((ActionServer)state!).HandleGetResult(request, response, ct), this),
                _typesupport.ResultServiceTypeSupport,
                QosProfile.ServicesDefault);

            _cancelGoalService = _node.CreateNativeService<CancelGoalService>(
                cancelGoalServiceName, static (request, response, state) =>
                ((ActionServer)state!).HandleCancelGoal(request, response), QosProfile.ServicesDefault, this);

            _statusPublisher = _node.CreatePublisher<GoalStatusArray>(statusTopicName, new(qos: QosProfile.ActionStatusDefault, textEncoding: textEncoding));
            _feedbackPublisher = new RclNativePublisher(node, feedbackTopicName, _typesupport.FeedbackMessageTypeSupport, new(qos: QosProfile.SensorData));

            // In case the given action name gets normalized.
            var sep = _feedbackPublisher.Name!.LastIndexOf(Constants.FeedbackTopic);
            Name = _feedbackPublisher.Name.Substring(0, sep);
            done = true;

            if (_resultTimeout > TimeSpan.Zero)
            {
                _ = ExpireResultsAsync(_shutdownSignal.Token);
            }
            else
            {
                _logger.LogDebug($"Result expiration for action server '{Name}' is disabled because result timeout is set to {_resultTimeout}.");
            }
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
            await timer.WaitOneAsync(cancellationToken).ConfigureAwait(false);

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

        Guid goalId = _typesupport.GoalService.Request.AsRef<UUID.Priv>(request.Data, 0);
        var goal = _typesupport.GoalService.Request.GetMemberPointer(request.Data, 1);

        if (!_goals.ContainsKey(goalId) && _handler.CanAccept(goalId, new RosMessageBuffer(goal, static (a, b) => { })))
        {
            // Make a copy of the goal because we don't own the request buffer.
            var copiedGoal = new RosMessageBuffer(
                _functions.CreateGoal(),
                (p, tab) => ((DynamicFunctionTable)tab!).DestroyGoal(p),
                _functions);

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

            ref var resp = ref response.AsRef<SendGoalResponse>();
            resp.Accepted = true;
            resp.Stamp.CopyFrom(ctx.CreationTime);

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

            if (!_node.Context.IsCurrent)
            {
                await _node.Context.Yield();
            }

            NotifyStatusChange();
            _handler.OnCompleted(context);

            context.Complete();
        }
    }

    private async Task HandleGetResult(RosMessageBuffer request, RosMessageBuffer response, CancellationToken cancellationToken)
    {
        if (_goals.TryGetValue(request.AsRef<GetResultRequest>().GoalId, out var ctx))
        {
            await ctx.Completion.WaitAsync(cancellationToken);

            _typesupport.ResultService.Response.AsRef<ActionGoalStatus>(response.Data, 0) = ctx.Status;
            if (ctx.Status == ActionGoalStatus.Succeeded)
            {
                unsafe
                {
                    _functions.CopyResult(ctx.ResultBuffer.Data,
                        _typesupport.ResultService.Response.GetMemberPointer(response.Data, 1));
                }
            }

            // If the timeout is configured to have value -1,
            // then goal results will be “kept forever” (until the action server shuts down).
            //
            // If the timeout is configured to have value 0,
            // then goal results are discarded immediately (after responding to any pending result requests).
            if (_resultTimeout == TimeSpan.Zero)
            {
                if (!_node.Context.IsCurrent) await _node.Context.Yield();
                _goals.Remove(ctx.GoalId);
                ctx.Dispose();
            }
        }
        else
        {
            _typesupport.ResultService.Response.AsRef<ActionGoalStatus>(response.Data, 0) = ActionGoalStatus.Unknown;
        }
    }

    private void HandleCancelGoal(RosMessageBuffer request, RosMessageBuffer response)
    {
        ref var req = ref request.AsRef<CancelGoalServiceRequest.Priv>();
        ref var res = ref response.AsRef<CancelGoalServiceResponse.Priv>();

        Guid goalId = req.GoalInfo.GoalId;
        if (goalId == Guid.Empty && req.GoalInfo.Stamp.Sec == 0 && req.GoalInfo.Stamp.Nanosec == 0)
        {
            var cancellableGoals = _goals.Values.Where(x => !x.Completion.IsCompleted).ToArray();
            CancelGoals(cancellableGoals, ref res);
        }
        else if (goalId == Guid.Empty)
        {
            TimeSpan stamp = req.GoalInfo.Stamp;
            var cancellableGoals = _goals.Values.Where(x => !x.Completion.IsCompleted && x.CreationTime <= stamp).ToArray();
            CancelGoals(cancellableGoals, ref res);
        }
        else if (goalId != Guid.Empty)
        {
            if (!_goals.TryGetValue(goalId, out var ctx))
            {
                _logger.LogWarning($"Unable to cancel goal [{goalId}]: Goal not found.");
                res.ReturnCode = CancelGoalServiceResponse.ERROR_UNKNOWN_GOAL_ID;
                return;
            }

            if (ctx.Completion.IsCompleted)
            {
                _logger.LogWarning($"Unable to cancel goal [{goalId}]: Goal is in terminal state.");
                res.ReturnCode = CancelGoalServiceResponse.ERROR_GOAL_TERMINATED;
                return;
            }

            CancelGoals(new[] { ctx }, ref res);
        }
        else
        {
            TimeSpan stamp = req.GoalInfo.Stamp;
            var cancellableGoals = _goals.Values
                .Where(x => !x.Completion.IsCompleted && (goalId == x.GoalId || x.CreationTime <= stamp))
                .ToArray();

            CancelGoals(cancellableGoals, ref res);
        }
    }

    private void CancelGoals(GoalContext[] cancellableGoals, ref CancelGoalServiceResponse.Priv res)
    {
        if (cancellableGoals.Length == 0)
        {
            _logger.LogWarning($"Unable to cancel goal: No matching goal found.");
            res.ReturnCode = CancelGoalServiceResponse.ERROR_REJECTED;
            return;
        }

        Span<GoalInfo.Priv> span = stackalloc GoalInfo.Priv[cancellableGoals.Length];

        var i = 0;
        foreach (var goal in _goals.Values)
        {
            goal.Status = ActionGoalStatus.Canceling;

            span[i].GoalId = goal.GoalId;
            span[i].Stamp.CopyFrom(goal.CreationTime);
            i++;
        }

        NotifyStatusChange();

        res.ReturnCode = CancelGoalServiceResponse.ERROR_NONE;
        res.GoalsCanceling.CopyFrom(span);

        _node.Context.SynchronizationContext.Post((state) =>
        {
            foreach (var goal in (GoalContext[])state!)
            {
                goal.Cancel();
            }
        }, cancellableGoals);
    }

    private void NotifyStatusChange()
    {
        using var statusBuffer = RosMessageBuffer.Create<GoalStatusArray>();
        ref var statusArray = ref statusBuffer.AsRef<GoalStatusArray.Priv>();

        Span<GoalStatus.Priv> goals = stackalloc GoalStatus.Priv[_goals.Count];

        var i = 0;
        foreach (var goal in _goals.Values)
        {
            goals[i].GoalInfo.GoalId.CopyFrom(goal.GoalId);
            goals[i].GoalInfo.Stamp.CopyFrom(goal.CreationTime);
            goals[i].Status = (sbyte)goal.Status;

            i++;
        }
        statusArray.StatusList.CopyFrom(goals);

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
        private readonly TaskCompletionSource _completion = new();

        public GoalContext(Guid id, ActionServer server, TimeSpan accepted)
        {
            _goalId = id;
            _server = server;
            CreationTime = accepted;

            _feedbackMessageBuffer = _server._typesupport.FeedbackMessage.CreateBuffer();
            _server._typesupport.FeedbackMessage.AsRef<UUID.Priv>(_feedbackMessageBuffer.Data, 0).CopyFrom(id);

            _resultBuffer = _server._typesupport.ResultService.Response.CreateBuffer();

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

        public unsafe void Report(RosMessageBuffer value)
        {
            _server._functions.CopyFeedback(value.Data,
                _server._typesupport.FeedbackMessage.GetMemberPointer(_feedbackMessageBuffer.Data, 1));
            _server._feedbackPublisher.Publish(_feedbackMessageBuffer);
        }
    }
}