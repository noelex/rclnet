using Rcl.Internal.Publishers;
using Rcl.Internal.Services;
using Rcl.Introspection;
using Rcl.Logging;
using Rosidl.Messages.Action;
using Rosidl.Messages.UniqueIdentifier;
using Rosidl.Runtime;
using System.Buffers;
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
    private readonly object _goalsGate = new();
    private readonly object _completionGate = new();
    private readonly CancellationTokenSource _shutdownSignal = new();
    private readonly CancellationToken _shutdownToken;
    private int _disposed;

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
        _shutdownToken = _shutdownSignal.Token;

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
                _ = ExpireResultsAsync(_shutdownToken);
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

        try
        {
            while (!cancellationToken.IsCancellationRequested)
            {
                // Ensure we wake up on the event loop.
                await timer.WaitOneAsync(false, cancellationToken).ConfigureAwait(false);

                var now = _clock.Elapsed;

                lock (_goalsGate)
                {
                    foreach (var goal in _goals.Values)
                    {
                        if (goal.Completion.IsCompleted && (now - goal.CompletionTime) >= _resultTimeout)
                        {
                            candidates.Add(goal);
                        }
                    }
                }

                foreach (var goal in candidates)
                {
                    RemoveGoal(goal);
                }

                candidates.Clear();
            }
        }
        catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
        {
        }
        catch (ObjectDisposedException) when (_node.Context.Handle.IsClosing)
        {
        }
    }

    public string Name { get; }

    protected virtual RosMessageBuffer CreateResultBuffer() => _functions.CreateResultBuffer();

    protected virtual RosMessageBuffer CreateFeedbackBuffer() => _typesupport.FeedbackMessage.CreateBuffer();

    protected virtual ValueTask PublishFeedbackAsync(RosMessageBuffer buffer) => _feedbackPublisher.PublishAsync(buffer);

    protected virtual Task WaitForResultAsync(Task completion, CancellationToken cancellationToken)
        => completion.WaitAsync(cancellationToken);

    protected virtual void CopyResult(RosMessageBuffer source, RosMessageBuffer response)
    {
        if (!_functions.CopyResult(source.Data, _typesupport.ResultService.Response.GetMemberPointer(response.Data, 1)))
        {
            throw new RclException("Unable to copy result buffer.");
        }
    }

    private void RemoveGoal(GoalContext goal)
    {
        bool removed;

        lock (_goalsGate)
        {
            removed = _goals.TryGetValue(goal.GoalId, out var current) && ReferenceEquals(current, goal)
                && _goals.Remove(goal.GoalId);
        }

        if (removed)
        {
            goal.Dispose();
        }
    }

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

        lock (_goalsGate)
        {
            if (_disposed != 0 || _goals.ContainsKey(goalId))
            {
                return;
            }
        }

        if (_handler.CanAccept(goalId, new RosMessageBuffer(goal, static (_, _) => { })))
        {
            // Make a copy of the goal because we don't own the request buffer.
            var copiedGoal = _functions.CreateGoalBuffer();

            if (!_functions.CopyGoal(goal, copiedGoal.Data))
            {
                copiedGoal.Dispose();
                throw new RclException("Unable to copy goal buffer.");
            }

            GoalContext? ctx = null;
            var executionStarted = false;

            try
            {
                ctx = new GoalContext(goalId, this, _clock.Elapsed);

                lock (_goalsGate)
                {
                    ObjectDisposedException.ThrowIf(_disposed != 0, this);
                    _goals[goalId] = ctx;
                }

                // Send response first, then notify status change.
                ctx.Status = ActionGoalStatus.Accepted;
                _node.Context.SynchronizationContext.Post(static state =>
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

                // The execution reference is reserved before any callback can close the server.
                _ = ExecuteGoalAsync(ctx, copiedGoal);
                executionStarted = true;
            }
            finally
            {
                if (!executionStarted)
                {
                    copiedGoal.Dispose();

                    if (ctx != null)
                    {
                        RemoveGoal(ctx);
                        ctx.Dispose();
                        NotifyGoalCompleted(ctx);
                        ctx.Release();
                    }
                }
            }
        }
    }

    private async Task ExecuteGoalAsync(GoalContext context, RosMessageBuffer goalBuffer)
    {
        try
        {
            using var ownedGoal = goalBuffer;
            // Make sure the following happens asynchronously, including when shutdown wins this yield.
            await _node.Context.Yield();

            using var cts = CancellationTokenSource.CreateLinkedTokenSource(_shutdownToken, context.CancelSignal, context.AbortSignal);

            ActionGoalStatus status;

            try
            {
                context.Status = ActionGoalStatus.Executing;
                NotifyStatusChange();

                cts.Token.ThrowIfCancellationRequested();
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
            if (Volatile.Read(ref _disposed) == 0 && !_node.Context.Handle.IsClosing)
            {
                context.CompletionTime = _clock.Elapsed;
            }

            await _node.Context.YieldIfNotCurrent();

            NotifyStatusChange();
        }
        finally
        {
            NotifyGoalCompleted(context);
            context.Complete();
            context.Release();
        }
    }

    private void NotifyGoalCompleted(GoalContext context)
    {
        // Context shutdown moves continuations to the thread pool, so yielding alone cannot serialize callbacks.
        lock (_completionGate)
        {
            Cleanup.Run(state => _handler.OnCompleted((GoalContext)state!), context);
        }
    }

    private async Task HandleGetResult(RosMessageBuffer request, RosMessageBuffer response, CancellationToken cancellationToken)
    {
        var goalId = RosidlRuntime.NativeAbi == RosidlNativeAbi.V1
            ? request.AsRef<GetResultRequest>().GoalId.ToGuid()
            : request.AsRef<GetResultRequestV2>().GoalId.ToGuid();

        GoalContext? ctx;

        lock (_goalsGate)
        {
            if (_goals.TryGetValue(goalId, out ctx) && !ctx.TryRetain())
            {
                ctx = null;
            }
        }

        if (ctx != null)
        {
            try
            {
                await WaitForResultAsync(ctx.Completion, cancellationToken).ConfigureAwait(false);

                // ActionGoalStatus maps directly to the ABI-independent int8 status member.
                _typesupport.ResultService.Response.UnsafeAsRef<ActionGoalStatus>(response.Data, 0) = ctx.Status;

                if (ctx.Status == ActionGoalStatus.Succeeded)
                {
                    CopyResult(ctx.ResultBuffer, response);
                }

                // Remove the cache's ownership without releasing buffers used by other result readers.
                if (_resultTimeout == TimeSpan.Zero)
                {
                    RemoveGoal(ctx);
                }
            }
            finally
            {
                ctx.Release();
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
            GoalContext[] cancellableGoals;

            lock (_goalsGate)
            {
                cancellableGoals = _goals.Values.Where(x => !x.Completion.IsCompleted).ToArray();
            }

            CancelGoals(cancellableGoals, response);
        }
        else if (goalId == Guid.Empty)
        {
            GoalContext[] cancellableGoals;

            lock (_goalsGate)
            {
                cancellableGoals = _goals.Values.Where(x => !x.Completion.IsCompleted && x.CreationTime <= stamp).ToArray();
            }

            CancelGoals(cancellableGoals, response);
        }
        else if (goalId != Guid.Empty)
        {
            GoalContext? ctx;

            lock (_goalsGate)
            {
                _goals.TryGetValue(goalId, out ctx);
            }

            if (ctx == null)
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
            GoalContext[] cancellableGoals;

            lock (_goalsGate)
            {
                cancellableGoals = _goals.Values
                    .Where(x => !x.Completion.IsCompleted && (goalId == x.GoalId || x.CreationTime <= stamp))
                    .ToArray();
            }

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
        if (Volatile.Read(ref _disposed) != 0 || _node.Context.Handle.IsClosing)
        {
            return;
        }

        using var statusBuffer = RosMessageBuffer.Create<GoalStatusArray>();
        GoalContext[] contexts;
        int count;

        lock (_goalsGate)
        {
            if (_disposed != 0)
            {
                return;
            }

            count = _goals.Count;
            contexts = ArrayPool<GoalContext>.Shared.Rent(count);
            _goals.Values.CopyTo(contexts, 0);
        }

        try
        {
            if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
            {
                ref var statusArray = ref statusBuffer.AsRef<GoalStatusArray.Priv>();
                Span<GoalStatus.Priv> goals = stackalloc GoalStatus.Priv[count];

                for (var i = 0; i < count; i++)
                {
                    var goal = contexts[i];
                    goals[i].GoalInfo.GoalId.CopyFrom(goal.GoalId);
                    goals[i].GoalInfo.Stamp.CopyFrom(goal.CreationTime);
                    goals[i].Status = (sbyte)goal.Status;
                }

                statusArray.StatusList.CopyFrom(goals);
            }
            else
            {
                ref var statusArray = ref statusBuffer.AsRef<GoalStatusArray.PrivV2>();
                Span<GoalStatus.PrivV2> goals = stackalloc GoalStatus.PrivV2[count];

                for (var i = 0; i < count; i++)
                {
                    var goal = contexts[i];
                    goals[i].GoalInfo.GoalId.CopyFrom(goal.GoalId);
                    goals[i].GoalInfo.Stamp.CopyFrom(goal.CreationTime);
                    goals[i].Status = (sbyte)goal.Status;
                }

                statusArray.StatusList.CopyFrom(goals);
            }
        }
        finally
        {
            // Snapshots only read managed metadata; clear references before returning the array to the pool.
            ArrayPool<GoalContext>.Shared.Return(contexts, clearArray: true);
        }

        try
        {
            _statusPublisher.Publish(statusBuffer);
        }
        catch (ObjectDisposedException) when (Volatile.Read(ref _disposed) != 0 || _node.Context.Handle.IsClosing)
        {
        }
    }

    public void Dispose()
    {
        GoalContext[] goals;

        lock (_goalsGate)
        {
            if (_disposed != 0)
            {
                return;
            }

            Volatile.Write(ref _disposed, 1);
            goals = _goals.Values.ToArray();
            _goals.Clear();
        }

        Cleanup.Run(static state => ((CancellationTokenSource)state!).Cancel(), _shutdownSignal);

        foreach (var goal in goals)
        {
            goal.Dispose();
        }

        Cleanup.Dispose(_feedbackPublisher);
        Cleanup.Dispose(_statusPublisher);
        Cleanup.Dispose(_cancelGoalService);
        Cleanup.Dispose(_getResultService);
        Cleanup.Dispose(_sendGoalService);
        _shutdownSignal.Dispose();
    }

    private class GoalContext : INativeActionGoalController, IDisposable
    {
        private readonly Guid _goalId;
        private readonly ActionServer _server;
        private readonly RosMessageBuffer _feedbackMessageBuffer, _resultBuffer;
        private readonly object _lifetimeGate = new(), _feedbackGate = new();
        // The cache and the reserved execution each own a reference from construction onward.
        private int _references = 2;
        private bool _disposeRequested;

        private readonly CancellationTokenSource _abort = new(), _cancel = new();
        private readonly TaskCompletionSource _completion = new(TaskCreationOptions.RunContinuationsAsynchronously);

        public unsafe GoalContext(Guid id, ActionServer server, TimeSpan accepted)
        {
            _goalId = id;
            _server = server;
            CreationTime = accepted;

            _feedbackMessageBuffer = CreateFeedbackMessage();

            try
            {
                _resultBuffer = _server.CreateResultBuffer();
            }
            catch
            {
                _feedbackMessageBuffer.Dispose();
                _abort.Dispose();
                _cancel.Dispose();
                throw;
            }

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
            if (!TryRetain())
            {
                return;
            }

            try
            {
                _abort.Cancel();
            }
            finally
            {
                Release();
            }
        }

        public void Cancel()
        {
            if (!TryRetain())
            {
                return;
            }

            try
            {
                _cancel.Cancel();
            }
            finally
            {
                Release();
            }
        }

        public void Complete()
        {
            _completion.TrySetResult();
        }

        public void Dispose()
        {
            lock (_lifetimeGate)
            {
                if (_disposeRequested)
                {
                    return;
                }

                _disposeRequested = true;
            }

            Release();
        }

        public bool TryRetain()
        {
            lock (_lifetimeGate)
            {
                if (_disposeRequested)
                {
                    return false;
                }

                _references++;
                return true;
            }
        }

        public void Release()
        {
            lock (_lifetimeGate)
            {
                if (--_references != 0)
                {
                    return;
                }
            }

            Cleanup.Dispose(_feedbackMessageBuffer);
            Cleanup.Dispose(_resultBuffer);
            _abort.Dispose();
            _cancel.Dispose();
            _server._logger.LogDebug($"Action goal context [{GoalId}] disposed.");
        }

        private RosMessageBuffer CreateFeedbackMessage()
        {
            var buffer = _server.CreateFeedbackBuffer();

            if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
            {
                _server._typesupport.FeedbackMessage.AsRef<UUID.Priv>(buffer.Data, 0).CopyFrom(GoalId);
            }
            else
            {
                _server._typesupport.FeedbackMessage.AsRef<UUID.PrivV2>(buffer.Data, 0).CopyFrom(GoalId);
            }

            return buffer;
        }

        private void CopyFeedbackFrom(RosMessageBuffer src, RosMessageBuffer destination)
        {
            if (!_server._functions.CopyFeedback(src.Data,
                _server._typesupport.FeedbackMessage.GetMemberPointer(destination.Data, 1)))
            {
                throw new RclException("Unable to copy feedback buffer.");
            }
        }

        private void RetainForFeedback()
        {
            ObjectDisposedException.ThrowIf(Volatile.Read(ref _server._disposed) != 0 || !TryRetain(), this);
        }

        public void Report(RosMessageBuffer value)
        {
            RetainForFeedback();

            try
            {
                lock (_feedbackGate)
                {
                    CopyFeedbackFrom(value, _feedbackMessageBuffer);
                    _server._feedbackPublisher.Publish(_feedbackMessageBuffer);
                }
            }
            finally
            {
                Release();
            }
        }

        public ValueTask ReportAsync(RosMessageBuffer buffer, CancellationToken cancellationToken = default)
        {
            cancellationToken.ThrowIfCancellationRequested();
            RetainForFeedback();
            var message = RosMessageBuffer.Empty;

            try
            {
                message = CreateFeedbackMessage();
                CopyFeedbackFrom(buffer, message);
                cancellationToken.ThrowIfCancellationRequested();
                return PublishFeedbackAsync(message);
            }
            catch
            {
                message.Dispose();
                Release();
                throw;
            }
        }

        private async ValueTask PublishFeedbackAsync(RosMessageBuffer message)
        {
            try
            {
                using (message)
                {
                    await _server.PublishFeedbackAsync(message).ConfigureAwait(false);
                }
            }
            finally
            {
                Release();
            }
        }
    }
}
