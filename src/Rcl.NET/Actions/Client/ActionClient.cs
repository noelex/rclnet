using Rcl.Internal.Clients;
using Rcl.Introspection;
using Rcl.Logging;
using Rosidl.Messages.Action;
using Rosidl.Messages.UniqueIdentifier;
using Rosidl.Runtime;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;

namespace Rcl.Actions.Client;

internal class ActionClient<TAction, TGoal, TResult, TFeedback>
    : IRclObject, IActionClientImpl, IActionClient<TGoal, TResult, TFeedback>
        where TAction : IAction<TGoal, TResult, TFeedback>
        where TGoal : IActionGoal
        where TResult : IActionResult
        where TFeedback : IActionFeedback
{
    private readonly IRclNativeSubscription _feedbackSubscription, _statusSubscription;
    private readonly IntrospectionClient _sendGoalClient, _getResultClient;
    private readonly RclClient<CancelGoalService, CancelGoalServiceRequest, CancelGoalServiceResponse> _cancelGoalClient;

    private readonly RclNodeImpl _node;
    private readonly ActionIntrospection _typesupport;
    private readonly Encoding _textEncoding;

    private readonly MessageBufferHelper _bufferHelper = new(TAction.TypeSupportName);
    private readonly ConcurrentDictionary<Guid, ActionGoalContextBase> _goals = new();
    private readonly ConcurrentDictionary<Guid, PendingGoalSend> _pendingSends = new();
    private readonly object _lifetimeGate = new();

    private readonly CancellationTokenSource _cts = new();
    private readonly CancellationToken _shutdownToken;
    private readonly TimeSpan _abandonedGoalTimeout;
    private readonly RclTimeProvider _timeoutProvider;
    private bool _disposed;

    public ActionClient(RclNodeImpl node, string actionName, ActionClientOptions options,
        TimeSpan? abandonedGoalTimeout = null)
    {
        _node = node;
        _textEncoding = options.TextEncoding;
        _shutdownToken = _cts.Token;
        _abandonedGoalTimeout = abandonedGoalTimeout ?? TimeSpan.FromSeconds(30);
        _timeoutProvider = new RclTimeProvider(node.Context, node.Clock);
        Options = options;

        var statusTopicName = actionName + Constants.StatusTopic;
        var feedbackTopicName = actionName + Constants.FeedbackTopic;
        var sendGoalServiceName = actionName + Constants.SendGoalService;
        var cancelGoalServiceName = actionName + Constants.CancelGoalService;
        var getResultServiceName = actionName + Constants.GetResultService;

        _typesupport = new ActionIntrospection(TAction.GetTypeSupportHandle());

        var done = false;

        try
        {
            _cancelGoalClient = new(node, cancelGoalServiceName,
                new(qos: options.CancelServiceQos, textEncoding: options.TextEncoding));
            _sendGoalClient = new(node, sendGoalServiceName,
                _typesupport.GoalServiceTypeSupport, new(qos: options.GoalServiceQos));
            _getResultClient = new(node, getResultServiceName,
                _typesupport.ResultServiceTypeSupport, new(qos: options.ResultServiceQos));

            _statusSubscription = _node.CreateNativeSubscription<GoalStatusArray>(
                statusTopicName,
               new(qos: options.StatusTopicQos));
            _feedbackSubscription = _node.CreateNativeSubscription(
                feedbackTopicName,
                _typesupport.FeedbackMessageTypeSupport,
                new(qos: options.FeedbackTopicQos));

            // In case the given action name gets normalized.
            var sep = _feedbackSubscription.Name!.LastIndexOf("/_action/feedback");
            Name = _feedbackSubscription.Name.Substring(0, sep);

            _ = ReadFeedbacksAsync(_cts.Token);
            _ = ReadStatusAsync(_cts.Token);
            done = true;
        }
        finally
        {
            if (!done)
            {
                _feedbackSubscription?.Dispose();
                _statusSubscription?.Dispose();
                _getResultClient?.Dispose();
                _sendGoalClient?.Dispose();
                _cancelGoalClient?.Dispose();
                _cts.Dispose();
                _timeoutProvider.Dispose();
            }
        }
    }

    public string Name { get; }

    public ActionClientOptions Options { get; }

    public bool IsServerAvailable =>
        _sendGoalClient.IsServerAvailable && _getResultClient.IsServerAvailable && _cancelGoalClient.IsServerAvailable &&
        _feedbackSubscription.Publishers > 0 && _statusSubscription.Publishers > 0;

    private async Task ReadFeedbacksAsync(CancellationToken cancellationToken = default)
    {
        var introspection = _typesupport.FeedbackMessage;

        await foreach (var buffer in _feedbackSubscription.ReadAllAsync(cancellationToken).ConfigureAwait(false))
        {
            using (buffer)
            {
                ProcessFeedbackMessage(introspection, buffer);
            }
        }
    }

    private void ProcessFeedbackMessage(IMessageIntrospection introspection, RosMessageBuffer buffer)
    {
        // UUID goal_id;
        // TFeedback.Priv feedback;
        Debug.Assert(introspection.MemberCount == 2);
        Debug.Assert(introspection.GetMemberName(0) == "goal_id");
        Debug.Assert(introspection.GetMemberName(1) == "feedback");

        var goalId = RosidlRuntime.NativeAbi switch
        {
            RosidlNativeAbi.V1 => introspection.AsRef<UUID.Priv>(buffer.Data, 0).ToGuid(),
            RosidlNativeAbi.V2 => introspection.AsRef<UUID.PrivV2>(buffer.Data, 0).ToGuid(),
            _ => throw new UnreachableException(),
        };

        if (!_goals.TryGetValue(goalId, out var ctx) || !ctx.HasFeedbackListeners)
        {
            return;
        }

        var feedbackBuffer = _bufferHelper.CreateFeedbackBuffer();
        _bufferHelper.CopyFeedback(introspection.GetMemberPointer(buffer.Data, 1), feedbackBuffer.Data);
        ctx.OnFeedbackReceived(feedbackBuffer);
    }

    private async Task ReadStatusAsync(CancellationToken cancellationToken)
    {
        await foreach (var buffer in _statusSubscription.ReadAllAsync(cancellationToken).ConfigureAwait(false))
        {
            using (buffer)
            {
                ProcessStatusArray(buffer);
            }
        }
    }

    private void ProcessStatusArray(RosMessageBuffer buffer)
    {
        if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
        {
            foreach (ref readonly var status in buffer.AsRef<GoalStatusArray.Priv>().StatusList.AsSpan())
            {
                ProcessStatus(status.GoalInfo.GoalId.ToGuid(), status.Status);
            }

            return;
        }

        foreach (ref readonly var status in buffer.AsRef<GoalStatusArray.PrivV2>().StatusList.AsSpan())
        {
            ProcessStatus(status.GoalInfo.GoalId.ToGuid(), status.Status);
        }

        void ProcessStatus(Guid id, sbyte status)
        {
            if (_goals.TryGetValue(id, out var ctx))
            {
                ctx.OnStatusChanged((ActionGoalStatus)status);
            }
        }
    }

    private async Task<bool> InvokeSendGoalAsync(RosMessageBuffer goalBuffer, Guid goalId, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        using var requestBuffer = _sendGoalClient.CreateRequestBuffer();
        BuildSendGoalRequest(goalId, goalBuffer.Data, requestBuffer.Data);

        using var responseBuffer = await _sendGoalClient.InvokeAsync(requestBuffer, timeoutMilliseconds, cancellationToken).ConfigureAwait(false);

        // bool accepted;
        // Time.Priv stamp;
        Debug.Assert(_typesupport.GoalService.Response.MemberCount == 2);

        if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
        {
            Debug.Assert(_typesupport.GoalService.Response.SizeOf == Unsafe.SizeOf<SendGoalResponse>());
            return responseBuffer.AsRef<SendGoalResponse>().Accepted;
        }

        Debug.Assert(_typesupport.GoalService.Response.SizeOf == Unsafe.SizeOf<SendGoalResponseV2>());
        return responseBuffer.AsRef<SendGoalResponseV2>().Accepted;
    }

    public async Task<INativeActionGoalContext> SendGoalAsync(RosMessageBuffer goalBuffer, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        ValidateGoalSend(timeoutMilliseconds, cancellationToken);
        var tracker = new NativeActionGoalContext(Guid.NewGuid(), this);
        return await SendGoalCoreAsync(goalBuffer, tracker, timeoutMilliseconds, cancellationToken).ConfigureAwait(false);
    }

    public Task<INativeActionGoalContext> SendGoalAsync(RosMessageBuffer goalBuffer, TimeSpan timeout, CancellationToken cancellationToken = default)
        => SendGoalAsync(goalBuffer, (int)timeout.TotalMilliseconds, cancellationToken);

    public Task<INativeActionGoalContext> SendGoalAsync(RosMessageBuffer goalBuffer, CancellationToken cancellationToken = default)
        => SendGoalAsync(goalBuffer, -1, cancellationToken);

    public async Task<IActionGoalContext<TResult, TFeedback>> SendGoalAsync(TGoal goal, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        ValidateGoalSend(timeoutMilliseconds, cancellationToken);
        using var goalBuffer = RosMessageBuffer.Create<TGoal>();
        goal.WriteTo(goalBuffer.Data, _textEncoding);

        var tracker = new ActionGoalContext<TResult, TFeedback>(Guid.NewGuid(), this, _textEncoding);
        return await SendGoalCoreAsync(goalBuffer, tracker, timeoutMilliseconds, cancellationToken).ConfigureAwait(false);
    }

    public Task<IActionGoalContext<TResult, TFeedback>> SendGoalAsync(TGoal goal, TimeSpan timeout, CancellationToken cancellationToken = default)
        => SendGoalAsync(goal, (int)timeout.TotalMilliseconds, cancellationToken);

    public Task<IActionGoalContext<TResult, TFeedback>> SendGoalAsync(TGoal goal, CancellationToken cancellationToken = default)
        => SendGoalAsync(goal, -1, cancellationToken);

    private void ValidateGoalSend(int timeoutMilliseconds, CancellationToken cancellationToken)
    {
        lock (_lifetimeGate)
        {
            ObjectDisposedException.ThrowIf(_disposed, this);
        }

        cancellationToken.ThrowIfCancellationRequested();
        ArgumentOutOfRangeException.ThrowIfLessThan(timeoutMilliseconds, Timeout.Infinite);
        if (timeoutMilliseconds == 0)
        {
            throw new TimeoutException();
        }
    }

    private async Task<TTracker> SendGoalCoreAsync<TTracker>(RosMessageBuffer goalBuffer,
        TTracker tracker, int timeoutMilliseconds, CancellationToken cancellationToken)
        where TTracker : ActionGoalContextBase
    {
        // Prepare the timeout before sending so a closed clock cannot leave a remote goal behind.
        using var timeout = timeoutMilliseconds == Timeout.Infinite
            ? null
            : new CancellationTokenSource(TimeSpan.FromMilliseconds(timeoutMilliseconds), _timeoutProvider);
        using var waitCancellation = timeout is null
            ? null
            : CancellationTokenSource.CreateLinkedTokenSource(cancellationToken, timeout.Token);
        var pending = new PendingGoalSend(tracker, _shutdownToken);
        var handedOff = false;
        var cleaningUp = false;

        try
        {
            lock (_lifetimeGate)
            {
                ObjectDisposedException.ThrowIf(_disposed, this);
                (waitCancellation?.Token ?? cancellationToken).ThrowIfCancellationRequested();
                _goals[tracker.GoalId] = tracker;
                _pendingSends[tracker.GoalId] = pending;
            }

            // The request owns its copied buffer and remains observable after the caller stops waiting.
            pending.Acceptance = InvokeSendGoalAsync(goalBuffer, tracker.GoalId,
                Timeout.Infinite, pending.Cancellation.Token);

            var accepted = await pending.Acceptance.WaitAsync(
                waitCancellation?.Token ?? cancellationToken).ConfigureAwait(false);

            if (!accepted)
            {
                throw new RclException("Action goal is not accepted by server.");
            }

            cancellationToken.ThrowIfCancellationRequested();
            lock (_lifetimeGate)
            {
                ObjectDisposedException.ThrowIf(_disposed, this);
                handedOff = true;
            }

            return tracker;
        }
        catch (Exception error) when (error is TimeoutException ||
            error is OperationCanceledException &&
            (cancellationToken.IsCancellationRequested || timeout?.IsCancellationRequested == true))
        {
            var timedOut = error is TimeoutException || !cancellationToken.IsCancellationRequested;
            cleaningUp = true;
            pending.Cleanup = CleanupAbandonedGoalAsync(pending,
                timedOut ? "send timeout" : "caller cancellation");
            if (error is OperationCanceledException)
            {
                cancellationToken.ThrowIfCancellationRequested();
                throw new TimeoutException();
            }

            throw;
        }
        finally
        {
            if (!cleaningUp)
            {
                await ReleasePendingSendAsync(pending, disposeTracker: !handedOff).ConfigureAwait(false);
            }
        }
    }

    private async Task CleanupAbandonedGoalAsync(PendingGoalSend pending, string reason)
    {
        using var deadline = CancellationTokenSource.CreateLinkedTokenSource(_shutdownToken);
        deadline.CancelAfter(_abandonedGoalTimeout);
        var lastFailure = "Acceptance response was not received.";

        try
        {
            // Await the original response, including a response that arrives after cancellation/timeout.
            if (!await pending.Acceptance.WaitAsync(deadline.Token).ConfigureAwait(false))
            {
                return;
            }

            lastFailure = "Cancellation response was not received.";
            while (true)
            {
                deadline.Token.ThrowIfCancellationRequested();
                if (pending.Tracker.Status is ActionGoalStatus.Succeeded or ActionGoalStatus.Canceled or ActionGoalStatus.Aborted)
                {
                    return;
                }

                try
                {
                    var response = await _cancelGoalClient.InvokeAsync(new CancelGoalServiceRequest
                    {
                        GoalInfo = new GoalInfo { GoalId = new UUID { Uuid = pending.Tracker.GoalId.ToByteArray() } }
                    }, TimeSpan.FromSeconds(3), deadline.Token).ConfigureAwait(false);

                    if (response.ReturnCode == CancelGoalServiceResponse.ERROR_GOAL_TERMINATED ||
                        response.ReturnCode == CancelGoalServiceResponse.ERROR_NONE &&
                        response.GoalsCanceling.Any(x => new Guid(x.GoalId.Uuid) == pending.Tracker.GoalId))
                    {
                        return;
                    }

                    lastFailure = $"Cancel response code {response.ReturnCode}; goal was not confirmed canceling.";
                    if (response.ReturnCode != CancelGoalServiceResponse.ERROR_UNKNOWN_GOAL_ID &&
                        response.ReturnCode != CancelGoalServiceResponse.ERROR_NONE)
                    {
                        LogCleanupFailure(pending, reason, lastFailure);
                        return;
                    }
                }
                catch (Exception error) when (error is TimeoutException or RclException)
                {
                    lastFailure = error.Message;
                }

                await Task.Delay(TimeSpan.FromMilliseconds(250), deadline.Token).ConfigureAwait(false);
            }
        }
        catch (OperationCanceledException) when (deadline.IsCancellationRequested)
        {
            LogCleanupFailure(pending, reason, _shutdownToken.IsCancellationRequested
                ? "Client disposed before cancellation was confirmed."
                : $"Cleanup deadline expired. {lastFailure}");
        }
        catch (Exception error)
        {
            LogCleanupFailure(pending, reason, error.ToString());
        }
        finally
        {
            await ReleasePendingSendAsync(pending, disposeTracker: true).ConfigureAwait(false);
        }
    }

    private void LogCleanupFailure(PendingGoalSend pending, string reason, string failure)
    {
        _node.Logger.LogError($"Unable to cancel abandoned goal [{pending.Tracker.GoalId}] " +
            $"for action '{Name}' after {reason}: {failure}");
    }

    private async Task ReleasePendingSendAsync(PendingGoalSend pending, bool disposeTracker)
    {
        pending.Cancellation.Cancel();
        try
        {
            await pending.Acceptance.ConfigureAwait(false);
        }
        catch (Exception)
        {
            // The caller or cleanup worker has already handled this request's failure.
        }

        pending.Cancellation.Dispose();
        if (disposeTracker)
        {
            pending.Tracker.Dispose();
            _ = pending.Tracker.Completion.Exception;
        }

        _pendingSends.TryRemove(pending.Tracker.GoalId, out _);
    }

    private sealed class PendingGoalSend
    {
        public PendingGoalSend(ActionGoalContextBase tracker, CancellationToken shutdownToken)
        {
            Tracker = tracker;
            Cancellation = CancellationTokenSource.CreateLinkedTokenSource(shutdownToken);
        }

        public ActionGoalContextBase Tracker { get; }
        public CancellationTokenSource Cancellation { get; }
        public Task<bool> Acceptance { get; set; } = Task.FromResult(false);
        public Task? Cleanup { get; set; }
    }

    private void BuildSendGoalRequest(Guid goalId, nint goalBuffer, nint requestBuffer)
    {
        var requestIntrospection = _typesupport.GoalService.Request;

        // Copy fields into SendGoal_Request:
        //
        // UUID goal_id;
        // TGoal.Priv goal;
        Debug.Assert(requestIntrospection.MemberCount == 2);
        Debug.Assert(requestIntrospection.GetMemberName(0) == "goal_id");
        Debug.Assert(requestIntrospection.GetMemberName(1) == "goal");

        if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
        {
            requestIntrospection.AsRef<UUID.Priv>(requestBuffer, 0).CopyFrom(goalId);
        }
        else
        {
            requestIntrospection.AsRef<UUID.PrivV2>(requestBuffer, 0).CopyFrom(goalId);
        }

        if (!_bufferHelper.CopyGoal(goalBuffer, requestIntrospection.GetMemberPointer(requestBuffer, 1)))
        {
            throw new RclException("Unable to copy goal buffer, send goal failed.");
        }
    }

    public void Dispose()
    {
        lock (_lifetimeGate)
        {
            if (_disposed)
            {
                return;
            }

            _disposed = true;
        }

        _cts.Cancel();
        _feedbackSubscription.Dispose();
        _statusSubscription.Dispose();
        _cancelGoalClient.Dispose();
        _sendGoalClient.Dispose();
        _getResultClient.Dispose();
        _timeoutProvider.Dispose();

        _cts.Dispose();

        foreach (var (_, v) in _goals)
        {
            v.Dispose();
        }
    }

    ActionIntrospection IActionClientImpl.Introspection => _typesupport;

    IRclClient<CancelGoalServiceRequest, CancelGoalServiceResponse> IActionClientImpl.CancelClient => _cancelGoalClient;

    IntrospectionClient IActionClientImpl.GetResultClient => _getResultClient;

    MessageBufferHelper IActionClientImpl.BufferHelper => _bufferHelper;

    bool IActionClientImpl.TryRemoveGoal(Guid goalId) => _goals.TryRemove(goalId, out _);

    public Task<bool> TryWaitForServerAsync(int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => _node.Graph.TryWaitForActionServerAsync(Name, timeoutMilliseconds, cancellationToken);

    public Task<bool> TryWaitForServerAsync(TimeSpan timeout, CancellationToken cancellationToken = default)
        => _node.Graph.TryWaitForActionServerAsync(Name, timeout, cancellationToken);

    public Task WaitForServerAsync(int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => _node.Graph.WaitForActionServerAsync(Name, timeoutMilliseconds, cancellationToken);

    public Task WaitForServerAsync(TimeSpan timeout, CancellationToken cancellationToken = default)
        => _node.Graph.WaitForActionServerAsync(Name, timeout, cancellationToken);

    public Task WaitForServerAsync(CancellationToken cancellationToken = default)
        => _node.Graph.WaitForActionServerAsync(Name, cancellationToken);
}
