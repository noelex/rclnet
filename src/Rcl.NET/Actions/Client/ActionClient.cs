﻿using Rcl.Internal.Clients;
using Rcl.Introspection;
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

    private readonly CancellationTokenSource _cts = new();

    public ActionClient(RclNodeImpl node, string actionName, ActionClientOptions options)
    {
        _node = node;
        _textEncoding = options.TextEncoding;
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
            using (buffer) ProcessFeedbackMessage(introspection, buffer);
        }
    }

    private void ProcessFeedbackMessage(IMessageIntrospection introspection, RosMessageBuffer buffer)
    {
        // UUID goal_id;
        // TFeedback.Priv feedback;
        Debug.Assert(introspection.MemberCount == 2);
        Debug.Assert(introspection.GetMemberName(0) == "goal_id");
        Debug.Assert(introspection.GetMemberName(1) == "feedback");

        ref var uuid = ref introspection.AsRef<UUID.Priv>(buffer.Data, 0);
        if (!_goals.TryGetValue(uuid, out var ctx) || !ctx.HasFeedbackListeners)
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
            using (buffer) ProcessStatusArray(buffer);
        }
    }

    private void ProcessStatusArray(RosMessageBuffer buffer)
    {
        ref var array = ref buffer.AsRef<GoalStatusArray.Priv>();
        foreach (var status in array.StatusList.AsSpan())
        {
            var id = status.GoalInfo.GoalId.ToGuid();
            if (_goals.TryGetValue(id, out var ctx))
            {
                ctx.OnStatusChanged((ActionGoalStatus)status.Status);
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
        Debug.Assert(_typesupport.GoalService.Response.SizeOf == Unsafe.SizeOf<SendGoalResponse>());
        return _typesupport.GoalService.Response
            .AsRef<SendGoalResponse>(responseBuffer.Data, 0).Accepted;
    }

    public async Task<INativeActionGoalContext> SendGoalAsync(RosMessageBuffer goalBuffer, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        var accepted = false;
        var uid = Guid.NewGuid();

        var tracker = new NativeActionGoalContext(uid, this);
        _goals[uid] = tracker;

        try
        {
            accepted = await InvokeSendGoalAsync(goalBuffer, uid, timeoutMilliseconds, cancellationToken).ConfigureAwait(false);

            if (!accepted)
            {
                throw new RclException("Action goal is not accepted by server.");
            }

            return tracker;
        }
        finally
        {
            if (!accepted) tracker.Dispose();
        }
    }

    public Task<INativeActionGoalContext> SendGoalAsync(RosMessageBuffer goalBuffer, TimeSpan timeout, CancellationToken cancellationToken = default)
        => SendGoalAsync(goalBuffer, (int)timeout.TotalMilliseconds, cancellationToken);

    public Task<INativeActionGoalContext> SendGoalAsync(RosMessageBuffer goalBuffer, CancellationToken cancellationToken = default)
        => SendGoalAsync(goalBuffer, cancellationToken);

    public async Task<IActionGoalContext<TResult, TFeedback>> SendGoalAsync(TGoal goal, int timeoutMilliseconds, CancellationToken cancellationToken = default)
    {
        var accepted = false;
        var uid = Guid.NewGuid();

        var tracker = new ActionGoalContext<TResult, TFeedback>(uid, this, _textEncoding);
        _goals[uid] = tracker;

        try
        {
            using var goalBuffer = RosMessageBuffer.Create<TGoal>();
            goal.WriteTo(goalBuffer.Data, _textEncoding);

            accepted = await InvokeSendGoalAsync(goalBuffer, uid, timeoutMilliseconds, cancellationToken).ConfigureAwait(false);

            if (!accepted)
            {
                throw new RclException("Action goal is not accepted by server.");
            }

            return tracker;
        }
        finally
        {
            if (!accepted) tracker.Dispose();
        }
    }

    public Task<IActionGoalContext<TResult, TFeedback>> SendGoalAsync(TGoal goal, TimeSpan timeout, CancellationToken cancellationToken = default)
        => SendGoalAsync(goal, (int)timeout.TotalMilliseconds, cancellationToken);

    public Task<IActionGoalContext<TResult, TFeedback>> SendGoalAsync(TGoal goal, CancellationToken cancellationToken = default)
        => SendGoalAsync(goal, -1, cancellationToken);

    private void BuildSendGoalRequest(Guid goalId, nint goalBuffer, nint requestBuffer)
    {
        var requestIntrospection = _typesupport.GoalService.Request;

        using var goalid = new UUID.Priv(goalId);

        // Copy fields into SendGoal_Request:
        //
        // UUID goal_id;
        // TGoal.Priv goal;
        Debug.Assert(requestIntrospection.MemberCount == 2);
        Debug.Assert(requestIntrospection.GetMemberName(0) == "goal_id");
        Debug.Assert(requestIntrospection.GetMemberName(1) == "goal");
        requestIntrospection.AsRef<UUID.Priv>(requestBuffer, 0).CopyFrom(in goalid);
        if (!_bufferHelper.CopyGoal(goalBuffer, requestIntrospection.GetMemberPointer(requestBuffer, 1)))
        {
            throw new RclException("Unable to copy goal buffer, send goal failed.");
        }
    }

    public void Dispose()
    {
        _feedbackSubscription.Dispose();
        _statusSubscription.Dispose();
        _cancelGoalClient.Dispose();
        _sendGoalClient.Dispose();
        _getResultClient.Dispose();

        _cts.Cancel();
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
