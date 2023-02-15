using Rcl.Internal.Clients;
using Rcl.Introspection;
using Rcl.Qos;
using Rosidl.Messages.Action;
using Rosidl.Messages.UniqueIdentifier;
using Rosidl.Runtime;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Text;

namespace Rcl.Actions.Client;

internal class ActionClient<TAction, TGoal, TResult, TFeedback>
    : IRclObject, IActionClientImpl, IActionClient<TGoal, TResult, TFeedback>
        where TAction : IAction<TGoal, TResult, TFeedback>
        where TGoal : IActionGoal
        where TResult : IActionResult
        where TFeedback : IActionFeedback
{
    private readonly string _statusTopicName, _feedbackTopicName,
        _sendGoalServiceName, _cancelGoalServiceName, _getResultServiceName;

    private readonly IRclNativeSubscription _feedbackSubscription, _statusSubscription;
    private readonly IntrospectionRclClient _sendGoalClient, _getResultClient;
    private readonly RclClient<CancelGoalService, CancelGoalServiceRequest, CancelGoalServiceResponse> _cancelGoalClient;

    private readonly RclNodeImpl _node;
    private readonly ActionIntrospection _typesupport;
    private readonly Encoding _textEncoding;

    private readonly DynamicFunctionTable _functions = new(TAction.TypeSupportName);
    private readonly ConcurrentDictionary<Guid, ActionGoalContextBase> _goals = new();

    private readonly CancellationTokenSource _cts = new();

    public ActionClient(RclNodeImpl node, string actionName, Encoding textEncoding)
    {
        Name = actionName;
        
        _node = node;
        _textEncoding = textEncoding;

        _statusTopicName = actionName + "/_action/status";
        _feedbackTopicName = actionName + "/_action/feedback";
        _sendGoalServiceName = actionName + "/_action/send_goal";
        _cancelGoalServiceName = actionName + "/_action/cancel_goal";
        _getResultServiceName = actionName + "/_action/get_result";

        _typesupport = new ActionIntrospection(TAction.GetTypeSupportHandle());

        var done = false;
        try
        {
            _cancelGoalClient = new(node, _cancelGoalServiceName, QosProfile.ServicesDefault, textEncoding);
            _sendGoalClient = new(node, _sendGoalServiceName,
                _typesupport.GoalServiceTypeSupport, QosProfile.ServicesDefault);
            _getResultClient = new(node, _getResultServiceName,
                _typesupport.ResultServiceTypeSupport, QosProfile.ServicesDefault);

            _statusSubscription = _node.CreateNativeSubscription<GoalStatusArray>(
                _statusTopicName,
                new(Depth: 1, Durability: DurabilityPolicy.TransientLocal, Reliability: ReliabilityPolicy.Reliable));
            _feedbackSubscription = _node.CreateNativeSubscription(
                _feedbackTopicName,
                _typesupport.FeedbackMessageTypeSupport,
                QosProfile.SensorData);

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

    public bool IsServerAvailable =>
        _sendGoalClient.IsServerAvailable && _getResultClient.IsServerAvailable && _cancelGoalClient.IsServerAvailable &&
        _feedbackSubscription.Publishers > 0 && _statusSubscription.Publishers > 0;

    private async Task ReadFeedbacksAsync(CancellationToken cancellationToken = default)
    {
        var introspection = _typesupport.FeedbackMessage;
        await foreach (var buffer in _feedbackSubscription.ReadAllAsync(cancellationToken))
        {
            using (buffer) ProcessFeedbackMessage(introspection, buffer);
        }
    }

    private unsafe void ProcessFeedbackMessage(MessageIntrospection introspection, RosMessageBuffer buffer)
    {
        // UUID goal_id;
        // TFeedback.Priv feedback;
        Debug.Assert(introspection.MemberCount == 2);
        Debug.Assert(introspection.GetMemberName(0) == "goal_id");
        Debug.Assert(introspection.GetMemberName(1) == "feedback");

        ref var uuid = ref introspection.AsRef<UUID.Priv>(buffer.Data, 0);
        var goalId = new Guid(uuid.Uuid.AsSpan());

        if (!_goals.TryGetValue(goalId, out var ctx) || !ctx.HasFeedbackListeners)
        {
            return;
        }

        var feedbackBuffer = introspection.CreateBuffer();
        _functions.CopyFeedback(introspection.GetMemberPointer(buffer.Data, 1), feedbackBuffer.Data);
        ctx.OnFeedbackReceived(feedbackBuffer);
    }

    private async Task ReadStatusAsync(CancellationToken cancellationToken)
    {
        await foreach (var buffer in _statusSubscription.ReadAllAsync(cancellationToken))
        {
            using (buffer) ProcessStatusArray(buffer);
        }
    }

    private void ProcessStatusArray(RosMessageBuffer buffer)
    {
        ref var array = ref buffer.AsRef<GoalStatusArray.Priv>();
        foreach (var status in array.StatusList.AsSpan())
        {
            var id = new Guid(status.GoalInfo.GoalId.Uuid.AsSpan());
            if (_goals.TryGetValue(id, out var ctx))
            {
                ctx.OnStatusChanged((ActionGoalStatus)status.Status);
            }
        }
    }

    private async Task<bool> InvokeSendGoalAsync(RosMessageBuffer goalBuffer, Guid goalId, TimeSpan timeout, CancellationToken cancellationToken = default)
    {
        using var requestBuffer = _sendGoalClient.CreateRequestBuffer();
        BuildSendGoalRequest(goalId, goalBuffer.Data, requestBuffer.Data);

        using var responseBuffer = await _sendGoalClient.InvokeAsync(requestBuffer, timeout, cancellationToken);

        // bool accepted;
        // Time.Priv stamp;
        Debug.Assert(_typesupport.GoalService.Response.MemberCount == 2);
        Debug.Assert(_typesupport.GoalService.Response.GetMemberName(0) == "accepted");
        Debug.Assert(_typesupport.GoalService.Response.GetMemberName(1) == "stamp");
        return _typesupport.GoalService.Response.AsRef<bool>(responseBuffer.Data, 0);
    }

    public async Task<INativeActionGoalContext> SendGoalAsync(RosMessageBuffer goalBuffer, TimeSpan timeout, CancellationToken cancellationToken = default)
    {
        var accepted = false;
        var uid = Guid.NewGuid();

        var tracker = new NativeActionGoalContext(uid, this);
        _goals[uid] = tracker;

        try
        {
            accepted = await InvokeSendGoalAsync(goalBuffer, uid, timeout, cancellationToken);

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

    public Task<INativeActionGoalContext> SendGoalAsync(RosMessageBuffer goalBuffer, int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => SendGoalAsync(goalBuffer, TimeSpan.FromMilliseconds(timeoutMilliseconds), cancellationToken);

    public Task<INativeActionGoalContext> SendGoalAsync(RosMessageBuffer goalBuffer, CancellationToken cancellationToken = default)
        => SendGoalAsync(goalBuffer, _sendGoalClient.DefaultRequestTimeout, cancellationToken);

    public async Task<IActionGoalContext<TResult, TFeedback>> SendGoalAsync(TGoal goal, TimeSpan timeout, CancellationToken cancellationToken = default)
    {
        var accepted = false;
        var uid = Guid.NewGuid();

        var tracker = new ActionGoalContext<TResult, TFeedback>(uid, this, _textEncoding);
        _goals[uid] = tracker;

        try
        {
            using var goalBuffer = RosMessageBuffer.Create<TGoal>();
            goal.WriteTo(goalBuffer.Data, _textEncoding);

            accepted = await InvokeSendGoalAsync(goalBuffer, uid, timeout, cancellationToken);

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

    public Task<IActionGoalContext<TResult, TFeedback>> SendGoalAsync(TGoal goal, int timeoutMilliseconds, CancellationToken cancellationToken = default)
        => SendGoalAsync(goal, TimeSpan.FromMilliseconds(timeoutMilliseconds), cancellationToken);

    public Task<IActionGoalContext<TResult, TFeedback>> SendGoalAsync(TGoal goal, CancellationToken cancellationToken = default)
        => SendGoalAsync(goal, _sendGoalClient.DefaultRequestTimeout, cancellationToken);

    private void BuildSendGoalRequest(Guid goalId, nint goalBuffer, nint requestBuffer)
    {
        var requestIntrospection = _typesupport.GoalService.Request;

        using var goalid = new UUID.Priv(goalId);

        unsafe
        {
            // Copy fields into SendGoal_Request:
            //
            // UUID goal_id;
            // TGoal.Priv goal;
            Debug.Assert(requestIntrospection.MemberCount == 2);
            Debug.Assert(requestIntrospection.GetMemberName(0) == "goal_id");
            Debug.Assert(requestIntrospection.GetMemberName(1) == "goal");
            requestIntrospection.AsRef<UUID.Priv>(requestBuffer, 0).CopyFrom(in goalid);
            if (!_functions.CopyGoal(goalBuffer, requestIntrospection.GetMemberPointer(requestBuffer, 1)))
            {
                throw new RclException("Unable to copy goal buffer, send goal failed.");
            }
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

    IntrospectionRclClient IActionClientImpl.GetResultClient => _getResultClient;

    DynamicFunctionTable IActionClientImpl.Functions => _functions;

    bool IActionClientImpl.TryRemoveGoal(Guid goalId) => _goals.TryRemove(goalId, out _);
}
