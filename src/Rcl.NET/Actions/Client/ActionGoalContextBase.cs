using Rosidl.Messages.Action;
using Rosidl.Messages.UniqueIdentifier;
using System.Diagnostics;

namespace Rcl.Actions.Client;

internal abstract class ActionGoalContextBase : IDisposable, IActionGoalContext
{
    private readonly Guid _goalId;
    private readonly TaskCompletionSource<ActionGoalStatus> _completion = new();
    private readonly IActionClientImpl _client;

    public ActionGoalContextBase(Guid goalId, IActionClientImpl actionClient)
    {
        _goalId = goalId;
        _client = actionClient;
    }

    protected IActionClientImpl Client => _client;

    public Guid GoalId => _goalId;

    public ActionGoalStatus Status { get; private set; }

    public event Action<ActionGoalStatus>? StatusChanged;

    public abstract bool HasFeedbackListeners { get; }

    public abstract void OnFeedbackReceived(RosMessageBuffer feedback);

    protected virtual void OnGoalStateChanged(ActionGoalStatus state) { }

    public void OnStatusChanged(ActionGoalStatus state)
    {
        if (Status == state)
        {
            return;
        }

        Status = state;
        OnGoalStateChanged(state);
        StatusChanged?.Invoke(state);

        if (state is ActionGoalStatus.Aborted or ActionGoalStatus.Canceled or ActionGoalStatus.Succeeded)
        {
            _completion.TrySetResult(state);
        }
    }

    public Task<ActionGoalStatus> Completion => _completion.Task;

    public Task<RosMessageBuffer> GetResultAsync(int timeoutMilliseconds, CancellationToken cancellationToken)
        => GetResultAsync(TimeSpan.FromMilliseconds(timeoutMilliseconds), cancellationToken);

    public Task<RosMessageBuffer> GetResultAsync(CancellationToken cancellationToken)
        => GetResultAsync(Timeout.Infinite, cancellationToken);

    public async Task<RosMessageBuffer> GetResultAsync(TimeSpan timeout, CancellationToken cancellationToken)
    {
        using var requestBuffer = _client.GetResultClient.CreateRequestBuffer();
        BuildRequest(requestBuffer.Data);

        using var response = await _client.GetResultClient.InvokeAsync(requestBuffer, timeout, cancellationToken);
        ThrowIfNonSuccess(ProcessResponse(response.Data, out var result));
        return result;

        void BuildRequest(nint requestBuffer)
        {
            // GetResult_Request contains a single UUID field.
            using var goalId = new UUID.Priv(GoalId);

            _client.Introspection.ResultService.Request.AsRef<UUID.Priv>(requestBuffer, 0).CopyFrom(goalId);
        }

        unsafe ActionGoalStatus ProcessResponse(nint responseBuffer, out RosMessageBuffer resultBuffer)
        {
            resultBuffer = RosMessageBuffer.Empty;

            var introspection = _client.Introspection.ResultService.Response;

            // int8_t status
            // TResult.Priv result;
            Debug.Assert(introspection.MemberCount == 2);
            Debug.Assert(introspection.GetMemberName(0) == "status");
            Debug.Assert(introspection.GetMemberName(1) == "result");

            var state = introspection.AsRef<ActionGoalStatus>(responseBuffer, 0);
            if (state != ActionGoalStatus.Succeeded)
            {
                return state;
            }

            resultBuffer = new(_client.Functions.CreateResult(),
                (p, func) => ((DynamicFunctionTable)func!).DestroyResult(p), _client.Functions);

            _client.Functions.CopyResult(introspection.GetMemberPointer(responseBuffer, 1), resultBuffer.Data);
            return state;
        }
    }

    public async Task CancelAsync(TimeSpan timeout, CancellationToken cancellationToken)
    {
        using var requestBuffer = RosMessageBuffer.Create<CancelGoalServiceRequest>();
        PrepareRequest(requestBuffer);

        using var responseBuffer = await _client.CancelClient.InvokeAsync(requestBuffer, timeout, cancellationToken);
        ProcessResponse(responseBuffer);

        void PrepareRequest(RosMessageBuffer buffer)
        {
            ref var request = ref buffer.AsRef<CancelGoalServiceRequest.Priv>();
            request.GoalInfo.GoalId.CopyFrom(GoalId);
        }

        void ProcessResponse(RosMessageBuffer buffer)
        {
            ref var response = ref buffer.AsRef<CancelGoalServiceResponse.Priv>();
            if (response.ReturnCode != CancelGoalServiceResponse.ERROR_NONE)
            {
                throw new RclException($"Failed to cancel action goal, server returned status code {response.ReturnCode}.");
            }
        }
    }

    public Task CancelAsync(int timeoutMilliseconds, CancellationToken cancellationToken)
        => CancelAsync(TimeSpan.FromMilliseconds(timeoutMilliseconds), cancellationToken);

    public Task CancelAsync(CancellationToken cancellationToken)
        => CancelAsync(cancellationToken);

    private static void ThrowIfNonSuccess(ActionGoalStatus state)
    {
        if (state != ActionGoalStatus.Succeeded)
        {
            throw new RclException($"Failed to get action goal result. Action completed with state '{state}'.");
        }
    }

    protected virtual void OnDispose() { }

    public void Dispose()
    {
        if (_client.TryRemoveGoal(GoalId))
        {
            OnDispose();
            if (!_completion.Task.IsCompleted)
            {
                _completion.TrySetException(new ObjectDisposedException(GetType().Name));
            }
        }
    }
}