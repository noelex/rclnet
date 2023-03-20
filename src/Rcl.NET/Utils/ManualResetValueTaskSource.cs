using System.Threading.Tasks.Sources;

internal sealed class ManualResetValueTaskSource<T> : IValueTaskSource<T>, IValueTaskSource
{
    private ManualResetValueTaskSourceCore<T> _core; // mutable struct; do not make this readonly

    private Action<object?>? _finallyCallback;

    private object? _finallyCallbackState;

    public bool RunContinuationsAsynchronously { get => _core.RunContinuationsAsynchronously; set => _core.RunContinuationsAsynchronously = value; }

    public short Version => _core.Version;

    public bool DisableContextCapture { get; set; }

    public object? Tag { get; set; }

    public void Reset()
    {
        _core.Reset();
        _finallyCallback = null;
        _finallyCallbackState = null;
        DisableContextCapture = false;
        Tag = null;
    }

    public void OnFinally(Action<object?> continuation, object? state)
    {
        _finallyCallback = continuation;
        _finallyCallbackState = state;
    }

    public void SetResult(T result) => _core.SetResult(result);

    public void SetException(Exception error) => _core.SetException(error);

    public T GetResult(short token)
    {
        try
        {
            return _core.GetResult(token);
        }
        finally
        {
            CallAndResetFinallyCallback(token);
        }
    }

    void IValueTaskSource.GetResult(short token)
    {
        try
        {
            _core.GetResult(token);
        }
        finally
        {
            CallAndResetFinallyCallback(token);
        }
    }

    private void CallAndResetFinallyCallback(short token)
    {
        // Call _finallyCallback exactly once when completed.
        if (_core.GetStatus(token) != ValueTaskSourceStatus.Pending)
        {
            Interlocked.Exchange(ref _finallyCallback, null)?.Invoke(_finallyCallbackState);
        }
    }

    public ValueTaskSourceStatus GetStatus(short token) => _core.GetStatus(token);

    [Obsolete("OnCompleted this not supposed to be called by application code. Use OnFinally instead.", true)]
    public void OnCompleted(Action<object?> continuation, object? state, short token, ValueTaskSourceOnCompletedFlags flags)
    {
        _core.OnCompleted(continuation, state, token, DisableContextCapture ? ValueTaskSourceOnCompletedFlags.None : flags);
    }
}