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
        return _core.GetResult(token);
    }

    void IValueTaskSource.GetResult(short token)
    {
        _core.GetResult(token);
    }

    public ValueTaskSourceStatus GetStatus(short token) => _core.GetStatus(token);

    [Obsolete("OnCompleted this not supposed to be called by application code. Use OnFinally instead.", true)]
    public void OnCompleted(Action<object?> continuation, object? state, short token, ValueTaskSourceOnCompletedFlags flags)
    {
        var args = ObjectPool.Rent<CompleteCallbacks>()
            .Reset(continuation, state, _finallyCallback, _finallyCallbackState);

        _core.OnCompleted(static s =>
        {
            var arg = (CompleteCallbacks)s!;
            try
            {
                arg.Callback?.Invoke(arg.State);
            }
            finally
            {
                arg.FinallyCallback?.Invoke(arg.FinallyCallbackState);
                ObjectPool.Return(arg);
            }
        }, args, token, DisableContextCapture ? ValueTaskSourceOnCompletedFlags.None : flags);
    }

    private class CompleteCallbacks
    {
        public Action<object?>? Callback { get; private set; }

        public object? State { get; private set; }

        public Action<object?>? FinallyCallback { get; private set; }

        public object? FinallyCallbackState { get; private set; }

        public CompleteCallbacks Reset(Action<object?>? callback, object? state,
             Action<object?>? finallyCallback, object? finallyCallbackState)
        {
            Callback = callback;
            State = state;
            FinallyCallback = finallyCallback;
            FinallyCallbackState = finallyCallbackState;
            return this;
        }
    }
}