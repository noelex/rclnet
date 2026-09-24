namespace Rcl.Utils;

// Callback state is never pooled. Reuse the completion source only after setup,
// the winning producer and the consumer have all stopped accessing it.
internal sealed class PendingOperation<T>
{
    private readonly ManualResetValueTaskSource<T> _source = ObjectPool.Rent<ManualResetValueTaskSource<T>>();
    private readonly Action<PendingOperation<T>, Exception> _cancel;
    private CancellationTokenRegistration _cancellation, _timeoutRegistration;
    private CancellationTokenSource? _timeoutSource;
    private CancellationToken _token;
    private TimeSpan _timeout;
    private int _references = 2; // setup + consumer
    private int _terminal;

    internal long Key { get; set; }

    internal ValueTask<T> Task { get; }

    internal ValueTask VoidTask { get; }

    internal PendingOperation(bool asynchronous, Action<PendingOperation<T>, Exception> cancel)
    {
        _cancel = cancel;
        _source.RunContinuationsAsynchronously = asynchronous;
        _source.OnFinally(static state => ((PendingOperation<T>)state!).Release(), this);
        Task = new(_source, _source.Version);
        VoidTask = new(_source, _source.Version);
    }

    internal void SetupCancellation(CancellationToken token, TimeSpan timeout, TimeProvider? provider = null)
    {
        _token = token;
        _timeout = timeout;

        if (Volatile.Read(ref _terminal) != 0)
        {
            return;
        }

        _cancellation = token.UnsafeRegister(static state =>
        {
            var self = (PendingOperation<T>)state!;
            self._cancel(self, new OperationCanceledException(self._token));
        }, this);

        if (timeout != Timeout.InfiniteTimeSpan && Volatile.Read(ref _terminal) == 0)
        {
            _timeoutSource = new CancellationTokenSource(timeout, provider ?? TimeProvider.System);
            _timeoutRegistration = _timeoutSource.Token.UnsafeRegister(static state =>
            {
                var self = (PendingOperation<T>)state!;
                self._cancel(self, new TimeoutException($"ROS service request timed out after {self._timeout}."));
            }, this);
        }
    }

    internal void FinishSetup() => Release();

    internal bool Succeed(T result) => Complete(result, null);

    internal bool Fail(Exception error, bool asynchronous = false) => Complete(default!, error, asynchronous);

    private bool Complete(T result, Exception? error, bool asynchronous = false)
    {
        if (Interlocked.CompareExchange(ref _terminal, 1, 0) != 0)
        {
            return false;
        }

        Interlocked.Increment(ref _references);

        try
        {
            if (asynchronous)
            {
                _source.RunContinuationsAsynchronously = true;
            }

            if (error is null)
            {
                _source.SetResult(result);
            }
            else
            {
                _source.SetException(error);
            }

            return true;
        }
        finally
        {
            Release();
        }
    }

    private void Release()
    {
        if (Interlocked.Decrement(ref _references) != 0)
        {
            return;
        }

        _cancellation.Dispose();
        _timeoutRegistration.Dispose();
        _timeoutSource?.Dispose();
        _source.Reset();
        ObjectPool.Return(_source);
    }
}
