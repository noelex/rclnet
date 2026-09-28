using Rcl.SafeHandles;
using System.Threading.Tasks.Sources;

namespace Rcl.Utils;

// Tables and queues own references which transfer to the producer when removed.
// Reuse waits for setup, consumption, all producers, and registered callbacks.
internal sealed class PendingOperation<T> : IValueTaskSource<T>, IValueTaskSource
{
    private ManualResetValueTaskSourceCore<T> _source;
    private Action<PendingOperation<T>, Exception> _cancel = null!;
    private CancellationTokenRegistration _cancellation;
    private ITimer? _timeoutTimer;
    private CancellationToken _token;
    private TimeSpan _timeout;
    private int _references;
    private int _terminal;
    private int _consumed;

    internal long Key { get; set; }

    internal bool IsCompleted => Volatile.Read(ref _terminal) != 0;

    internal ValueTask<T> Task => new(this, _source.Version);

    internal ValueTask VoidTask => new(this, _source.Version);

    public PendingOperation()
    {
    }

    internal static PendingOperation<T> Rent(bool asynchronous, Action<PendingOperation<T>, Exception> cancel)
    {
        var operation = ObjectPool.Rent<PendingOperation<T>>();
        operation._cancel = cancel;
        operation._source.RunContinuationsAsynchronously = asynchronous;
        operation._references = 2; // setup + consumer
        operation._terminal = 0;
        operation._consumed = 0;
        return operation;
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
            if (!self.TryAddReference())
            {
                return;
            }

            try
            {
                self._cancel(self, new OperationCanceledException(self._token));
            }
            finally
            {
                self.Release();
            }
        }, this);

        if (timeout != Timeout.InfiniteTimeSpan && Volatile.Read(ref _terminal) == 0)
        {
            _timeoutTimer = (provider ?? TimeProvider.System).CreateTimer(static state =>
            {
                var self = (PendingOperation<T>)state!;
                if (!self.TryAddReference())
                {
                    return;
                }

                try
                {
                    self._cancel(self, new TimeoutException($"ROS service request timed out after {self._timeout}."));
                }
                finally
                {
                    self.Release();
                }
            }, this, timeout, Timeout.InfiniteTimeSpan);
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

    internal void AddReference() => Interlocked.Increment(ref _references);

    private bool TryAddReference()
    {
        var references = Volatile.Read(ref _references);

        while (references != 0)
        {
            var previous = Interlocked.CompareExchange(ref _references, references + 1, references);
            if (previous == references)
            {
                return true;
            }

            references = previous;
        }

        // Cleanup already owns the operation and will drain this callback before reuse.
        return false;
    }

    internal void Release()
    {
        if (Interlocked.Decrement(ref _references) != 0)
        {
            return;
        }

        _ = RecycleAsync();
    }

    private async System.Threading.Tasks.Task RecycleAsync()
    {
        try
        {
            // Awaiting also handles cleanup initiated from inside a cancellation callback.
            await _cancellation.DisposeAsync().ConfigureAwait(false);

            if (_timeoutTimer is RclTimeProviderTimer timer)
            {
                await timer.DisposeCallbacksAsync().ConfigureAwait(false);
            }
            else if (_timeoutTimer != null)
            {
                await _timeoutTimer.DisposeAsync().ConfigureAwait(false);
            }

            _cancellation = default;
            _timeoutTimer = null;
            _cancel = null!;
            _token = default;
            _timeout = default;
            Key = 0;
            _source.Reset();
            ObjectPool.Return(this);
        }
        catch (Exception error)
        {
            // A failed cleanup cannot safely return callback state to the pool.
            HandleReleaseDiagnostics.Record(new(GetType().Name, "pending operation cleanup", null, error.ToString()));
        }
    }

    T IValueTaskSource<T>.GetResult(short token) => GetResult(token);

    void IValueTaskSource.GetResult(short token) => GetResult(token);

    private T GetResult(short token)
    {
        // Validate before releasing the consumer reference, including faulted operations.
        if (_source.GetStatus(token) == ValueTaskSourceStatus.Pending)
        {
            throw new InvalidOperationException("The operation has not completed.");
        }

        try
        {
            return _source.GetResult(token);
        }
        finally
        {
            if (Interlocked.Exchange(ref _consumed, 1) == 0)
            {
                Release();
            }
        }
    }

    public ValueTaskSourceStatus GetStatus(short token) => _source.GetStatus(token);

    public void OnCompleted(Action<object?> continuation, object? state, short token, ValueTaskSourceOnCompletedFlags flags)
        => _source.OnCompleted(continuation, state, token, flags);
}
