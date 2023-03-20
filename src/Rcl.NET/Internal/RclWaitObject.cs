using Rcl.SafeHandles;

namespace Rcl.Internal;

internal abstract class RclWaitObject<T> : RclContextualObject<T>, IRclWaitObject where T : RclObjectHandle
{
    private SpinLock _syncRoot = new();
    private WaitHandleRegistration _registration;

    private readonly Dictionary<int, ManualResetValueTaskSource<bool>> _awaiters = new();
    private readonly List<ManualResetValueTaskSource<bool>> _awaiterSnapshot = new();

    private int _id, _disposed;

    protected bool IsDisposed => _disposed != 0;

    protected RclWaitObject(RclContext context, T handle) : base(context, handle)
    {

    }

    /// <summary>
    /// Register the wait handle in RclContext.
    /// </summary>
    /// <remarks>
    /// This method must be called AFTER the implementation is ready to receive events.
    /// </remarks>
    protected void RegisterWaitHandle()
    {
        _registration = Context.Register(this, OnSignalReceived, this);
    }

    protected virtual void OnWaitCompleted()
    {

    }

    private static void OnSignalReceived(RclObjectHandle handle, object? state)
    {
        var self = (RclWaitObject<T>)state!;

        if (Volatile.Read(ref self._disposed) == 1)
        {
            return;
        }

        self.OnWaitCompleted();

        using (ScopedLock.Lock(ref self._syncRoot))
        {
            // Snapshot awaiters to prevent prematurely completing "future" awaiters
            // which will be registered in synchronous continuation.
            foreach (var (_, v) in self._awaiters)
            {
                self._awaiterSnapshot.Add(v);
            }

            // In case RunContinuationsAsynchronously = true,
            // callbacks registered with OnCompletedInternal are executed asynchronously,
            // which means the awaiter is not guaranteed to be removed before next trigger.
            // So we remove the awaiter here to avoid calling SetResult twice on the same
            // ValueTaskSource.
            self._awaiters.Clear();
        }

        foreach (var awaiter in self._awaiterSnapshot)
        {
            awaiter.SetResult(true);
        }
        self._awaiterSnapshot.Clear();
    }

    private void AddAwaiter(int token, ManualResetValueTaskSource<bool> item)
    {
        using (ScopedLock.Lock(ref _syncRoot))
        {
            _awaiters[token] = item;
        }
    }

    private bool RemoveAwaiter(int token)
    {
        using (ScopedLock.Lock(ref _syncRoot))
        {
            return _awaiters.Remove(token);
        }
    }

    public ValueTask WaitOneAsync(bool runContinuationAsynchronously, CancellationToken cancellationToken = default)
    {
        if (Volatile.Read(ref _disposed) == 1)
        {
            throw new ObjectDisposedException(GetType().Name);
        }

        // TODO: Maybe use private ObjectPools?
        var id = Interlocked.Increment(ref _id);
        var tcs = ObjectPool.Rent<ManualResetValueTaskSource<bool>>();
        tcs.RunContinuationsAsynchronously = runContinuationAsynchronously;

        var cancellationArgs = ObjectPool.Rent<CancellationCallbackArgs>()
            .Reset(this, id, tcs, cancellationToken);
        var reg = cancellationToken.Register(
            static state => ((CancellationCallbackArgs)state!).TryCancel(), cancellationArgs);

        var completionArg =
            ObjectPool.Rent<CompletedCallbackArgs>()
            .Reset(this, reg, id, tcs, cancellationArgs);
        tcs.OnFinally(static state =>
            ((RclWaitObject<T>.CompletedCallbackArgs)state!).Return(), completionArg);

        AddAwaiter(id, tcs);
        return new ValueTask(tcs, tcs.Version);
    }

    public ValueTask WaitOneAsync(CancellationToken cancellationToken = default)
        => WaitOneAsync(true, cancellationToken);

    public override void Dispose()
    {
        if (Interlocked.CompareExchange(ref _disposed, 1, 0) == 0)
        {
            _registration.Dispose();
            base.Dispose();

            Dictionary<int, ManualResetValueTaskSource<bool>> snapshot;
            using (ScopedLock.Lock(ref _syncRoot))
            {
                snapshot = new(_awaiters);
                _awaiters.Clear();
            }

            foreach (var (_, v) in snapshot)
            {
                v.SetException(new ObjectDisposedException(GetType().Name));
            }
        }
    }

    private class CancellationCallbackArgs
    {
        public RclWaitObject<T> This { get; private set; } = null!;
        public int Id { get; private set; }
        public ManualResetValueTaskSource<bool> Completion { get; private set; } = null!;
        public CancellationToken Cancellation { get; private set; }

        public CancellationCallbackArgs Reset(
            RclWaitObject<T> self,
            int id,
            ManualResetValueTaskSource<bool> completion,
            CancellationToken cancellationToken)
        {
            This = self;
            Id = id;
            Completion = completion;
            Cancellation = cancellationToken;
            return this;
        }

        public void TryCancel()
        {
            if (This.RemoveAwaiter(Id))
            {
                Completion.SetException(new OperationCanceledException(Cancellation));
            }
        }

        public void Return()
        {
            This = default!;
            Id = default;
            Completion = default!;
            Cancellation = default;

            ObjectPool.Return(this);
        }
    }

    private class CompletedCallbackArgs
    {
        public RclWaitObject<T> This { get; private set; } = null!;
        public CancellationTokenRegistration Registration { get; private set; }
        public int Id { get; private set; }
        public ManualResetValueTaskSource<bool> Completion { get; private set; } = null!;

        public CancellationCallbackArgs CancellationArgs { get; private set; } = null!;

        public CompletedCallbackArgs Reset(
            RclWaitObject<T> self,
            CancellationTokenRegistration reg,
            int id,
            ManualResetValueTaskSource<bool> completion,
            CancellationCallbackArgs cancellationArgs)
        {
            This = self;
            Registration = reg;
            Id = id;
            Completion = completion;
            CancellationArgs = cancellationArgs;
            return this;
        }

        public void Return()
        {
            Registration.Dispose();
            Completion.Reset();

            CancellationArgs.Return();

            ObjectPool.Return(Completion);
            ObjectPool.Return(this);
        }
    }
}
