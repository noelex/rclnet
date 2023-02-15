using Rcl.SafeHandles;

namespace Rcl.Internal;

internal abstract class RclWaitObject<T> : RclObject<T>, IRclWaitObject where T : RclObjectHandle
{
    private SpinLock _syncRoot = new();
    private readonly WaitHandleRegistration _registration;
    private readonly Dictionary<int, ManualResetValueTaskSource<bool>> _awaiters = new();
    private readonly List<ManualResetValueTaskSource<bool>> _awaiterSnapshot = new();
    private readonly CancellationTokenSource _shutdownSignal = new();

    private int _id, _disposed;
    protected bool IsDisposed => _disposed != 0;

    protected RclWaitObject(RclContext context, T handle) : base(handle)
    {
        _registration = context.Register(this, OnSignalReceived, this);
    }

    protected virtual void OnWaitCompleted()
    {

    }

    private static void OnSignalReceived(object? state)
    {
        var self = (RclWaitObject<T>)state!;

        if (Volatile.Read(ref self._disposed) == 1)
        {
            return;
        }

        self.OnWaitCompleted();

        bool success = false;
        try
        {
            self._syncRoot.Enter(ref success);

            // Snapshot awaiters to prevent prematurely completing "future" awaiters
            // which will be registered in synchronous continuation.
            foreach (var (id, v) in self._awaiters)
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
        finally
        {
            if (success) self._syncRoot.Exit();
        }

        foreach (var awaiter in self._awaiterSnapshot)
        {
            awaiter.SetResult(true);
        }
        self._awaiterSnapshot.Clear();
    }

    private void AddAwaiter(int token, ManualResetValueTaskSource<bool> item)
    {
        bool success = false;
        try
        {
            _syncRoot.Enter(ref success);
            _awaiters[token] = item;
        }
        finally
        {
            if (success) _syncRoot.Exit();
        }
    }

    private bool RemoveAwaiter(int token)
    {
        bool success = false;
        try
        {
            _syncRoot.Enter(ref success);
            return _awaiters.Remove(token);
        }
        finally
        {
            if (success) _syncRoot.Exit();
        }
    }

    public ValueTask WaitOneAsync(bool runContinuationAsynchronously, CancellationToken cancellationToken = default)
    {
        if (Volatile.Read(ref _disposed) == 1)
        {
            throw new ObjectDisposedException(nameof(RclGuardConditionImpl));
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
            ((RclWaitObject<T>.CompletedCallbackArgs)state!).Return(), (object)completionArg);

        AddAwaiter(id, tcs);
        return new ValueTask(tcs, tcs.Version);
    }

    public ValueTask WaitOneAsync(CancellationToken cancellationToken = default)
        => WaitOneAsync(false, cancellationToken);

    public override void Dispose()
    {
        if (Interlocked.CompareExchange(ref _disposed, 1, 0) == 0)
        {
            _registration.Dispose();
            base.Dispose();

            bool success = false;
            Dictionary<int, ManualResetValueTaskSource<bool>> snapshot;

            try
            {
                _syncRoot.Enter(ref success);
                snapshot = new(_awaiters);
                _awaiters.Clear();
            }
            finally
            {
                if (success) _syncRoot.Exit();
            }

            foreach (var (id, v) in snapshot)
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
