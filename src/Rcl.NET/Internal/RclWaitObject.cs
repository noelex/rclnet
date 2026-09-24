using Rcl.SafeHandles;

namespace Rcl.Internal;

internal abstract class RclWaitObject<T> : RclContextualObject<T>, IRclWaitObject where T : RclObjectHandle
{
    private readonly object _pendingGate = new();
    private readonly Action<PendingOperation<bool>, Exception> _cancelPending;
    private WaitHandleRegistration _registration;
    private readonly Dictionary<long, PendingOperation<bool>> _awaiters = new();
    private readonly List<PendingOperation<bool>> _awaiterSnapshot = new();
    private long _id;
    private int _stopped, _detached;

    protected bool IsDisposed => Volatile.Read(ref _stopped) != 0;

    protected RclWaitObject(RclContext context, T handle) : base(context, handle) => _cancelPending = Cancel;

    // Publish only after the derived implementation is ready for callbacks.
    protected void RegisterWaitHandle()
    {
        try
        {
            Context.Register(Handle, OnSignalReceived, this, ref _registration,
                static state => ((RclWaitObject<T>)state!).Stop(),
                static state => ((RclWaitObject<T>)state!).Detached());
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    protected virtual void OnWaitCompleted()
    {
    }

    protected virtual void OnStopped()
    {
    }

    protected virtual void OnDetached()
    {
    }

    private void Detached()
    {
        if (Interlocked.Exchange(ref _detached, 1) == 0)
        {
            OnDetached();
        }
    }

    private static void OnSignalReceived(RclObjectHandle handle, object? state)
    {
        var self = (RclWaitObject<T>)state!;

        if (self.IsDisposed)
        {
            return;
        }

        self.OnWaitCompleted();

        lock (self._pendingGate)
        {
            self._awaiterSnapshot.AddRange(self._awaiters.Values);
            self._awaiters.Clear();
        }

        try
        {
            foreach (var pending in self._awaiterSnapshot)
            {
                pending.Succeed(true);
            }
        }
        finally
        {
            self._awaiterSnapshot.Clear();
        }
    }

    private void Cancel(PendingOperation<bool> pending, Exception error)
    {
        lock (_pendingGate)
        {
            if (!_awaiters.TryGetValue(pending.Key, out var current) || !ReferenceEquals(current, pending))
            {
                return;
            }

            _awaiters.Remove(pending.Key);
        }

        pending.Fail(error);
    }

    public ValueTask WaitOneAsync(bool runContinuationAsynchronously, CancellationToken cancellationToken = default)
    {
        Handle.ThrowIfOperationClosed();
        var pending = new PendingOperation<bool>(runContinuationAsynchronously, _cancelPending)
        {
            Key = Interlocked.Increment(ref _id)
        };
        bool published = false;

        try
        {
            lock (_pendingGate)
            {
                Handle.ThrowIfOperationClosed();
                ObjectDisposedException.ThrowIf(IsDisposed, this);
                _awaiters.Add(pending.Key, pending);
                published = true;
            }

            pending.SetupCancellation(cancellationToken, Timeout.InfiniteTimeSpan);
        }
        catch (Exception error)
        {
            if (published)
            {
                Cancel(pending, error);
            }
            else
            {
                pending.Fail(error);
            }
        }
        finally
        {
            pending.FinishSetup();
        }

        return pending.VoidTask;
    }

    public ValueTask WaitOneAsync(CancellationToken cancellationToken = default)
        => WaitOneAsync(true, cancellationToken);

    private void Stop()
    {
        PendingOperation<bool>[] snapshot;

        lock (_pendingGate)
        {
            if (_stopped != 0)
            {
                return;
            }

            Volatile.Write(ref _stopped, 1);
            snapshot = _awaiters.Values.ToArray();
            _awaiters.Clear();
        }

        try
        {
            foreach (var pending in snapshot)
            {
                pending.Fail(new ObjectDisposedException(GetType().Name), asynchronous: true);
            }
        }
        finally
        {
            OnStopped();
        }
    }

    protected override void DisposeCore()
    {
        Cleanup.Run(static state => ((RclWaitObject<T>)state!).Stop(), this);
        _registration.Dispose();

        if (_registration.IsEmpty)
        {
            Context.ScheduleCleanup(static state => ((RclWaitObject<T>)state!).Detached(), this);
        }

        base.DisposeCore();
    }
}
