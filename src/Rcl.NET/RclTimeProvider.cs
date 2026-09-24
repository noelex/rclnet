using Rcl.SafeHandles;

namespace Rcl;

/// <summary>
/// Adapts an RCL clock and context to the .NET time APIs.
/// </summary>
/// <remarks>
/// The context and clock must outlive this provider. Timers use the clock for scheduling,
/// and their callbacks run on the thread pool rather than the RCL event loop.
/// ROS time can pause or move backwards, so elapsed time measured with this provider can do the same.
/// Cancel pending waits before disposing their node or context.
/// </remarks>
public sealed class RclTimeProvider : TimeProvider, IDisposable
{
    private readonly RclContext _context;
    private readonly RclClock _clock;
    private readonly object _gate = new();
    private readonly HashSet<RclTimeProviderTimer> _timers = new();
    private bool _disposed;

    /// <summary>
    /// Creates a time provider using the specified context and clock.
    /// </summary>
    /// <param name="context">The context that schedules timers.</param>
    /// <param name="clock">The clock that measures timer delays.</param>
    public RclTimeProvider(IRclContext context, IRclClock clock)
    {
        _context = context as RclContext
            ?? throw new NotSupportedException("RclTimeProvider supports only RclContext.");
        _clock = clock as RclClock
            ?? throw new NotSupportedException("RclTimeProvider supports only RclClock.");
    }

    /// <summary>Gets the clock's current UTC time.</summary>
    /// <remarks>For a steady clock, returns system UTC because steady time has no calendar epoch.</remarks>
    public override DateTimeOffset GetUtcNow()
        => _clock.Type == RclClockType.Steady ? TimeProvider.System.GetUtcNow() : _clock.Now;

    /// <inheritdoc/>
    public override long GetTimestamp() => _clock.Impl.Nanoseconds;

    /// <inheritdoc/>
    public override long TimestampFrequency => 1_000_000_000;

    /// <inheritdoc/>
    public override ITimer CreateTimer(TimerCallback callback, object? state, TimeSpan dueTime, TimeSpan period)
    {
        ArgumentNullException.ThrowIfNull(callback);
        RclTimeProviderTimer timer;

        lock (_gate)
        {
            ObjectDisposedException.ThrowIf(_disposed, this);
            timer = new RclTimeProviderTimer(this, _context, _clock, callback, state, dueTime, period);

            try
            {
                _timers.Add(timer);
            }
            catch
            {
                timer.Dispose();
                throw;
            }
        }

        return timer;
    }

    internal void Remove(RclTimeProviderTimer timer)
    {
        lock (_gate)
        {
            _timers.Remove(timer);
        }
    }

    /// <summary>
    /// Stops timers created by this provider. The context and clock remain owned by their callers.
    /// </summary>
    /// <remarks>Stopping a timer does not complete a pending .NET delay or wait.</remarks>
    public void Dispose()
    {
        RclTimeProviderTimer[] timers;

        lock (_gate)
        {
            if (_disposed)
            {
                return;
            }

            _disposed = true;
            timers = _timers.ToArray();
        }

        foreach (var timer in timers)
        {
            timer.Dispose();
        }
    }
}

internal sealed class RclTimeProviderTimer : ITimer
{
    private readonly RclTimeProvider _owner;
    private readonly RclContext _context;
    private readonly SafeTimerHandle _handle;
    private readonly TimerCallback _callback;
    private readonly object? _state;
    private readonly ExecutionContext? _executionContext;
    private readonly object _gate = new();
    private WaitHandleRegistration _registration;
    private TaskCompletionSource? _disposeCompletion;
    private TimeSpan _period;
    private bool _scheduled;
    private bool _firstTick;
    private bool _disposed;
    private bool _handleReleased;
    private int _pendingCallbacks;

    internal RclTimeProviderTimer(RclTimeProvider owner, RclContext context, RclClock clock,
        TimerCallback callback, object? state, TimeSpan dueTime, TimeSpan period)
    {
        Validate(dueTime, nameof(dueTime));
        Validate(period, nameof(period));

        _owner = owner;
        _context = context;
        _callback = callback;
        _state = state;
        _executionContext = ExecutionContext.Capture();
        _handle = new SafeTimerHandle(context.Handle, clock.Impl.Handle, 1_000_000);

        try
        {
            unsafe
            {
                using var lease = _handle.Acquire();
                RclException.ThrowIfNonSuccess(rcl_timer_cancel(lease.Object));
            }

            context.Register(_handle, static (_, state) => ((RclTimeProviderTimer)state!).OnTimer(), this, ref _registration);
            Change(dueTime, period);
        }
        catch
        {
            lock (context.RegistrationGate)
            {
                _handle.TryBeginClose();
            }

            _registration.Dispose();
            context.AfterDetach(_registration, static state => ((SafeTimerHandle)state!).Dispose(), _handle);
            throw;
        }
    }

    private static void Validate(TimeSpan value, string name)
    {
        if (value != Timeout.InfiniteTimeSpan)
        {
            ArgumentOutOfRangeException.ThrowIfLessThan(value, TimeSpan.Zero, name);
            ArgumentOutOfRangeException.ThrowIfGreaterThan(value.TotalMilliseconds, uint.MaxValue - 1, name);
        }
    }

    public unsafe bool Change(TimeSpan dueTime, TimeSpan period)
    {
        Validate(dueTime, nameof(dueTime));
        Validate(period, nameof(period));

        lock (_gate)
        {
            if (_disposed)
            {
                return false;
            }

            using var lease = _handle.Acquire();
            _period = period;
            _firstTick = true;
            _scheduled = dueTime != Timeout.InfiniteTimeSpan;

            if (_scheduled)
            {
                long previous;
                RclException.ThrowIfNonSuccess(rcl_timer_exchange_period(
                    lease.Object, dueTime.Ticks * 100, &previous));
                RclException.ThrowIfNonSuccess(rcl_timer_reset(lease.Object));
            }
            else
            {
                RclException.ThrowIfNonSuccess(rcl_timer_cancel(lease.Object));
            }
        }

        _context.NotifyTimerChanged();
        return true;
    }

    private unsafe void OnTimer()
    {
        lock (_gate)
        {
            if (_disposed || !_scheduled)
            {
                return;
            }

            using var lease = _handle.Acquire();
            bool ready;
            RclException.ThrowIfNonSuccess(rcl_timer_is_ready(lease.Object, &ready));

            if (!ready)
            {
                return;
            }

            RclException.ThrowIfNonSuccess(rcl_timer_call(lease.Object));

            if (_period == TimeSpan.Zero || _period == Timeout.InfiniteTimeSpan)
            {
                _scheduled = false;
                RclException.ThrowIfNonSuccess(rcl_timer_cancel(lease.Object));
            }
            else if (_firstTick)
            {
                // The native timer has one period; .NET timers have a distinct first delay.
                long previous;
                RclException.ThrowIfNonSuccess(rcl_timer_exchange_period(
                    lease.Object, _period.Ticks * 100, &previous));
                RclException.ThrowIfNonSuccess(rcl_timer_reset(lease.Object));
                _firstTick = false;
            }

            _pendingCallbacks++;
        }

        ThreadPool.UnsafeQueueUserWorkItem(static timer => timer.InvokeCallback(), this, preferLocal: false);
    }

    private void InvokeCallback()
    {
        try
        {
            lock (_gate)
            {
                if (_disposed || _context.Handle.IsClosing)
                {
                    return;
                }
            }

            if (_executionContext is null)
            {
                _callback(_state);
            }
            else
            {
                ExecutionContext.Run(_executionContext,
                    static state => ((RclTimeProviderTimer)state!).InvokeUserCallback(), this);
            }
        }
        finally
        {
            lock (_gate)
            {
                if (--_pendingCallbacks == 0 && _handleReleased)
                {
                    _disposeCompletion?.TrySetResult();
                }
            }
        }
    }

    private void InvokeUserCallback() => _callback(_state);

    public void Dispose()
    {
        lock (_gate)
        {
            if (_disposed)
            {
                return;
            }

            _disposed = true;

            lock (_context.RegistrationGate)
            {
                _handle.TryBeginClose();
            }
        }

        _registration.Dispose();
        _context.AfterDetach(_registration, static state => ((RclTimeProviderTimer)state!).ReleaseHandle(), this);
        _owner.Remove(this);
    }

    private void ReleaseHandle()
    {
        _handle.Dispose();

        lock (_gate)
        {
            _handleReleased = true;

            if (_pendingCallbacks == 0)
            {
                _disposeCompletion?.TrySetResult();
            }
        }
    }

    public ValueTask DisposeAsync()
    {
        Dispose();

        lock (_gate)
        {
            if (_pendingCallbacks == 0 && _handleReleased)
            {
                return ValueTask.CompletedTask;
            }

            _disposeCompletion ??= new(TaskCreationOptions.RunContinuationsAsynchronously);
            return new ValueTask(_disposeCompletion.Task);
        }
    }
}
