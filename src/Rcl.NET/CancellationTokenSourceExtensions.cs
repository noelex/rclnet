using Rcl.Logging;
using Rcl.SafeHandles;

namespace Rcl;

/// <summary>
/// Represents a registration of timeout event on an <see cref="RclContext"/>.
/// </summary>
/// <remarks>
/// <see cref="Dispose"/> must be called if the registration is no long needed.
/// It's the caller's responsibility the make sure that the registration
/// is disposed exactly once.
/// </remarks>
public readonly struct TimeoutRegistration : IDisposable
{
    private readonly ObjectPool<ReusableTimer> _pool;
    private readonly ReusableTimer _timer;

    internal TimeoutRegistration(ObjectPool<ReusableTimer> pool, ReusableTimer timer)
    {
        _pool = pool;
        _timer = timer;
    }

    /// <summary>
    /// Represents an empty registration, which does nothing when disposed.
    /// </summary>
    public static readonly TimeoutRegistration Empty = new();

    /// <inheritdoc/>
    public void Dispose()
    {
        if (_pool != null)
        {
            _timer.Reset();
            _pool.Return(_timer);
        }
    }
}

unsafe class ReusableTimer : IDisposable
{
    private RclContext? _context;
    private SafeTimerHandle? _handle;
    private WaitHandleRegistration _registration;

    public void Start(CancellationTokenSource cts,
        RclContext context, RclClockImpl clock, TimeSpan period)
    {
        _context = context;
        _handle = new SafeTimerHandle(context.Handle, clock.Handle, (long)period.TotalNanoseconds);
        _registration = context.Register(_handle, OnWaitCompleted, cts);

        context.DefaultLogger.LogDebug($"Started new ReusableTimer {_handle.DangerousGetHandle()} with period {period}.");
    }

    public void Reset()
    {
        var ctx = Interlocked.Exchange(ref _context, null);
        if (ctx != null)
        {
            ctx.DefaultLogger.LogDebug($"Released ReusableTimer {_handle!.DangerousGetHandle()}.");

            _registration.Dispose();
            _registration = WaitHandleRegistration.Empty;

            ctx.SynchronizationContext.Post(static s => ((IDisposable)s!).Dispose(), _handle);
            _handle = null;
        }
    }

    private static unsafe void OnWaitCompleted(RclObjectHandle handle, object? state)
    {
        rcl_timer_cancel(((SafeTimerHandle)handle).Object);
        ((CancellationTokenSource)state!).Cancel();
    }

    public void Dispose()
    {
        Reset();
    }
}

/// <summary>
/// Define helper methods for canceling <see cref="CancellationTokenSource"/>s with <see cref="RclClock"/>.
/// </summary>
public static class CancellationTokenSourceExtensions
{
    private const string ReusableTimerPoolFeature = nameof(ReusableTimerPoolFeature);

    /// <summary>
    /// Cancel the <see cref="CancellationTokenSource"/> after specific period of time, measured with <see cref="RclClock"/> specified by <paramref name="clock"/>.
    /// </summary>
    /// <param name="source">The <see cref="CancellationTokenSource"/> to be canceled when the delay expires.</param>
    /// <param name="timeout">
    /// The countdown for the delay starts during this call.  When the delay expires,
    /// this <see cref="CancellationTokenSource"/> is canceled, if it has
    /// not been canceled already.
    /// <para>
    /// If <see cref="Timeout.InfiniteTimeSpan"/> is specified, the <see cref="CancellationTokenSource"/> will never be canceled.
    /// </para>
    /// </param>
    /// <param name="clock">
    /// An <see cref="RclClock"/> used for measuring the countdown time.
    /// </param>
    /// <param name="context">
    /// The <see cref="RclContext"/> for registering the wait operation.
    /// </param>
    /// <returns>
    /// A <see cref="TimeoutRegistration"/> for unregistering the operation from the <see cref="RclContext"/>.
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException"></exception>
    public static TimeoutRegistration CancelAfter(
        this CancellationTokenSource source, TimeSpan timeout, RclClock clock, RclContext context)
    {
        if (timeout < Timeout.InfiniteTimeSpan)
        {
            throw new ArgumentOutOfRangeException(nameof(timeout), "Specified timeout value is out of range.");
        }

        if (timeout == Timeout.InfiniteTimeSpan)
        {
            return TimeoutRegistration.Empty;
        }

        if (timeout == TimeSpan.Zero)
        {
            source.Cancel();
            return TimeoutRegistration.Empty;
        }

        var pool = context.GetOrAddFeature<ObjectPool<ReusableTimer>>(ReusableTimerPoolFeature, x => new());
        var timer = pool.Rent();
        timer.Start(source, context, clock.Impl, timeout);

        return new TimeoutRegistration(pool, timer);
    }

    /// <summary>
    /// Cancel the <see cref="CancellationTokenSource"/> after specific period of time, measured with <see cref="RclClock"/> specified by <paramref name="clock"/>.
    /// </summary>
    /// <param name="source">The <see cref="CancellationTokenSource"/> to be canceled when the delay expires.</param>
    /// <param name="timeoutMilliseconds">
    /// The countdown for the delay starts during this call.  When the delay expires,
    /// this <see cref="CancellationTokenSource"/> is canceled, if it has
    /// not been canceled already.
    /// <para>
    /// If <see cref="Timeout.Infinite"/> is specified, the <see cref="CancellationTokenSource"/> will never be canceled.
    /// </para>
    /// </param>
    /// <param name="clock"></param>
    /// <param name="context"></param>
    /// <returns>
    /// A <see cref="TimeoutRegistration"/> for unregistering the operation from the <see cref="RclContext"/>.
    /// </returns>
    public static TimeoutRegistration CancelAfter(this CancellationTokenSource source, int timeoutMilliseconds, RclClock clock, RclContext context)
        => source.CancelAfter(TimeSpan.FromMilliseconds(timeoutMilliseconds), clock, context);

    /// <summary>
    /// Cancel the <see cref="CancellationTokenSource"/> after specific period of time, measured with <see cref="IRclNode.Clock"/>.
    /// </summary>
    /// <param name="source">The <see cref="CancellationTokenSource"/> to be canceled when the delay expires.</param>
    /// <param name="timeoutMilliseconds">
    /// The countdown for the delay starts during this call.  When the delay expires,
    /// this <see cref="CancellationTokenSource"/> is canceled, if it has
    /// not been canceled already.
    /// <para>
    /// If <see cref="Timeout.Infinite"/> is specified, the <see cref="CancellationTokenSource"/> will never be canceled.
    /// </para>
    /// </param>
    /// <param name="node">
    /// An <see cref="IRclNode"/> which provides the clock for measuring the countdown time.
    /// </param>
    /// <returns>
    /// A <see cref="TimeoutRegistration"/> for unregistering the operation from the <see cref="RclContext"/>.
    /// </returns>
    public static TimeoutRegistration CancelAfter(this CancellationTokenSource source, int timeoutMilliseconds, IRclNode node)
        => source.CancelAfter(timeoutMilliseconds, node.Clock, node.Context);

    /// <summary>
    /// Cancel the <see cref="CancellationTokenSource"/> after specific period of time, measured with <see cref="IRclNode.Clock"/>.
    /// </summary>
    /// <param name="source">The <see cref="CancellationTokenSource"/> to be canceled when the delay expires.</param>
    /// <param name="timeout">
    /// The countdown for the delay starts during this call.  When the delay expires,
    /// this <see cref="CancellationTokenSource"/> is canceled, if it has
    /// not been canceled already.
    /// <para>
    /// If <see cref="Timeout.InfiniteTimeSpan"/> is specified, the <see cref="CancellationTokenSource"/> will never be canceled.
    /// </para>
    /// </param>
    /// <param name="node">An <see cref="IRclNode"/> which provides the clock for measuring the countdown time.</param>
    /// <returns>
    /// A <see cref="TimeoutRegistration"/> for unregistering the operation from the <see cref="RclContext"/>.
    /// </returns>
    public static TimeoutRegistration CancelAfter(this CancellationTokenSource source, TimeSpan timeout, IRclNode node)
        => source.CancelAfter(timeout, node.Clock, node.Context);
}
