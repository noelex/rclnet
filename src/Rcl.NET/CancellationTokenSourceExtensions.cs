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
[Obsolete("Use TimeProvider-aware APIs such as new CancellationTokenSource(timeout, node.TimeProvider); no separate timeout registration is needed.")]
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
        var handle = new SafeTimerHandle(context.Handle, clock.Handle, (long)period.TotalNanoseconds);
        _context = context;
        _handle = handle;

        try
        {
            context.Register(handle, OnWaitCompleted, cts, ref _registration);
            context.DefaultLogger.LogDebug($"Started new ReusableTimer {handle.DangerousGetHandle()} with period {period}.");
        }
        catch
        {
            Reset();
            throw;
        }
    }

    public void Reset()
    {
        var ctx = Interlocked.Exchange(ref _context, null);

        if (ctx != null)
        {
            var handle = _handle!;

            lock (ctx.RegistrationGate)
            {
                handle.TryBeginClose();
            }

            var registration = _registration;
            _registration = WaitHandleRegistration.Empty;
            _handle = null;

            try
            {
                ctx.DefaultLogger.LogDebug($"Released ReusableTimer {handle.DangerousGetHandle()}.");
            }
            finally
            {
                registration.Dispose();
                ctx.AfterDetach(registration, static s => ((IDisposable)s!).Dispose(), handle);
            }
        }
    }

    private static unsafe void OnWaitCompleted(RclObjectHandle handle, object? state)
    {
        using var lease = ((SafeTimerHandle)handle).Acquire();
        rcl_timer_cancel(lease.Object);
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
    /// Cancel the <see cref="CancellationTokenSource"/> after specific period of time, measured with <see cref="IRclClock"/> specified by <paramref name="clock"/>.
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
    /// An <see cref="IRclClock"/> used for measuring the countdown time.
    /// </param>
    /// <param name="context">
    /// The <see cref="IRclContext"/> for registering the wait operation.
    /// </param>
    /// <returns>
    /// A <see cref="TimeoutRegistration"/> for unregistering the operation from the <see cref="IRclContext"/>.
    /// </returns>
    /// <exception cref="ArgumentOutOfRangeException"></exception>
    [Obsolete("Use new CancellationTokenSource(timeout, node.TimeProvider) or a TimeProvider-aware wait API instead.")]
    public static TimeoutRegistration CancelAfter(
        this CancellationTokenSource source, TimeSpan timeout, IRclClock clock, IRclContext context)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(timeout, Timeout.InfiniteTimeSpan);

        if (timeout == Timeout.InfiniteTimeSpan)
        {
            return TimeoutRegistration.Empty;
        }

        if (timeout == TimeSpan.Zero)
        {
            source.Cancel();
            return TimeoutRegistration.Empty;
        }

        if (clock is not RclClock rclClock || context is not RclContext rclContext)
        {
            throw new NotSupportedException("CancelAfter supports only RclClock and RclContext.");
        }

        var pool = rclContext.GetOrAddFeature<ObjectPool<ReusableTimer>>(ReusableTimerPoolFeature, x => new());
        var timer = pool.Rent();

        try
        {
            timer.Start(source, rclContext, rclClock.Impl, timeout);
        }
        catch
        {
            pool.Return(timer);
            throw;
        }

        return new TimeoutRegistration(pool, timer);
    }

    /// <summary>
    /// Cancel the <see cref="CancellationTokenSource"/> after specific period of time, measured with <see cref="IRclClock"/> specified by <paramref name="clock"/>.
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
    /// A <see cref="TimeoutRegistration"/> for unregistering the operation from the <see cref="IRclContext"/>.
    /// </returns>
    [Obsolete("Use new CancellationTokenSource(timeout, node.TimeProvider) or a TimeProvider-aware wait API instead.")]
    public static TimeoutRegistration CancelAfter(this CancellationTokenSource source, int timeoutMilliseconds, IRclClock clock, IRclContext context)
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
    /// A <see cref="TimeoutRegistration"/> for unregistering the operation from the <see cref="IRclContext"/>.
    /// </returns>
    [Obsolete("Use new CancellationTokenSource(timeout, node.TimeProvider) or a TimeProvider-aware wait API instead.")]
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
    /// A <see cref="TimeoutRegistration"/> for unregistering the operation from the <see cref="IRclContext"/>.
    /// </returns>
    [Obsolete("Use new CancellationTokenSource(timeout, node.TimeProvider) or a TimeProvider-aware wait API instead.")]
    public static TimeoutRegistration CancelAfter(this CancellationTokenSource source, TimeSpan timeout, IRclNode node)
        => source.CancelAfter(timeout, node.Clock, node.Context);
}
