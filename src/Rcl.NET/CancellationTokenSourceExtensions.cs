using Rcl.Logging;
using Rcl.SafeHandles;

namespace Rcl;

public readonly struct RosTimerCallbackRegistration : IDisposable
{
    private readonly ObjectPool<ReusableTimer> _pool;
    private readonly ReusableTimer _timer;

    internal RosTimerCallbackRegistration(ObjectPool<ReusableTimer> pool, ReusableTimer timer)
    {
        _pool= pool;
        _timer= timer;
    }

    public static readonly RosTimerCallbackRegistration Empty = new();

    public void Dispose()
    {
        if (_pool != null)
        {
            _timer.Reset();
            _pool.Return(_timer);
        }
        
    }
}

unsafe class ReusableTimer:IDisposable
{
    private RclContext? _context;
    private SafeTimerHandle? _handle;
    private CancellationTokenSource? _cts;
    private WaitHandleRegistration _registration;

    public void Start(CancellationTokenSource cts,
        RclContext context, RclClockImpl clock, TimeSpan period)
    {
        _context = context;
        _handle = new SafeTimerHandle(context.Handle, clock.Handle, (long)period.TotalNanoseconds);
        _cts = cts;
        _registration = context.Register(_handle, OnWaitCompleted, this);

        context.DefaultLogger.LogDebug($"Started new ReusableTimer {_handle.DangerousGetHandle()} with period {period}.");
    }

    public void Reset()
    {
        if (!_registration.IsEmpty)
        {
            _registration.Dispose();
            _registration = WaitHandleRegistration.Empty;
            _handle!.Dispose();

            _context?.DefaultLogger.LogDebug($"Released ReusableTimer {_handle.DangerousGetHandle()}.");

            _cts = null;
            _handle = null;
            _context = null;
        }
    }

    private static void OnWaitCompleted(object? state)
    {
        var self = (ReusableTimer)state!;
        self._cts?.Cancel();
    }

    public void Dispose()
    {
        Reset();
    }
}

public static class CancellationTokenSourceExtensions
{
    private const string ReusableTimerPoolFeature = nameof(ReusableTimerPoolFeature);

    public static RosTimerCallbackRegistration CancelAfter(
        this CancellationTokenSource source, TimeSpan timeout, RclClock clock, RclContext context)
    {
        if (clock.Type == RclClockType.Steady)
        {
            source.CancelAfter(timeout);
            return RosTimerCallbackRegistration.Empty;
        }

        if (timeout < Timeout.InfiniteTimeSpan)
        {
            throw new ArgumentOutOfRangeException(nameof(timeout), "Specified timeout value is out of range.");
        }

        if (timeout == Timeout.InfiniteTimeSpan)
        {
            return RosTimerCallbackRegistration.Empty;
        }

        var pool = context.GetOrAddFeature<ObjectPool<ReusableTimer>>(ReusableTimerPoolFeature, x => new());
        var timer = pool.Rent();
        timer.Start(source, context, clock.Impl, timeout);

        return new RosTimerCallbackRegistration(pool, timer);
    }

    public static RosTimerCallbackRegistration CancelAfter(this CancellationTokenSource source, int timeoutMilliseconds, RclClock clock, RclContext context)
        => source.CancelAfter(TimeSpan.FromMilliseconds(timeoutMilliseconds), clock, context);

    /// <summary>
    /// Cancel the <see cref="CancellationTokenSource"/> after specific period of time, measured with <see cref="IRclNode.Clock"/>.
    /// </summary>
    /// <param name="source"></param>
    /// <param name="timeoutMilliseconds"></param>
    /// <param name="node"></param>
    /// <returns></returns>
    public static RosTimerCallbackRegistration CancelAfter(this CancellationTokenSource source, int timeoutMilliseconds, IRclNode node)
        => source.CancelAfter(timeoutMilliseconds, node.Clock, node.Context);

    /// <summary>
    /// Cancel the <see cref="CancellationTokenSource"/> after specific period of time, measured with <see cref="IRclNode.Clock"/>.
    /// </summary>
    /// <param name="source"></param>
    /// <param name="timeout"></param>
    /// <param name="node"></param>
    /// <returns></returns>
    public static RosTimerCallbackRegistration CancelAfter(this CancellationTokenSource source, TimeSpan timeout, IRclNode node)
        => source.CancelAfter(timeout, node.Clock, node.Context);

    private class NoopTimer : IDisposable
    {
        public static NoopTimer Default { get; } = new();

        public void Dispose() { }
    }
}
