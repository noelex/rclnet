using Rcl.SafeHandles;

namespace Rcl;

public static class CancellationTokenSourceExtensions
{
    public static IDisposable CancelAfter(this CancellationTokenSource source, TimeSpan timeout, RclClock clock, RclContext context)
    {
        if (clock.Type == RclClockType.Steady)
        {
            source.CancelAfter(timeout);
            return NoopTimer.Default;
        }

        if (timeout < Timeout.InfiniteTimeSpan)
        {
            throw new ArgumentOutOfRangeException(nameof(timeout), "Specified timeout value is out of range.");
        }

        // TODO: Make CancellationTokenSourceTimer reusable.
        return timeout == Timeout.InfiniteTimeSpan
              ? NoopTimer.Default
              : new CancellationTokenSourceTimer(source, context, clock.Impl, timeout);
    }

    public static IDisposable CancelAfter(this CancellationTokenSource source, int timeoutMilliseconds, RclClock clock, RclContext context)
        => source.CancelAfter(TimeSpan.FromMilliseconds(timeoutMilliseconds), clock, context);

    /// <summary>
    /// Cancel the <see cref="CancellationTokenSource"/> after specific period of time, measured with <see cref="IRclNode.Clock"/>.
    /// </summary>
    /// <param name="source"></param>
    /// <param name="timeoutMilliseconds"></param>
    /// <param name="node"></param>
    /// <returns></returns>
    public static IDisposable CancelAfter(this CancellationTokenSource source, int timeoutMilliseconds, IRclNode node)
        => source.CancelAfter(timeoutMilliseconds, node.Clock, node.Context);

    /// <summary>
    /// Cancel the <see cref="CancellationTokenSource"/> after specific period of time, measured with <see cref="IRclNode.Clock"/>.
    /// </summary>
    /// <param name="source"></param>
    /// <param name="timeout"></param>
    /// <param name="node"></param>
    /// <returns></returns>
    public static IDisposable CancelAfter(this CancellationTokenSource source, TimeSpan timeout, IRclNode node)
        => source.CancelAfter(timeout, node.Clock, node.Context);

    private unsafe class CancellationTokenSourceTimer : RclObject<SafeTimerHandle>
    {
        private readonly CancellationTokenSource _cts;
        private readonly WaitHandleRegistration _registration;

        public CancellationTokenSourceTimer(
            CancellationTokenSource cts, RclContext context, RclClockImpl clock, TimeSpan period)
            : base(new SafeTimerHandle(context.Handle, clock.Handle, (long)period.TotalNanoseconds))
        {
            _cts = cts;
            _registration = context.Register(Handle, OnWaitCompleted, this);
        }

        public override void Dispose()
        {
            _registration.Dispose();
            base.Dispose();
        }

        private static void OnWaitCompleted(object? state)
        {
            var self = (CancellationTokenSourceTimer)state!;
            if (!self._cts.IsCancellationRequested)
            {
                self._cts.Cancel();
            }
        }
    }

    private class NoopTimer : IDisposable
    {
        public static NoopTimer Default { get; } = new();

        public void Dispose() { }
    }
}
