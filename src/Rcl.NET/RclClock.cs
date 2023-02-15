namespace Rcl;

public sealed class RclClock : IRclClock
{
    private readonly RclClockImpl _impl;
    private readonly bool _shared;

    public RclClock(RclClockType type)
        :this(type, false)
    {

    }

    private RclClock(RclClockType type, bool shared)
    {
        _impl = new(type);
       _shared = shared;
    }

    internal RclClockImpl Impl => _impl;

    public TimeSpan Elapsed => _impl.Elapsed;

    public DateTimeOffset Now => _impl.Now;

    public static RclClock Steady { get; } = new RclClock(RclClockType.Steady, true);

    public static RclClock Ros { get; } = new RclClock(RclClockType.Ros, true);

    public static RclClock System { get; } = new RclClock(RclClockType.System, true);

    public void Dispose()
    {
        if (_shared)
        {
            throw new InvalidOperationException("Shared RclClock object cannot be disposed.");
        }

        _impl.Dispose();
    }
}