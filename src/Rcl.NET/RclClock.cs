namespace Rcl;

/// <summary>
/// Represents an abstraction over various kind of clocks.
/// </summary>
public sealed class RclClock : IRclClock
{
    private readonly RclClockImpl _impl;
    private readonly bool _shared;

    /// <summary>
    /// Create a new <see cref="RclClock"/> with specific type.
    /// </summary>
    /// <param name="context"><see cref="RclContext"/> to attach the clock.</param>
    /// <param name="type">Type of the clock to create.</param>
    public RclClock(RclClockType type)
        : this(type, false)
    {

    }

    private RclClock(RclClockType type, bool shared)
    {
        _impl = new(type);
        _shared = shared;
    }

    /// <summary>
    /// Gets the default steady clock which can be shared across <see cref="RclContext"/>s.
    /// </summary>
    public static RclClock SteadyClock { get; } = new RclClock(RclClockType.Steady, true);

    /// <summary>
    /// Gets the default system clock which can be shared across <see cref="RclContext"/>s.
    /// </summary>
    public static RclClock SystemClock { get; } = new RclClock(RclClockType.System, true);

    internal RclClockImpl Impl => _impl;

    /// <inheritdoc/>
    public TimeSpan Elapsed => _impl.Elapsed;

    /// <inheritdoc/>
    public DateTimeOffset Now => _impl.Now;

    /// <summary>
    /// Gets the type of the clock.
    /// </summary>
    public RclClockType Type => _impl.Type;

    /// <inheritdoc/>
    public void Dispose()
    {
        if (_shared)
        {
            throw new InvalidOperationException("Cannot dispose shared RclClock instance.");
        }

        _impl.Dispose();
    }
}