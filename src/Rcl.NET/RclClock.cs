namespace Rcl;

/// <summary>
/// Represents an abstraction over various kind of clocks.
/// </summary>
/// <remarks>
/// <see cref="RclClock"/>s created by <see cref="IRclNode"/>s (usually <see cref="RclClockType.Ros"/> clocks) are thread-safe within the <see cref="RclContext"/> of the node.
/// <see cref="SteadyClock"/> and <see cref="SystemClock"/> are globally thread-safe.
/// <para>
/// Manually and <see cref="IRclNode"/> created <see cref="RclClock"/>s can also be shared across multiple <see cref="RclContext"/>s, with 
/// the limitation that the shared clock instance must only be disposed after all other referring <see cref="RclContext"/>s are completely
/// shutdown (e.g. by awaiting <see cref="RclContext.DisposeAsync"/>).
/// </para>
/// </remarks>
public sealed class RclClock : IRclClock
{
    private readonly RclClockImpl _impl;
    private readonly bool _shared;

    /// <summary>
    /// Create a new <see cref="RclClock"/> with specific type.
    /// </summary>
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
            throw new InvalidOperationException("Cannot dispose a shared RclClock instance.");
        }

        _impl.Dispose();
    }
}