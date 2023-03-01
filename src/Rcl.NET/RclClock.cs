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
    /// <param name="type">Type of the clock to create.</param>
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

    /// <inheritdoc/>
    public TimeSpan Elapsed => _impl.Elapsed;

    /// <inheritdoc/>
    public DateTimeOffset Now => _impl.Now;

    /// <summary>
    /// Get an <see cref="RclClock"/> representing an <see cref="RclClockType.Steady"/> clock.
    /// </summary>
    /// <remarks>
    /// This is a shared clock, diposing it will cause an <see cref="InvalidOperationException"/>.
    /// </remarks>
    public static RclClock Steady { get; } = new RclClock(RclClockType.Steady, true);

    /// <summary>
    /// Get an <see cref="RclClock"/> representing an <see cref="RclClockType.System"/> clock.
    /// </summary>
    /// <remarks>
    /// This is a shared clock, diposing it will cause an <see cref="InvalidOperationException"/>.
    /// </remarks>
    public static RclClock System { get; } = new RclClock(RclClockType.System, true);

    /// <summary>
    /// Gets the type of the clock.
    /// </summary>
    public RclClockType Type => _impl.Type;

    /// <summary>
    /// Checks whether current <see cref="RclClock"/> is a shared instance.
    /// </summary>
    public bool IsShared => _shared;

    /// <inheritdoc/>
    public void Dispose()
    {
        if (_shared)
        {
            throw new InvalidOperationException("Shared RclClock object cannot be disposed.");
        }

        _impl.Dispose();
    }
}