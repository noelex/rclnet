namespace Rcl;

/// <summary>
/// Represents an abstraction over various kind of clocks.
/// </summary>
public sealed class RclClock : IRclClock
{
    private readonly RclClockImpl _impl;

    /// <summary>
    /// Create a new <see cref="RclClock"/> with specific type.
    /// </summary>
    /// <param name="context"><see cref="RclContext"/> to attach the clock.</param>
    /// <param name="type">Type of the clock to create.</param>
    internal RclClock(RclContext context, RclClockType type)
    {
        _impl = new(context, type);
    }

    internal RclClockImpl Impl => _impl;

    public RclContext Context => _impl.Context;

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
        _impl.Dispose();
    }
}