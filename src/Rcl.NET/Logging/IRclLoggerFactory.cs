namespace Rcl.Logging;

/// <summary>
/// Represents a type used to configure the logging system and create instances of <see cref="IRclLogger"/>.
/// </summary>
public interface IRclLoggerFactory
{
    /// <summary>
    /// Creates a logger with specific name.
    /// </summary>
    /// <param name="name">The name of the logger.</param>
    /// <returns>A <see cref="IRclLogger"/> instance for writing logs.</returns>
    IRclLogger CreateLogger(string name);
}