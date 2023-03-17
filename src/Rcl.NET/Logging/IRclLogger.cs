namespace Rcl.Logging;

/// <summary>
/// Represents a type used to perform logging in rclnet.
/// </summary>
public interface IRclLogger
{
    /// <summary>
    /// Gets the name of the logger.
    /// </summary>
    string Name { get; }

    /// <summary>
    /// Writes a log message.
    /// </summary>
    /// <param name="severity">Severity level of the log message.</param>
    /// <param name="message">The content of the log.</param>
    /// <param name="file">The full file name of the caller.</param>
    /// <param name="functionName">The function name of the caller.</param>
    /// <param name="lineNumber">The line number of the caller.</param>
    void Log(LogSeverity severity, string? message, string file = "", string functionName = "", int lineNumber = 0);
}