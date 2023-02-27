using System.Runtime.CompilerServices;

namespace Rcl.Logging;

/// <summary>
/// Helpers methods for writing logs.
/// </summary>
public static class LoggingExtensions
{
    /// <summary>
    /// Writes a debug log.
    /// </summary>
    /// <param name="logger">The <see cref="IRclLogger"/> to write to.</param>
    /// <param name="message">The content of the log.</param>
    /// <param name="file">The full file name of the caller.</param>
    /// <param name="functionName">The function name of the caller.</param>
    /// <param name="lineNumber">The line number of the caller.</param>
    public static void LogDebug(this IRclLogger logger, string? message,
        [CallerFilePath] string file = "", [CallerMemberName] string functionName = "", [CallerLineNumber] int lineNumber = 0)
    {
        logger.Log(LogSeverity.Debug, message, file, functionName, lineNumber);
    }

    /// <summary>
    /// Writes an information log.
    /// </summary>
    /// <param name="logger">The <see cref="IRclLogger"/> to write to.</param>
    /// <param name="message">The content of the log.</param>
    /// <param name="file">The full file name of the caller.</param>
    /// <param name="functionName">The function name of the caller.</param>
    /// <param name="lineNumber">The line number of the caller.</param>
    public static void LogInformation(this IRclLogger logger, string? message,
        [CallerFilePath] string file = "", [CallerMemberName] string functionName = "", [CallerLineNumber] int lineNumber = 0)
    {
        logger.Log(LogSeverity.Information, message, file, functionName, lineNumber);
    }

    /// <summary>
    /// Writes an warning log.
    /// </summary>
    /// <param name="logger">The <see cref="IRclLogger"/> to write to.</param>
    /// <param name="message">The content of the log.</param>
    /// <param name="file">The full file name of the caller.</param>
    /// <param name="functionName">The function name of the caller.</param>
    /// <param name="lineNumber">The line number of the caller.</param>
    public static void LogWarning(this IRclLogger logger, string? message,
        [CallerFilePath] string file = "", [CallerMemberName] string functionName = "", [CallerLineNumber] int lineNumber = 0)
    {
        logger.Log(LogSeverity.Warning, message, file, functionName, lineNumber);
    }

    /// <summary>
    /// Writes an error log.
    /// </summary>
    /// <param name="logger">The <see cref="IRclLogger"/> to write to.</param>
    /// <param name="message">The content of the log.</param>
    /// <param name="file">The full file name of the caller.</param>
    /// <param name="functionName">The function name of the caller.</param>
    /// <param name="lineNumber">The line number of the caller.</param>
    public static void LogError(this IRclLogger logger, string? message,
        [CallerFilePath] string file = "", [CallerMemberName] string functionName = "", [CallerLineNumber] int lineNumber = 0)
    {
        logger.Log(LogSeverity.Debug, message, file, functionName, lineNumber);
    }

    /// <summary>
    /// Writes a fatal log.
    /// </summary>
    /// <param name="logger">The <see cref="IRclLogger"/> to write to.</param>
    /// <param name="message">The content of the log.</param>
    /// <param name="file">The full file name of the caller.</param>
    /// <param name="functionName">The function name of the caller.</param>
    /// <param name="lineNumber">The line number of the caller.</param>
    public static void LogFatal(this IRclLogger logger, string? message,
        [CallerFilePath] string file = "", [CallerMemberName] string functionName = "", [CallerLineNumber] int lineNumber = 0)
    {
        logger.Log(LogSeverity.Fatal, message, file, functionName, lineNumber);
    }
}

