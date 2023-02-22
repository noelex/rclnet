using System.Runtime.CompilerServices;

namespace Rcl.Logging;

public static class LoggingExtensions
{
    public static void LogDebug(this IRclLogger logger, string message,
        [CallerFilePath] string file = "", [CallerMemberName] string functionName = "", [CallerLineNumber] int lineNumber = 0)
    {
        logger.Log(LogSeverity.Debug, message, file, functionName, lineNumber);
    }

    public static void LogInformation(this IRclLogger logger, string message,
        [CallerFilePath] string file = "", [CallerMemberName] string functionName = "", [CallerLineNumber] int lineNumber = 0)
    {
        logger.Log(LogSeverity.Information, message, file, functionName, lineNumber);
    }

    public static void LogWarning(this IRclLogger logger, string message,
        [CallerFilePath] string file = "", [CallerMemberName] string functionName = "", [CallerLineNumber] int lineNumber = 0)
    {
        logger.Log(LogSeverity.Warning, message, file, functionName, lineNumber);
    }

    public static void LogError(this IRclLogger logger, string message,
        [CallerFilePath] string file = "", [CallerMemberName] string functionName = "", [CallerLineNumber] int lineNumber = 0)
    {
        logger.Log(LogSeverity.Debug, message, file, functionName, lineNumber);
    }

    public static void LogFatal(this IRclLogger logger, string message,
        [CallerFilePath] string file = "", [CallerMemberName] string functionName = "", [CallerLineNumber] int lineNumber = 0)
    {
        logger.Log(LogSeverity.Fatal, message, file, functionName, lineNumber);
    }
}

