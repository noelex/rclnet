namespace Rcl.Logging;

/// <summary>
/// Represents the severity levels of log messages / loggers.
/// </summary>
public enum LogSeverity
{
    /// <summary>
    /// Not used for writing log messages.
    /// Specifies that the level of the log message of logger is not set.
    /// </summary>
    Unknown = 0,

    /// <summary>
    /// Logs that are used for interactive investigation during development.
    /// These logs should primarily contain information useful for debugging and have no long-term value.
    /// </summary>
    Debug = 10,

    /// <summary>
    /// Logs that track the general flow of the application. These logs should have long-term value.
    /// </summary>
    Information = 20,

    /// <summary>
    /// Logs that highlight an abnormal or unexpected event in the application flow,
    /// but do not otherwise cause the application execution to stop.
    /// </summary>
    Warning = 30,

    /// <summary>
    /// Logs that highlight when the current flow of execution is stopped due to a failure.
    /// These should indicate a failure in the current activity, not an application-wide failure.
    /// </summary>
    Error = 40,

    /// <summary>
    /// Logs that describe an unrecoverable application or system crash, or a catastrophic failure that requires immediate attention.
    /// </summary>
    Fatal = 50
}
