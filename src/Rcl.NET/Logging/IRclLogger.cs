using System.Runtime.CompilerServices;

namespace Rcl.Logging;

public interface IRclLogger
{
    string Name { get; }

    void Log(LogSeverity severity, string message, string file = "", string functionName = "", int lineNumber = 0);
}