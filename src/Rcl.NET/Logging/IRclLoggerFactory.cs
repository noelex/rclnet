namespace Rcl.Logging;

public interface IRclLoggerFactory
{
    IRclLogger CreateLogger(string name);
}