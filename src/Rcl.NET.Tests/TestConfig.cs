namespace Rcl.NET.Tests;

internal class TestConfig
{
    // Specifies '--ros-args --disable-external-lib-logs' to avoid flooding the log directory with a bunch of empty log files.
    public static readonly string[] DefaultContextArguments = new[] { "--ros-args", "--disable-external-lib-logs" };
}
