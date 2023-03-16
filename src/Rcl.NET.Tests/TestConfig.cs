namespace Rcl.NET.Tests;

internal class TestConfig
{
    /// <summary>
    /// Specifies '--ros-args --disable-external-lib-logs' to avoid flooding the log directory with a bunch of empty log files.
    /// </summary>
    public static readonly string[] DefaultContextArguments = new[] { "--ros-args", "--disable-external-lib-logs" };

    public static readonly bool GitHubActions = Environment.GetEnvironmentVariable("GITHUB_ACTIONS") == "true";
}
