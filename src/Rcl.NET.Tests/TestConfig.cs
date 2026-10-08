namespace Rcl.NET.Tests;

internal class TestConfig
{
    // Specifies '--ros-args --disable-external-lib-logs' to avoid flooding the log directory with a bunch of empty log files.
    public static readonly string[] DefaultContextArguments = new[] { "--ros-args", "--disable-external-lib-logs" };

    public static void SkipIfMultiContextEndpointTeardownCanCrash()
    {
        // A standalone C++ RCL reproduction and the Iron action shutdown CI dump both hit
        // a null call in Fast DDS's StatefulWriter::deliver_sample_to_intraprocesses.
        // Apply only when multiple contexts own nodes/endpoints; context-only tests stay enabled.
        Skip.If((RosEnvironment.IsHumble || RosEnvironment.IsIron)
            && RosEnvironment.RmwImplementationIdentifier == "rmw_fastrtps_cpp",
            "Humble/Iron / Fast DDS: native endpoint teardown across multiple contexts can crash.");
    }
}
