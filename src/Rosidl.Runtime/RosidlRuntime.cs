namespace Rosidl.Runtime;

/// <summary>
/// Provides information about the ROSIDL runtime used by the current process.
/// </summary>
public static class RosidlRuntime
{
    private static readonly RosidlNativeAbi s_nativeAbi =
        RosidlAbiResolver.Resolve(Environment.GetEnvironmentVariable("ROS_DISTRO"));

    /// <summary>
    /// Gets the native ROSIDL message ABI for the ROS distribution selected by
    /// the <c>ROS_DISTRO</c> environment variable.
    /// </summary>
    /// <remarks>
    /// The environment variable is read when <see cref="RosidlRuntime"/> is initialized.
    /// </remarks>
    /// <exception cref="TypeInitializationException">
    /// Initialization failed because the selected ROS distribution is missing or unsupported.
    /// </exception>
    public static RosidlNativeAbi NativeAbi => s_nativeAbi;

    /// <summary>
    /// Verifies that the current ROS distribution uses the expected native ROSIDL message ABI.
    /// </summary>
    /// <param name="expected">The native ABI required by the caller.</param>
    /// <exception cref="TypeInitializationException">
    /// Initialization failed because the selected ROS distribution is missing or unsupported.
    /// </exception>
    /// <exception cref="InvalidOperationException">
    /// The current native ROSIDL message ABI does not match <paramref name="expected"/>.
    /// </exception>
    public static void RequireNativeAbi(RosidlNativeAbi expected)
    {
        var actual = NativeAbi;
        if (actual != expected)
        {
            throw new InvalidOperationException(
                $"The current ROSIDL native ABI is '{actual}', but '{expected}' is required.");
        }
    }
}

internal static class RosidlAbiResolver
{
    private const string SupportedDistributions = "foxy, humble, iron, jazzy, kilted, lyrical";

    public static RosidlNativeAbi Resolve(string? distro)
    {
        return distro switch
        {
            "foxy" or "humble" or "iron" or "jazzy" or "kilted" => RosidlNativeAbi.V1,
            "lyrical" => RosidlNativeAbi.V2,
            _ => throw new NotSupportedException(
                $"ROS distribution '{FormatDistribution(distro)}' is not supported. " +
                $"Supported distributions: {SupportedDistributions}.")
        };
    }

    private static string FormatDistribution(string? distro) => distro switch
    {
        null => "<unset>",
        "" => "<empty>",
        _ => distro
    };
}
