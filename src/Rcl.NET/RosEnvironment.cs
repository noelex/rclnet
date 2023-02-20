using Rosidl.Runtime.Interop;

namespace Rcl;

/// <summary>
/// Helper class for accessing ROS 2 environment information.
/// </summary>
public unsafe static class RosEnvironment
{
    /// <summary>
    /// ROS 2 Foxy Fitzroy.
    /// </summary>
    public const string Foxy = "foxy";

    /// <summary>
    /// ROS 2 Humble Hawksbill.
    /// </summary>
    public const string Humble = "humble";

    /// <summary>
    /// Gets whether the application is running in foxy.
    /// </summary>
    public static bool IsFoxy => Distribution == Foxy;

    /// <summary>
    /// Gets whether the application is running in humble.
    /// </summary>
    public static bool IsHumble => Distribution == Humble;

    /// <summary>
    /// Get the name of the rmw implementation being used.
    /// </summary>
    public static string RmwImplementationIdentifier => StringMarshal.CreatePooledString(rmw_get_implementation_identifier())!;

    /// <summary>
    /// Get the unique serialization format for current middleware.
    /// </summary>
    public static string RmwSerializationFormat => StringMarshal.CreatePooledString(rmw_get_serialization_format())!;

    /// <summary>
    /// Get the name of the ROS distribution currently in use.
    /// </summary>
    public static string Distribution { get; } = Environment.GetEnvironmentVariable("ROS_DISTRO") ?? "";
}
