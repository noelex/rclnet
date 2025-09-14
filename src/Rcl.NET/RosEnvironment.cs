using Rosidl.Runtime.Interop;

namespace Rcl;

internal enum VersionRequirement
{
    Exact,
    SinceInclusive,
    UntilExclusive
}

/// <summary>
/// Helper class for accessing ROS 2 environment information.
/// </summary>
public unsafe static class RosEnvironment
{
    private static readonly string[] SupportedDistributions = new[] { Foxy, Humble, Iron, Jazzy };

    /// <summary>
    /// ROS 2 Foxy Fitzroy.
    /// </summary>
    public const string Foxy = "foxy";

    /// <summary>
    /// ROS 2 Humble Hawksbill.
    /// </summary>
    public const string Humble = "humble";

    /// <summary>
    /// ROS 2 Iron Irwini.
    /// </summary>
    public const string Iron = "iron";
    /// <summary>
    /// ROS 2 Jazzy.
    /// </summary>
    public const string Jazzy = "jazzy";

    /// <summary>
    /// Gets whether the application is running in foxy.
    /// </summary>
    public static bool IsFoxy => Distribution == Foxy;

    /// <summary>
    /// Gets whether the application is running in humble.
    /// </summary>
    public static bool IsHumble => Distribution == Humble;

    /// <summary>
    /// Gets whether the application is running in iron.
    /// </summary>
    public static bool IsIron => Distribution == Iron;
    /// <summary>
    /// Gets whether the application is running in iron.
    /// </summary>
    public static bool IsJazzy => Distribution == Jazzy;

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

    private static void ThrowUnsupportedDistribution(string? featureName = null)
    {
        featureName = featureName == null ? "The specific feature" : $"'{featureName}' feature";
        throw new NotSupportedException($"{featureName} is not supported by ROS distribution '{Distribution}'.");
    }

    internal static bool IsSupported(string targetDistro, VersionRequirement requirement = VersionRequirement.SinceInclusive)
    {
        var v = Array.IndexOf(SupportedDistributions, Distribution);
        if (v < 0)
        {
            return false;
        }

        var t = Array.IndexOf(SupportedDistributions, targetDistro);
        if (t < 0)
        {
            throw new ArgumentException("Specified target distribution is not supported.", nameof(targetDistro));
        }

        return requirement switch
        {
            VersionRequirement.SinceInclusive => v >= t,
            VersionRequirement.UntilExclusive => v < t,
            VersionRequirement.Exact => v == t,
            _ => false,
        };
    }

    internal static void Require(string targetDistro, VersionRequirement requirement = VersionRequirement.SinceInclusive, string? feature = null)
    {
        if (!IsSupported(targetDistro, requirement))
        {
            ThrowUnsupportedDistribution(feature);
        }
    }
}
