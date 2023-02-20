namespace Rcl;

/// <summary>
/// ROS 2 distributions.
/// </summary>
public static class RosDistribution
{
    /// <summary>
    /// ROS 2 Foxy Fitzroy.
    /// </summary>
    public const string Foxy = "foxy";

    /// <summary>
    /// ROS 2 Humble Hawksbill.
    /// </summary>
    public const string Humble = "humble";

    public static bool IsFoxy => RclContext.RosDistro == Foxy;

    public static bool IsHumble => RclContext.RosDistro == Humble;
}
