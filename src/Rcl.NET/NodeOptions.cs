using Rcl.Qos;
using Rcl.Runtime;

namespace Rcl;

/// <summary>
/// Encapsulation of options for node initialization.
/// </summary>
public record NodeOptions
{
    /// <summary>
    /// Create a new <see cref="NodeOptions"/>.
    /// </summary>
    /// <param name="arguments">
    /// Arguments used to extract remappings used by the node and other ROS specific settings, as well as user defined non-ROS arguments.
    /// <para>
    /// Defaults to [].
    /// </para>
    /// </param>
    /// <param name="domaindId">
    /// If set, then this value overrides the ROS_DOMAIN_ID environment variable.
    /// <para>
    /// If unset, the node will use the ROS domain ID set in the ROS_DOMAIN_ID environment
    /// variable, or on some systems 0 if the environment variable is not set.
    /// </para>
    /// <para>
    /// Supported distro(s): =foxy
    /// </para>
    /// </param>
    /// <param name="useGlobalArguments">
    /// If <see langword="true"/> this will cause the node's behavior to be influenced by "global"
    /// arguments, i.e.arguments not targeted at specific nodes, as well as the
    /// arguments targeted at the current node.
    /// <para>
    /// Defaults to <see langword="true"/>.
    /// </para>
    /// </param>
    /// <param name="enableRosOut">
    /// If <see langword="false"/> this will cause the node not to use rosout logging.
    /// <para>
    /// Defaults to <see langword="true"/>.
    /// </para>
    /// </param>
    /// <param name="clock">
    /// <see cref="RclClock"/> to be used by the node.
    /// <para>
    /// Defaults to <see cref="RclClock.Ros"/>.
    /// </para>
    /// </param>
    /// <param name="clockQos">
    /// The QoS settings to be used for the subscriber on /clock topic, if enabled.
    /// <para>
    /// Defaults to <see cref="QosProfile.Clock"/>.
    /// </para>
    /// </param>
    /// <param name="rosOutQos">
    /// The QoS settings to be used for the publisher on /rosout topic, if enabled.
    /// <para>
    /// Defaults to <see cref="QosProfile.RosOut"/>.
    /// </para>
    /// <para>
    /// Supported distro(s): >=humble
    /// </para>
    /// </param>
    public NodeOptions(
        string[]? arguments = null,
        [SupportedDistribution(RosEnvironment.Foxy)]
        nuint? domaindId = null,
        bool useGlobalArguments = true,
        bool enableRosOut = true,
        RclClock? clock = null,
        QosProfile? clockQos = null,
        [SupportedSinceDistribution(RosEnvironment.Humble)]
        QosProfile? rosOutQos = null)
    {
        if (domaindId != null)
        {
            RosEnvironment.Require(RosEnvironment.Foxy, VersionRequirement.Exact, nameof(DomaindId));
        }

        if (rosOutQos != null)
        {
            RosEnvironment.Require(RosEnvironment.Humble, feature: nameof(RosOutQos));
        }

        Arguments = arguments ?? Array.Empty<string>();
        DomaindId = domaindId;
        UseGlobalArguments = useGlobalArguments;
        EnableRosOut = enableRosOut;
        Clock = clock ?? RclClock.Ros;
        ClockQos = clockQos ?? QosProfile.Clock;
        RosOutQos = rosOutQos ?? QosProfile.RosOut;
    }

    /// <summary>
    /// Arguments used to extract remappings used by the node and other ROS specific settings, as well as user defined non-ROS arguments.
    /// <para>
    /// Defaults to [].
    /// </para>
    /// </summary>
    public string[] Arguments { get; }

    /// <summary>
    /// If set, then this value overrides the ROS_DOMAIN_ID environment variable.
    /// <para>
    /// If unset, the node will use the ROS domain ID set in the ROS_DOMAIN_ID environment
    /// variable, or on some systems 0 if the environment variable is not set.
    /// </para>
    /// <para>
    /// Supported by: =foxy
    /// </para>
    /// </summary>
    [SupportedDistribution(RosEnvironment.Foxy)]
    public nuint? DomaindId { get; }

    /// <summary>
    /// If <see langword="true"/> this will cause the node's behavior to be influenced by "global"
    /// arguments, i.e.arguments not targeted at specific nodes, as well as the
    /// arguments targeted at the current node.
    /// <para>
    /// Defaults to <see langword="true"/>.
    /// </para>
    /// </summary>
    public bool UseGlobalArguments { get; }

    /// <summary>
    /// If <see langword="false"/> this will cause the node not to use rosout logging.
    /// <para>
    /// Defaults to <see langword="true"/>.
    /// </para>
    /// </summary>
    public bool EnableRosOut { get; }

    /// <summary>
    /// The QoS settings to be used for the publisher on /rosout topic, if enabled.
    /// <para>
    /// Defaults to <see cref="QosProfile.RosOut"/>.
    /// </para>
    /// </summary>
    /// <remarks>
    /// Supported distro(s): >=humble
    /// </remarks>
    [SupportedSinceDistribution(RosEnvironment.Humble)]
    public QosProfile RosOutQos { get; }

    /// <summary>
    /// <see cref="RclClock"/> to be used by the node.
    /// <para>
    /// Defaults to <see cref="RclClock.Ros"/>.
    /// </para>
    /// </summary>
    public RclClock Clock { get; }

    /// <summary>
    /// The QoS settings to be used for the subscriber on /clock topic, if enabled.
    /// <para>
    /// Defaults to <see cref="QosProfile.Clock"/>.
    /// </para>
    /// </summary>
    public QosProfile ClockQos { get; }

    /// <summary>
    /// Gets a <see cref="NodeOptions"/> instance with all options set to default values.
    /// </summary>
    public static NodeOptions Default { get; } = new();
}