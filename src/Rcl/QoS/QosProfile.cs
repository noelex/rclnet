using Rcl.Interop;

namespace Rcl.Qos;

/// <summary>
/// ROS quality of service profile.
/// </summary>
/// <param name="History">History QoS policy setting.</param>
/// <param name="Depth">Size of the message queue.</param>
/// <param name="Reliability">Reliability QoS policy setting.</param>
/// <param name="Durability">Durability QoS policy setting.</param>
/// <param name="Deadline">The period at which messages are expected to be sent/received.</param>
/// <param name="LifeSpan">The age at which messages are considered expired and no longer valid.</param>
/// <param name="Liveliness">Liveliness QoS policy setting。</param>
/// <param name="LivelinessLeaseDuration">The time within which the node or publisher must show that it is alive</param>
/// <param name="AvoidRosNamespaceConversions">
/// If <see langword="true"/>, any ROS specific namespacing conventions will be circumvented.
/// <p>
/// In the case of DDS and topics, for example, this means the typical
/// ROS specific prefix of `rt` would not be applied as described here:
/// http://design.ros2.org/articles/topic_and_service_names.html#ros-specific-namespace-prefix
/// </p>
/// <p>
/// This might be useful when trying to directly connect a native DDS topic with a ROS 2 topic.
/// </p>
/// </param>
public record QosProfile(
    HistoryPolicy History = HistoryPolicy.Default,
    int Depth = 0,
    ReliabilityPolicy Reliability = ReliabilityPolicy.Default,
    DurabilityPolicy Durability = DurabilityPolicy.Default,
    TimeSpan Deadline = default,
    TimeSpan LifeSpan = default,
    LivelinessPolicy Liveliness = LivelinessPolicy.Default,
    TimeSpan LivelinessLeaseDuration = default,
    bool AvoidRosNamespaceConversions =false
)
{
    public static QosProfile SensorData { get; } =
        new(
            HistoryPolicy.KeepLast,
            5,
            ReliabilityPolicy.BestEffort,
            DurabilityPolicy.Volatile,
            TimeSpan.Zero,
            TimeSpan.Zero,
            LivelinessPolicy.Default,
            TimeSpan.Zero,
            false
        );

    public static QosProfile Parameters { get; } =
        new(
            HistoryPolicy.KeepLast,
            1000,
            ReliabilityPolicy.Reliable,
            DurabilityPolicy.Volatile,
            TimeSpan.Zero,
            TimeSpan.Zero,
            LivelinessPolicy.Default,
            TimeSpan.Zero,
            false
        );

    public static QosProfile Default { get; } =
        new(
            HistoryPolicy.KeepLast,
            10,
            ReliabilityPolicy.Reliable,
            DurabilityPolicy.Volatile,
            TimeSpan.Zero,
            TimeSpan.Zero,
            LivelinessPolicy.Default,
            TimeSpan.Zero,
            false
        );

    public static QosProfile ServicesDefault { get; } =
        new(
            HistoryPolicy.KeepLast,
            10,
            ReliabilityPolicy.Reliable,
            DurabilityPolicy.Volatile,
            TimeSpan.Zero,
            TimeSpan.Zero,
            LivelinessPolicy.Default,
            TimeSpan.Zero,
            false
        );

    public static QosProfile ParameterEvents { get; } =
        new(
            HistoryPolicy.KeepLast,
            1000,
            ReliabilityPolicy.Reliable,
            DurabilityPolicy.Volatile,
            TimeSpan.Zero,
            TimeSpan.Zero,
            LivelinessPolicy.Default,
            TimeSpan.Zero,
            false
        );

    public static QosProfile SystemDefault { get; } =
        new(
            HistoryPolicy.Default,
            0,
            ReliabilityPolicy.Default,
            DurabilityPolicy.Default,
            TimeSpan.Zero,
            TimeSpan.Zero,
            LivelinessPolicy.Default,
            TimeSpan.Zero,
            false
        );

    public static QosProfile Unknown { get; } =
        new(
            HistoryPolicy.Unknown,
            0,
            ReliabilityPolicy.Unknown,
            DurabilityPolicy.Unknown,
            TimeSpan.Zero,
            TimeSpan.Zero,
            LivelinessPolicy.Unknown,
            TimeSpan.Zero,
            false
        );

    public override string ToString()
    {
        return $"QosProfile(Depth = {Depth}, Reliability = {Enum.GetName(Reliability)}, Durability = {Enum.GetName(Durability)})";
    }

    internal rmw_qos_profile_t ToRmwQosProfile()
    {
        return new rmw_qos_profile_t
        {
            history = (rmw_qos_history_policy_t)History,
            depth = (uint)Depth,
            reliability = (rmw_qos_reliability_policy_t)Reliability,
            durability = (rmw_qos_durability_policy_t)Durability,
            deadline = Deadline.ToRmwTime(),
            lifespan = LifeSpan.ToRmwTime(),
            liveliness = (rmw_qos_liveliness_policy_t)Liveliness,
            avoid_ros_namespace_conventions = AvoidRosNamespaceConversions
        };
    }

    internal static QosProfile Create(in rmw_qos_profile_t profile)
    {
        return new(
            (HistoryPolicy)profile.history,
            (int)profile.depth.Value,
            (ReliabilityPolicy)profile.reliability,
            (DurabilityPolicy)profile.durability,
            profile.deadline.ToTimeSpan(),
            profile.lifespan.ToTimeSpan(),
            (LivelinessPolicy)profile.liveliness,
            profile.liveliness_lease_duration.ToTimeSpan(),
            profile.avoid_ros_namespace_conventions);
    }
}