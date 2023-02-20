namespace Rcl.Qos;

/// <summary>
/// QoS liveliness enumerations that describe a publisher's reporting policy for its alive status.
/// For a subscriber, these are its requirements for its topic's publishers.
/// </summary>
public enum LivelinessPolicy
{
    /// <summary>
    /// Implementation specific default
    /// </summary>
    Default = rmw_qos_liveliness_policy_t.RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,

    /// <summary>
    /// The signal that establishes a Topic is alive comes from the ROS rmw layer.
    /// </summary>
    Automatic = rmw_qos_liveliness_policy_t.RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,

    /// <summary>
    /// The signal that establishes a Topic is alive is at the Node level.
    /// </summary>
    ManualByNode = rmw_qos_liveliness_policy_t.RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE,

    /// <summary>
    /// The signal that establishes a Topic is alive is at the Topic level. Only publishing a message
    /// on the Topic or an explicit signal from the application to assert liveliness on the Topic
    /// will mark the Topic as being alive.
    /// Using `3` for backwards compatibility.
    /// </summary>
    ManualByTopic = rmw_qos_liveliness_policy_t.RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,

    /// <summary>
    /// Liveliness policy has not yet been set
    /// </summary>
    Unknown = rmw_qos_liveliness_policy_t.RMW_QOS_POLICY_LIVELINESS_UNKNOWN,
}