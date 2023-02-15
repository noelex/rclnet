namespace Rcl.Qos;

public enum LivelinessPolicy
{
    /// <summary>
    /// Implementation specific default
    /// </summary>
    Default = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,

    /// <summary>
    /// The signal that establishes a Topic is alive comes from the ROS rmw layer.
    /// </summary>
    Automatic = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,

    ManualByNode = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE,

    /// <summary>
    /// The signal that establishes a Topic is alive is at the Topic level. Only publishing a message
    /// on the Topic or an explicit signal from the application to assert liveliness on the Topic
    /// will mark the Topic as being alive.
    /// Using `3` for backwards compatibility.
    /// </summary>
    ManualByTopic = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,

    /// <summary>
    /// Liveliness policy has not yet been set
    /// </summary>
    Unknown = RMW_QOS_POLICY_LIVELINESS_UNKNOWN,
}