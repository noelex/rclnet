namespace Rcl.Qos;

/// <summary>
/// Select one or more policies specified by a <see cref="QosProfile"/>.
/// </summary>
public enum QosPolicyKind
{
    /// <summary>
    /// No QoS policy is specified.
    /// </summary>
    None = 1 << 0,

    /// <summary>
    /// <see cref="QosProfile.Durability"/> is specified.
    /// </summary>
    Durability = 1 << 1,

    /// <summary>
    /// <see cref="QosProfile.Deadline"/> is specified.
    /// </summary>
    Deadline = 1 << 2,

    /// <summary>
    /// <see cref="QosProfile.Liveliness"/> is specified.
    /// </summary>
    Liveliness = 1 << 3,

    /// <summary>
    /// <see cref="QosProfile.Reliability"/> is specified.
    /// </summary>
    Reliability = 1 << 4,

    /// <summary>
    /// <see cref="QosProfile.History"/> is specified.
    /// </summary>
    History = 1 << 5,

    /// <summary>
    /// <see cref="QosProfile.LifeSpan"/> is specified.
    /// </summary>
    LifeSpan = 1 << 6,

    /// <summary>
    /// <see cref="QosProfile.Depth"/> is specified.
    /// </summary>
    Depth = 1 << 7,

    /// <summary>
    /// <see cref="QosProfile.LivelinessLeaseDuration"/> is specified.
    /// </summary>
    LivelinessLeaseDuration = 1 << 8,

    /// <summary>
    /// <see cref="QosProfile.AvoidRosNamespaceConventions"/> is specified.
    /// </summary>
    AvoidRosNamespaceConventions = 1 << 9,
}