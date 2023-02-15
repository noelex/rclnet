namespace Rcl.Qos;

public enum ReliabilityPolicy
{
    /// <summary>
    /// Implementation specific default
    /// </summary>
    Default = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,

    /// <summary>
    /// Guarantee that samples are delivered, may retry multiple times.
    /// </summary>
    Reliable = RMW_QOS_POLICY_RELIABILITY_RELIABLE,

    /// <summary>
    /// Attempt to deliver samples, but some may be lost if the network is not robust
    /// </summary>
    BestEffort = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,

    /// <summary>
    /// Reliability policy has not yet been set
    /// </summary>
    Unknown = RMW_QOS_POLICY_RELIABILITY_UNKNOWN,
}