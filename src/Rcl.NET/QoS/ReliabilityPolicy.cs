namespace Rcl.Qos;

/// <summary>
/// Reliability requirement of the service.
/// </summary>
public enum ReliabilityPolicy
{
    /// <summary>
    /// Implementation specific default
    /// </summary>
    Default = rmw_qos_reliability_policy_t.RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,

    /// <summary>
    /// Guarantee that samples are delivered, may retry multiple times.
    /// </summary>
    Reliable = rmw_qos_reliability_policy_t.RMW_QOS_POLICY_RELIABILITY_RELIABLE,

    /// <summary>
    /// Attempt to deliver samples, but some may be lost if the network is not robust
    /// </summary>
    BestEffort = rmw_qos_reliability_policy_t.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,

    /// <summary>
    /// Reliability policy has not yet been set
    /// </summary>
    Unknown = rmw_qos_reliability_policy_t.RMW_QOS_POLICY_RELIABILITY_UNKNOWN,
}