namespace Rcl.Qos;

/// <summary>
/// QoS durability enumerations describing how samples persist
/// </summary>
public enum DurabilityPolicy
{
    /// <summary>
    /// Implementation specific default
    /// </summary>
    Default = rmw_qos_durability_policy_t.RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,

    /// <summary>
    /// The rmw publisher is responsible for persisting samples for “late-joining” subscribers
    /// </summary>
    TransientLocal = rmw_qos_durability_policy_t.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,

    /// <summary>
    /// Samples are not persistent
    /// </summary>
    Volatile = rmw_qos_durability_policy_t.RMW_QOS_POLICY_DURABILITY_VOLATILE,

    /// <summary>
    /// Durability policy has not yet been set
    /// </summary>
    Unknown = rmw_qos_durability_policy_t.RMW_QOS_POLICY_DURABILITY_UNKNOWN,
}