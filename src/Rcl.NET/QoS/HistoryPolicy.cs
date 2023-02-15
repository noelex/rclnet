namespace Rcl.Qos;

/// <summary>
/// QoS history enumerations describing how samples endure
/// </summary>
public enum HistoryPolicy
{
    /// <summary>
    /// Implementation default for history policy
    /// </summary>
    Default = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,

    /// <summary>
    /// Only store up to a maximum number of samples, dropping oldest once max is exceeded
    /// </summary>
    KeepLast = RMW_QOS_POLICY_HISTORY_KEEP_LAST,

    /// <summary>
    /// Store all samples, subject to resource limits
    /// </summary>
    KeepAll = RMW_QOS_POLICY_HISTORY_KEEP_ALL,

    /// <summary>
    /// History policy has not yet been set
    /// </summary>
    Unknown = RMW_QOS_POLICY_HISTORY_UNKNOWN,
}