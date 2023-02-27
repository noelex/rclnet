namespace Rcl.Runtime;

/// <summary>
/// Indicates that the code element is only supported by specific ROS 2 distribution.
/// </summary>
[AttributeUsage(AttributeTargets.All)]
public class SupportedDistributionAttribute : Attribute
{
    /// <summary>
    /// Creates a new <see cref="SupportedDistributionAttribute"/>.
    /// </summary>
    /// <param name="distribution">The name of the supported distribution.</param>
    public SupportedDistributionAttribute(string distribution)
    {
        Distribution = distribution;
    }

    /// <summary>
    /// Gets the name of the supported distribution.
    /// </summary>
    public string Distribution { get; set; }
}

/// <summary>
/// Indicates that the code element is only supported after (inclusive) specific ROS 2 distribution.
/// </summary>
[AttributeUsage(AttributeTargets.All)]
public class SupportedSinceDistributionAttribute : Attribute
{
    /// <summary>
    /// Creates a new <see cref="SupportedSinceDistributionAttribute"/>.
    /// </summary>
    /// <param name="distribution">The name of the supported distribution.</param>
    public SupportedSinceDistributionAttribute(string distribution)
    {
        Distribution = distribution;
    }

    /// <summary>
    /// Gets the name of the supported distribution.
    /// </summary>
    public string Distribution { get; set; }
}

/// <summary>
/// Indicates that the code element is only supported until (exclusive) specific ROS 2 distribution.
/// </summary>
[AttributeUsage(AttributeTargets.All)]
public class SupportedUntilDistributionAttribute : Attribute
{
    /// <summary>
    /// Creates a new <see cref="SupportedUntilDistributionAttribute"/>.
    /// </summary>
    /// <param name="distribution">The name of the supported distribution.</param>
    public SupportedUntilDistributionAttribute(string distribution)
    {
        Distribution = distribution;
    }

    /// <summary>
    /// Gets the name of the supported distribution.
    /// </summary>
    public string Distribution { get; set; }
}