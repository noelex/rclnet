namespace Rcl.Runtime;

/// <summary>
/// Indicates that the code element is not supported by specific ROS 2 distribution.
/// </summary>
[AttributeUsage(AttributeTargets.All)]
public class UnsupportedDistributionAttribute : Attribute
{
    /// <summary>
    /// Creates a new <see cref="UnsupportedDistributionAttribute"/>.
    /// </summary>
    /// <param name="distribution">The name of the unsupported distribution.</param>
    public UnsupportedDistributionAttribute(string distribution)
    {
        Distribution = distribution;
    }

    /// <summary>
    /// Gets the name of the unsupported distribution.
    /// </summary>
    public string Distribution { get; set; }
}

/// <summary>
/// Indicates that the code element is not supported since (inclusive) specific ROS 2 distribution.
/// </summary>
[AttributeUsage(AttributeTargets.All)]
public class UnsupportedSinceDistributionAttribute : Attribute
{
    /// <summary>
    /// Creates a new <see cref="UnsupportedSinceDistributionAttribute"/>.
    /// </summary>
    /// <param name="distribution">The name of the unsupported distribution.</param>
    public UnsupportedSinceDistributionAttribute(string distribution)
    {
        Distribution = distribution;
    }

    /// <summary>
    /// Gets the name of the unsupported distribution.
    /// </summary>
    public string Distribution { get; set; }
}

/// <summary>
/// Indicates that the code element is not supported until (exclusive) specific ROS 2 distribution.
/// </summary>
[AttributeUsage(AttributeTargets.All)]
public class UnsupportedUntilDistributionAttribute : Attribute
{
    /// <summary>
    /// Creates a new <see cref="UnsupportedUntilDistributionAttribute"/>.
    /// </summary>
    /// <param name="distribution">The name of the unsupported distribution.</param>
    public UnsupportedUntilDistributionAttribute(string distribution)
    {
        Distribution = distribution;
    }

    /// <summary>
    /// Gets the name of the unsupported distribution.
    /// </summary>
    public string Distribution { get; set; }
}