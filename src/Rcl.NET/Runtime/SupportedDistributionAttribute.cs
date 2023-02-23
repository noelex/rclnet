namespace Rcl.Runtime;

/// <summary>
/// Indicates that the code element is only supported by specific ROS 2 distribution.
/// </summary>
[AttributeUsage(AttributeTargets.All)]
public class SupportedDistributionAttribute : Attribute
{
    public SupportedDistributionAttribute(string distribution)
    {
        Distribution = distribution;
    }

    public string Distribution { get; set; }
}

/// <summary>
/// Indicates that the code element is only supported after (inclusive) specific ROS 2 distribution.
/// </summary>
[AttributeUsage(AttributeTargets.All)]
public class SupportedSinceDistributionAttribute : Attribute
{
    public SupportedSinceDistributionAttribute(string distribution)
    {
        Distribution = distribution;
    }

    public string Distribution { get; set; }
}

/// <summary>
/// Indicates that the code element is only supported until (exclusive) specific ROS 2 distribution.
/// </summary>
[AttributeUsage(AttributeTargets.All)]
public class SupportedUntilDistributionAttribute : Attribute
{
    public SupportedUntilDistributionAttribute(string distribution)
    {
        Distribution = distribution;
    }

    public string Distribution { get; set; }
}