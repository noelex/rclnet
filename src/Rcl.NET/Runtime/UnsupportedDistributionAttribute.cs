namespace Rcl.Runtime;

/// <summary>
/// Indicates that the code element is not supported by specific ROS 2 distribution.
/// </summary>
[AttributeUsage(AttributeTargets.All)]
public class UnsupportedDistributionAttribute : Attribute
{
    public UnsupportedDistributionAttribute(string distribution)
    {
        Distribution = distribution;
    }

    public string Distribution { get; set; }
}

/// <summary>
/// Indicates that the code element is not supported after (inclusive) specific ROS 2 distribution.
/// </summary>
[AttributeUsage(AttributeTargets.All)]
public class UnsupportedSinceDistributionAttribute : Attribute
{
    public UnsupportedSinceDistributionAttribute(string distribution)
    {
        Distribution = distribution;
    }

    public string Distribution { get; set; }
}

/// <summary>
/// Indicates that the code element is not supported until (exclusive) specific ROS 2 distribution.
/// </summary>
[AttributeUsage(AttributeTargets.All)]
public class UnsupportedUntilDistributionAttribute : Attribute
{
    public UnsupportedUntilDistributionAttribute(string distribution)
    {
        Distribution = distribution;
    }

    public string Distribution { get; set; }
}