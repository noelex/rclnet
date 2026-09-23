using Xunit;

namespace CodegenTests;

[CollectionDefinition("ROS environment", DisableParallelization = true)]
public sealed class RosEnvironmentTestCollection
{
    public const string Name = "ROS environment";
}
