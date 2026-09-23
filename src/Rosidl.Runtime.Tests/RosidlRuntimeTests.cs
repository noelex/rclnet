using Xunit;

namespace Rosidl.Runtime.Tests;

public class RosidlRuntimeTests
{
    public static TheoryData<string, RosidlNativeAbi> SupportedDistributions => new()
    {
        { "foxy", RosidlNativeAbi.V1 },
        { "humble", RosidlNativeAbi.V1 },
        { "iron", RosidlNativeAbi.V1 },
        { "jazzy", RosidlNativeAbi.V1 },
        { "kilted", RosidlNativeAbi.V1 },
        { "lyrical", RosidlNativeAbi.V2 }
    };

    [Theory]
    [MemberData(nameof(SupportedDistributions))]
    public void ResolveNativeAbiMapsSupportedDistributions(string distro, RosidlNativeAbi expected)
    {
        Assert.Equal(expected, RosidlAbiResolver.Resolve(distro));
    }

    [Theory]
    [InlineData(null, "<unset>")]
    [InlineData("", "<empty>")]
    [InlineData("rolling", "rolling")]
    public void ResolveNativeAbiRejectsUnsupportedDistributions(string? distro, string displayedDistro)
    {
        var exception = Assert.Throws<NotSupportedException>(
            () => RosidlAbiResolver.Resolve(distro));

        Assert.Contains(displayedDistro, exception.Message);
        Assert.Contains("foxy, humble, iron, jazzy, kilted, lyrical", exception.Message);
    }

    [Fact]
    public void NativeAbiMatchesCurrentRosEnvironment()
    {
        // Keep the process ABI consistent with the native libraries used by other tests.
        var expectedAbi = RosidlAbiResolver.Resolve(Environment.GetEnvironmentVariable("ROS_DISTRO"));
        var incompatibleAbi = expectedAbi == RosidlNativeAbi.V1
            ? RosidlNativeAbi.V2
            : RosidlNativeAbi.V1;

        Assert.Equal(expectedAbi, RosidlRuntime.NativeAbi);
        RosidlRuntime.RequireNativeAbi(expectedAbi);

        var exception = Assert.Throws<InvalidOperationException>(
            () => RosidlRuntime.RequireNativeAbi(incompatibleAbi));
        Assert.Contains(expectedAbi.ToString(), exception.Message);
        Assert.Contains(incompatibleAbi.ToString(), exception.Message);
    }
}
