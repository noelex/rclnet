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
    public void NativeAbiIsResolvedDuringTypeInitialization()
    {
        var originalDistro = Environment.GetEnvironmentVariable("ROS_DISTRO");

        try
        {
            Environment.SetEnvironmentVariable("ROS_DISTRO", "lyrical");

            Assert.Equal(RosidlNativeAbi.V2, RosidlRuntime.NativeAbi);
            RosidlRuntime.RequireNativeAbi(RosidlNativeAbi.V2);

            var exception = Assert.Throws<InvalidOperationException>(
                () => RosidlRuntime.RequireNativeAbi(RosidlNativeAbi.V1));
            Assert.Contains("V2", exception.Message);
            Assert.Contains("V1", exception.Message);
        }
        finally
        {
            Environment.SetEnvironmentVariable("ROS_DISTRO", originalDistro);
        }
    }
}
