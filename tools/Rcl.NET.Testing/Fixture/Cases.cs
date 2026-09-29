using Xunit;
using Xunit.Abstractions;

namespace InfrastructureFixture;

public sealed class Cases(ITestOutputHelper output)
{
    [Fact]
    public void Pass()
    {
        Assert.Equal("rmw_probe", Environment.GetEnvironmentVariable("RMW_IMPLEMENTATION"));
        Assert.Equal("overlay", Environment.GetEnvironmentVariable("RCLNET_FIXTURE"));
        output.WriteLine("worker output captured");
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    public void Rows(int value)
    {
        Assert.InRange(value, 1, 2);
    }

    [Fact(Skip = "intentional skip")]
    public void Skip()
    {
    }

    [Fact]
    public void Failure()
    {
        Assert.Fail("intentional failure");
    }

    [Fact]
    public void Crash()
    {
        Environment.Exit(23);
    }

    [Fact]
    public async Task Hang()
    {
        await Task.Delay(Timeout.Infinite);
    }
}
