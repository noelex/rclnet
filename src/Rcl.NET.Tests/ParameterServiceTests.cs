using Rcl.Parameters;

namespace Rcl.NET.Tests;

[Collection("Sequential")]
public class ParameterServiceTests
{
    [Fact]
    public async Task ParameterOverrides()
    {
        var parameters = new Dictionary<string, Variant>
        {
            ["param1"] = 1,
            ["param2"] = "string"
        };

        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(
            NameGenerator.GenerateNodeName(),
            options: new(parameterOverrides: parameters)
        );

        Assert.Equal(parameters["param1"], node.Parameters.Declare("param1", 2));
        Assert.Equal(parameters["param2"], node.Parameters.Declare("param2", "some string"));
    }

    [Fact]
    public async Task IgnoreParameterOverrides()
    {
        var parameters = new Dictionary<string, Variant>
        {
            ["param1"] = 1,
            ["param2"] = "string"
        };

        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(
            NameGenerator.GenerateNodeName(),
            options: new(parameterOverrides: parameters)
        );

        Assert.Equal(2, node.Parameters.Declare("param1", 2, true));
        Assert.Equal("some string", node.Parameters.Declare("param2", "some string", true));
    }

    [Fact]
    public async Task DeclareParameterFromOverrides()
    {
        var parameters = new Dictionary<string, Variant>
        {
            ["param1"] = 1,
            ["param2"] = "string"
        };

        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(
            NameGenerator.GenerateNodeName(),
            options: new(parameterOverrides: parameters, declareParameterFromOverrides: true)
        );

        Assert.Equal(parameters["param1"], node.Parameters.Get("param1"));
        Assert.Equal(parameters["param2"], node.Parameters.Get("param2"));
    }
}
