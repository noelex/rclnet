using Rosidl.Generator.CSharp;
using Xunit;

namespace CodegenTests;

public class AbiSpecTests
{
    [Theory]
    [InlineData("abi", "'abi' requires exactly one argument.")]
    [InlineData("abi v3", "'v3' is not a valid value for 'abi'.")]
    [InlineData("abi v1\r\nabi v2", "'abi' can only be specified once in a spec file.")]
    public void InvalidAbiDirectivesReturnAnError(string contents, string expectedMessage)
    {
        var specPath = Path.GetTempFileName();

        try
        {
            File.WriteAllText(specPath, contents);
            var (exitCode, output) = RunGenerator(specPath);

            Assert.Equal(2, exitCode);
            Assert.Contains(expectedMessage, output);
        }
        finally
        {
            File.Delete(specPath);
        }
    }

    [Fact]
    public void DuplicateAbiCommandLineOptionsReturnAnError()
    {
        var (exitCode, output) = RunGenerator("--abi=v1", "--abi=v2");

        Assert.Equal(2, exitCode);
        Assert.Contains("Option 'abi'", output);
        Assert.Contains("can only be specified once", output);
    }

    private static (int ExitCode, string Output) RunGenerator(params string[] arguments)
    {
        var originalOutput = Console.Out;
        using var output = new StringWriter();

        try
        {
            Console.SetOut(output);
            return (CSharpCodeGenerator.Generate(arguments), output.ToString());
        }
        finally
        {
            Console.SetOut(originalOutput);
        }
    }
}
