using Rosidl.Generator.CSharp;
using Xunit;

namespace CodegenTests;

[Collection(RosEnvironmentTestCollection.Name)]
public class AbiSpecTests
{
    public static TheoryData<string, RosidlAbiMode> SupportedDistributions => new()
    {
        { "foxy", RosidlAbiMode.V1 },
        { "humble", RosidlAbiMode.V1 },
        { "iron", RosidlAbiMode.V1 },
        { "jazzy", RosidlAbiMode.V1 },
        { "kilted", RosidlAbiMode.V1 },
        { "lyrical", RosidlAbiMode.V2 }
    };

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

    [Theory]
    [MemberData(nameof(SupportedDistributions))]
    public void NativeAbiResolvesFromRosDistro(string distro, RosidlAbiMode expected)
    {
        var actual = WithRosDistro(distro, () => new GeneratorOptions { Abi = RosidlAbiMode.Native }.Abi);

        Assert.Equal(expected, actual);
    }

    [Theory]
    [InlineData(null, "<unset>")]
    [InlineData("rolling", "rolling")]
    public void NativeAbiRejectsMissingOrUnsupportedRosDistro(string? distro, string displayedDistro)
    {
        var (exitCode, output) = WithRosDistro(distro, () => RunGenerator("--abi=native"));

        Assert.Equal(2, exitCode);
        Assert.Contains("ABI mode 'native'", output);
        Assert.Contains(displayedDistro, output);
    }

    [Theory]
    [InlineData("foxy", "V1", "V2")]
    [InlineData("lyrical", "V2", "V1")]
    public void NativeAbiGeneratesOnlyTheDistroLayout(
        string distro,
        string expectedAbi,
        string unexpectedAbi)
    {
        var outputDirectory = Path.Combine(Path.GetTempPath(), $"ros2cs-native-abi-{Guid.NewGuid():N}");

        try
        {
            var (exitCode, _) = WithRosDistro(distro, () => RunGenerator(
                "--abi=native",
                $"--from-directory={FindPackageRoot()}",
                "--include=ros2cs_test_msgs",
                "--ignore-missing",
                $"--output={outputDirectory}"));

            Assert.Equal(0, exitCode);
            var generatedFiles = Directory.GetFiles(outputDirectory, "*.g.cs", SearchOption.AllDirectories);
            Assert.NotEmpty(generatedFiles);
            Assert.All(generatedFiles, path =>
            {
                var contents = File.ReadAllText(path);
                Assert.Contains($"RosidlNativeAbi.{expectedAbi}", contents);
                Assert.Contains($"RosidlRuntime.RequireNativeAbi(global::Rosidl.Runtime.RosidlNativeAbi.{expectedAbi});", contents);
                Assert.DoesNotContain($"RosidlNativeAbi.{unexpectedAbi}", contents);
            });
        }
        finally
        {
            if (Directory.Exists(outputDirectory))
            {
                Directory.Delete(outputDirectory, recursive: true);
            }
        }
    }

    [Fact]
    public void CommandLineAbiCanOverrideNativeSpecWithoutRosDistro()
    {
        var specPath = Path.GetTempFileName();

        try
        {
            File.WriteAllText(specPath, "abi native");

            var (exitCode, _) = WithRosDistro(
                null,
                () => RunGenerator("--abi=portable", specPath));

            Assert.Equal(0, exitCode);
        }
        finally
        {
            File.Delete(specPath);
        }
    }

    [Fact]
    public void NativeCommandLineAbiOverridesPortableSpec()
    {
        var specPath = Path.GetTempFileName();

        try
        {
            File.WriteAllText(specPath, "abi portable");

            var (exitCode, output) = WithRosDistro(
                null,
                () => RunGenerator("--abi=native", specPath));

            Assert.Equal(2, exitCode);
            Assert.Contains("ABI mode 'native'", output);
        }
        finally
        {
            File.Delete(specPath);
        }
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

    private static T WithRosDistro<T>(string? distro, Func<T> action)
    {
        var originalDistro = Environment.GetEnvironmentVariable("ROS_DISTRO");

        try
        {
            Environment.SetEnvironmentVariable("ROS_DISTRO", distro);
            return action();
        }
        finally
        {
            Environment.SetEnvironmentVariable("ROS_DISTRO", originalDistro);
        }
    }

    private static string FindPackageRoot()
    {
        for (var directory = new DirectoryInfo(AppContext.BaseDirectory);
             directory != null;
             directory = directory.Parent)
        {
            var packageRoot = Path.Combine(directory.FullName, "packages");
            if (File.Exists(Path.Combine(packageRoot, "ros2cs_test_msgs", "package.xml")))
            {
                return packageRoot;
            }
        }

        throw new DirectoryNotFoundException("Could not find the CodegenTests package directory.");
    }
}
