using Rosidl.Generator.CSharp;
using Xunit;

namespace CodegenTests;

public class MappingSpecTests
{
    [Fact]
    public void MapNameFromSpecChangesGeneratedPackageName()
    {
        RunMappingTest("map-name ros2cs_test_msgs:FromSpec", [], "Rosidl.Messages.FromSpec");
    }

    [Fact]
    public void CommandLineMapNameOverridesSpec()
    {
        RunMappingTest(
            "map-name ros2cs_test_msgs:FromSpec",
            ["--map-name=ros2cs_test_msgs:FromCommandLine"],
            "Rosidl.Messages.FromCommandLine");
    }

    [Fact]
    public void LegacyMapPackageDirectiveRemainsSupported()
    {
        RunMappingTest("map-package ros2cs_test_msgs:LegacyName", [], "Rosidl.Messages.LegacyName");
    }

    [Fact]
    public void CommandLineMapNamespaceOverridesSpec()
    {
        RunMappingTest(
            "map-namespace ros2cs_test_msgs:From.Spec",
            ["--map-namespace=ros2cs_test_msgs:From.CommandLine"],
            "From.CommandLine.Ros2csTest");
    }

    private static void RunMappingTest(string specContents, string[] extraArguments, string expectedNamespace)
    {
        var tempRoot = Path.Combine(Path.GetTempPath(), $"ros2cs-map-name-{Guid.NewGuid():N}");
        var outputDirectory = Path.Combine(tempRoot, "generated");
        var specPath = Path.Combine(tempRoot, "ros2cs.spec");
        Directory.CreateDirectory(tempRoot);

        try
        {
            File.WriteAllText(specPath, specContents);

            var arguments = new[]
            {
                $"--from-directory={FindPackageRoot()}",
                "--include=ros2cs_test_msgs",
                "--ignore-missing",
                $"--output={outputDirectory}",
            }.Concat(extraArguments).Append(specPath).ToArray();

            var exitCode = CSharpCodeGenerator.Generate(arguments);

            Assert.Equal(0, exitCode);
            var generatedFiles = Directory.GetFiles(outputDirectory, "*.g.cs", SearchOption.AllDirectories);
            Assert.NotEmpty(generatedFiles);
            Assert.All(generatedFiles, path =>
                Assert.Contains($"namespace {expectedNamespace}", File.ReadAllText(path)));
        }
        finally
        {
            Directory.Delete(tempRoot, recursive: true);
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
