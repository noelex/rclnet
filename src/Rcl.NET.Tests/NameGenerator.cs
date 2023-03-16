namespace Rcl.NET.Tests;

internal class NameGenerator
{
    private static string GenerateUniqueId() => string.Join("", Guid.NewGuid().ToByteArray().Select(x => x.ToString("x2")));

    public static string GenerateNodeName() => $"node_{GenerateUniqueId()}";

    public static string GenerateTopicName() => $"topic_{GenerateUniqueId()}";

    public static string GenerateServiceName() => $"service_{GenerateUniqueId()}";

    public static string GenerateActionName() => $"action_{GenerateUniqueId()}";
}
