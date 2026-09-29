using System.Text.Json;

namespace Rcl.NET.Testing;

internal static class Wire
{
    public const string Prefix = "@@RCLNET@@";
    public static readonly JsonSerializerOptions Json = new() { PropertyNameCaseInsensitive = true, WriteIndented = false };
}

internal sealed class Request
{
    public string Source { get; set; } = "";
    public string Mode { get; set; } = "discover";
    public bool Debug { get; set; }
    public string[] TestIds { get; set; } = [];
}

internal sealed class Event
{
    public string Kind { get; set; } = "";
    public string Id { get; set; } = "";
    public string Name { get; set; } = "";
    public string Method { get; set; } = "";
    public string File { get; set; } = "";
    public int Line { get; set; }
    public string Outcome { get; set; } = "";
    public string Message { get; set; } = "";
    public string Stack { get; set; } = "";
    public string Output { get; set; } = "";
    public double Seconds { get; set; }
    public int ProcessId { get; set; }
    public Dictionary<string, List<string>> Traits { get; set; } = [];
}

internal sealed class Profile
{
    public string Id { get; set; } = "";
    public string Os { get; set; } = "";
    public string Distro { get; set; } = "";
    public string Setup { get; set; } = "";
    public string[] Overlays { get; set; } = [];
    public string[] Rmw { get; set; } = ["rmw_fastrtps_cpp", "rmw_cyclonedds_cpp"];
    public string PixiManifest { get; set; } = "";
    public string PixiExecutable { get; set; } = "pixi.exe";
    public string Python { get; set; } = "";
    public string[] PathPrepend { get; set; } = [];
    public Dictionary<string, string> Environment { get; set; } = [];
    public bool Enabled { get; set; } = true;
    public bool AutoDetect { get; set; }
    public int TimeoutSeconds { get; set; } = 600;
}

internal sealed class Configuration
{
    public int Version { get; set; } = 1;
    public Profile[] Profiles { get; set; } = [];
}
