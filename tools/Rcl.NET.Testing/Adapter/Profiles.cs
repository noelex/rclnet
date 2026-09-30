using System.Text.Json;

namespace Rcl.NET.Testing;

internal sealed record Variant(Profile Profile, string Rmw)
{
    public string Key => Profile.Id + "/" + Rmw;
}

internal static class Profiles
{
    internal static Variant[] Load(string source)
    {
        var directory = Path.GetDirectoryName(Path.GetFullPath(source))!;
        var profiles = new Dictionary<string, Profile>(StringComparer.Ordinal);
        foreach (var name in new[] { "ros-test-profiles.json", "ros-test-profiles.local.json" })
        {
            var path = Path.Combine(directory, name);
            if (!File.Exists(path))
            {
                continue;
            }

            var config = JsonSerializer.Deserialize<Configuration>(File.ReadAllText(path), Wire.Json)
                ?? throw new InvalidDataException($"Empty profile configuration: {path}");
            if (config.Version != 1)
            {
                throw new InvalidDataException($"Unsupported profile configuration version: {config.Version}");
            }

            var ids = new HashSet<string>();
            foreach (var profile in config.Profiles)
            {
                if (string.IsNullOrWhiteSpace(profile.Id) || !ids.Add(profile.Id))
                {
                    throw new InvalidDataException($"Empty or duplicate profile ID in {path}: {profile.Id}");
                }

                if (profile.Os is not ("windows" or "linux"))
                {
                    throw new InvalidDataException($"Profile {profile.Id}: os must be windows or linux.");
                }

                profiles[profile.Id] = profile;
            }
        }

        var os = OperatingSystem.IsWindows() ? "windows" : "linux";
        var variants = new List<Variant>();
        foreach (var profile in profiles.Values.Where(p => p.Enabled && p.Os == os))
        {
            var paths = new[] { profile.Setup }.Concat(profile.Overlays).ToArray();
            foreach (var path in paths)
            {
                if (!Path.IsPathFullyQualified(path))
                {
                    throw new InvalidDataException($"Profile {profile.Id}: setup/overlay must be an absolute path: {path}");
                }
            }

            if (profile.AutoDetect && paths.Any(path => !File.Exists(path)))
            {
                continue;
            }

            foreach (var path in paths)
            {
                if (!File.Exists(path))
                {
                    throw new InvalidDataException($"Profile {profile.Id}: setup/overlay does not exist: {path}");
                }
            }

            if (profile.TimeoutSeconds <= 0 || profile.Rmw.Length == 0 || profile.Rmw.Any(string.IsNullOrWhiteSpace)
                || profile.Rmw.Distinct().Count() != profile.Rmw.Length)
            {
                throw new InvalidDataException($"Profile {profile.Id}: invalid timeout or RMW list.");
            }

            if (profile.PixiManifest.Length > 0 && (!Path.IsPathFullyQualified(profile.PixiManifest) || !File.Exists(profile.PixiManifest)))
            {
                throw new InvalidDataException($"Profile {profile.Id}: Pixi manifest not found: {profile.PixiManifest}");
            }

            if (profile.PixiManifest.Length == 0 && profile.Python.Length > 0
                && (!Path.IsPathFullyQualified(profile.Python) || !File.Exists(profile.Python)))
            {
                throw new InvalidDataException($"Profile {profile.Id}: Python executable not found: {profile.Python}");
            }

            variants.AddRange(profile.Rmw.Select(rmw => new Variant(profile, rmw)));
        }

        if (variants.Count == 0)
        {
            throw new InvalidDataException($"No enabled ROS profiles for {os}. Configure ros-test-profiles.local.json and rebuild.");
        }

        return variants.ToArray();
    }
}
