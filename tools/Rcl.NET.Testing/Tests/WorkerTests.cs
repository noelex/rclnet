using System.Text.Json;
using Rcl.NET.Testing;
using Xunit;

namespace Rcl.NET.Testing.Tests;

public sealed class WorkerTests : IDisposable
{
    private readonly string temp = Path.Combine(Path.GetTempPath(), "rclnet fixture " + Guid.NewGuid().ToString("N"));
    private readonly string source = Path.Combine(AppContext.BaseDirectory, "fixture", "Fixture.dll");
    private readonly Variant variant;

    public WorkerTests()
    {
        Directory.CreateDirectory(temp);
        var windows = OperatingSystem.IsWindows();
        var setup = Path.Combine(temp, windows ? "setup.cmd" : "setup.sh");
        var overlay = Path.Combine(temp, windows ? "overlay.cmd" : "overlay.sh");
        File.WriteAllText(setup, windows ? "@echo off\r\n" : "true\n");
        File.WriteAllText(overlay, windows ? "@set RCLNET_FIXTURE=overlay\r\n" : "export RCLNET_FIXTURE=overlay\n");
        variant = new Variant(new Profile { Id = "fixture", Setup = setup, Overlays = [overlay], TimeoutSeconds = 20 }, "rmw_probe");
    }

    private List<Event> Run(Request request, CancellationToken token = default)
    {
        var events = new List<Event>();
        WorkerProcess.Run(source, variant, request, token, events.Add);
        return events;
    }

    private Request Select(params string[] methods)
    {
        var discovery = Run(new Request { Source = source });
        return new Request
        {
            Source = source, Mode = "run",
            TestIds = discovery.Where(e => e.Kind == "case" && methods.Any(m => e.Method.EndsWith("." + m))).Select(e => e.Id).ToArray()
        };
    }

    [Fact]
    public void StaleProfilesDoNotEnableVariantDiscoveryWithoutMarker()
    {
        var assembly = Path.Combine(temp, "Rcl.NET.Tests.dll");
        File.WriteAllText(assembly, "unused");
        File.WriteAllText(Path.Combine(temp, "ros-test-profiles.json"), "{}");
        var marker = Path.ChangeExtension(assembly, ".ros-variants");

        Assert.Empty(TestCases.Discover([assembly], null!, default));
        File.WriteAllText(marker, "1");
        Assert.True(TestCases.IsVariantSource(assembly));
        File.Delete(marker);
        Assert.Empty(TestCases.Discover([assembly], null!, default));
    }

    [Fact]
    public void DiscoveryHasStableTheoryIdsAndSourceLocations()
    {
        var first = Run(new Request { Source = source }).Where(e => e.Kind == "case").ToArray();
        var second = Run(new Request { Source = source }).Where(e => e.Kind == "case").ToArray();
        Assert.Equal(7, first.Length);
        Assert.Equal(first.Select(e => e.Id).Order(), second.Select(e => e.Id).Order());
        Assert.Equal(7, first.Select(e => e.Id).Distinct().Count());
        Assert.All(first, e => Assert.True(e.Line > 0 && e.File.EndsWith("Cases.cs")));
    }

    [Fact]
    public void MapsPassFailureSkipOutputAndTheoryRows()
    {
        var events = Run(Select("Pass", "Failure", "Skip", "Rows"));
        var results = events.Where(e => e.Kind == "result").ToArray();
        Assert.Equal(5, results.Length);
        Assert.Equal(3, results.Count(e => e.Outcome == "Passed"));
        Assert.Single(results, e => e.Outcome == "Skipped" && e.Output.Contains("intentional skip"));
        Assert.Single(results, e => e.Outcome == "Failed" && e.Message.Contains("intentional failure") && e.Stack.Length > 0);
        Assert.Contains(results, e => e.Output.Contains("worker output captured"));
    }

    [Fact]
    public void RunsOnlyOneSelectedTheoryRow()
    {
        var request = Select("Rows");
        request.TestIds = [request.TestIds[0]];
        var results = Run(request).Where(e => e.Kind == "result").ToArray();
        Assert.Equal(request.TestIds[0], Assert.Single(results).Id);
    }

    [Fact]
    public void NativeCrashIsReportedAsWorkerFailure()
    {
        var request = Select("Crash");
        var exception = Assert.Throws<InvalidOperationException>(() => Run(request));
        Assert.Contains("23", exception.Message);
    }

    [Fact]
    public void TimeoutKillsWorker()
    {
        var request = Select("Hang");
        variant.Profile.TimeoutSeconds = 1;
        Assert.Throws<TimeoutException>(() => Run(request));
    }

    [Fact]
    public void CancellationKillsWorker()
    {
        var request = Select("Hang");
        using var cancellation = new CancellationTokenSource();
        Assert.Throws<OperationCanceledException>(() => WorkerProcess.Run(source, variant, request, cancellation.Token, e =>
        {
            if (e.Kind == "start")
            {
                cancellation.Cancel();
            }
        }));
    }

    [Fact]
    public void DebugHandshakeWaitsBeforeExecutingTests()
    {
        var request = Select("Rows");
        request.Debug = true;
        var events = Run(request);
        Assert.Equal("debug", events[0].Kind);
        Assert.NotEqual(Environment.ProcessId, events[0].ProcessId);
        Assert.Equal(2, events.Count(e => e.Kind == "result"));
    }

    [Fact]
    public void MissingSelectionFailsInsteadOfSilentlyRunningEverything()
    {
        var events = new List<Event>();
        Assert.Throws<InvalidOperationException>(() => WorkerProcess.Run(source, variant,
            new Request { Source = source, Mode = "run", TestIds = ["missing"] }, default, events.Add));
        Assert.Contains(events, e => e.Kind == "fatal" && e.Message.Contains("no longer exist"));
    }

    [Fact]
    public void LocalProfilesOverrideDefaultsAndRespectHostOs()
    {
        var os = OperatingSystem.IsWindows() ? "windows" : "linux";
        var profile = variant.Profile;
        profile.Os = os;
        var path = Path.Combine(temp, "Fixture.dll");
        File.WriteAllText(Path.Combine(temp, "ros-test-profiles.json"), JsonSerializer.Serialize(new Configuration { Profiles = [profile] }));
        Assert.Equal(2, Profiles.Load(path).Length);
        profile.Rmw = ["rmw_custom"];
        File.WriteAllText(Path.Combine(temp, "ros-test-profiles.local.json"), JsonSerializer.Serialize(new Configuration { Profiles = [profile] }));
        Assert.Equal("rmw_custom", Assert.Single(Profiles.Load(path)).Rmw);
    }

    [Theory]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(false, false)]
    public void AutoDetectSkipsProfilesWithMissingPrerequisites(bool setupExists, bool overlayExists)
    {
        var available = variant.Profile;
        available.Os = OperatingSystem.IsWindows() ? "windows" : "linux";
        var incomplete = new Profile
        {
            Id = "incomplete",
            Os = available.Os,
            AutoDetect = true,
            Setup = setupExists ? available.Setup : Path.Combine(temp, "missing-setup"),
            Overlays = [available.Overlays[0], overlayExists ? available.Overlays[0] : Path.Combine(temp, "missing-overlay")]
        };
        File.WriteAllText(Path.Combine(temp, "ros-test-profiles.json"),
            JsonSerializer.Serialize(new Configuration { Profiles = [incomplete, available] }));
        var path = Path.Combine(temp, "Fixture.dll");

        var variants = Profiles.Load(path);
        Assert.Equal(available.Rmw.Length, variants.Length);
        Assert.All(variants, item => Assert.Equal(available.Id, item.Profile.Id));

        incomplete.AutoDetect = false;
        File.WriteAllText(Path.Combine(temp, "ros-test-profiles.json"),
            JsonSerializer.Serialize(new Configuration { Profiles = [incomplete, available] }));
        Assert.Contains("does not exist", Assert.Throws<InvalidDataException>(() => Profiles.Load(path)).Message);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void AutoDetectRejectsRelativePathsBeforeCheckingExistence(bool relativeSetup)
    {
        var profile = variant.Profile;
        profile.Os = OperatingSystem.IsWindows() ? "windows" : "linux";
        profile.AutoDetect = true;
        profile.Setup = relativeSetup ? "relative-setup" : Path.Combine(temp, "missing-setup");
        profile.Overlays = [relativeSetup ? Path.Combine(temp, "missing-overlay") : "relative-overlay"];
        File.WriteAllText(Path.Combine(temp, "ros-test-profiles.json"),
            JsonSerializer.Serialize(new Configuration { Profiles = [profile] }));

        var exception = Assert.Throws<InvalidDataException>(() => Profiles.Load(Path.Combine(temp, "Fixture.dll")));
        Assert.Contains("must be an absolute path", exception.Message);
    }

    public void Dispose()
    {
        Directory.Delete(temp, recursive: true);
    }
}
