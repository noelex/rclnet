using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using Microsoft.VisualStudio.TestPlatform.ObjectModel;
using Microsoft.VisualStudio.TestPlatform.ObjectModel.Adapter;
using Microsoft.VisualStudio.TestPlatform.ObjectModel.Logging;

namespace Rcl.NET.Testing;

internal static class TestCases
{
    internal const string ExecutorUri = "executor://rclnet/ros-xunit/v1";
    internal static readonly TestProperty XunitId = TestProperty.Register("RclNet.XunitId", "xUnit ID", typeof(string), typeof(TestCase));
    internal static readonly TestProperty Profile = TestProperty.Register("RclNet.Profile", "ROS profile", typeof(string), typeof(TestCase));
    internal static readonly TestProperty Rmw = TestProperty.Register("RclNet.Rmw", "RMW", typeof(string), typeof(TestCase));
    internal static readonly TestProperty Distro = TestProperty.Register("RclNet.Distro", "ROS distro", typeof(string), typeof(TestCase));

    internal static bool IsVariantSource(string source)
    {
        var marker = Path.ChangeExtension(source, ".ros-variants");
        return Path.GetFileName(source) == "Rcl.NET.Tests.dll"
            && File.Exists(marker) && File.ReadAllText(marker).Trim() == "1"
            && File.Exists(Path.Combine(Path.GetDirectoryName(source)!, "ros-test-profiles.json"));
    }

    private static string Framework(string source)
    {
        using var config = JsonDocument.Parse(File.ReadAllText(Path.ChangeExtension(source, ".runtimeconfig.json")));
        return config.RootElement.GetProperty("runtimeOptions").GetProperty("tfm").GetString()!;
    }

    internal static IEnumerable<TestCase> Discover(IEnumerable<string> sources, IMessageLogger logger, CancellationToken cancellation)
    {
        foreach (var source in sources.Where(IsVariantSource))
        {
            var framework = Framework(source);
            foreach (var variant in Profiles.Load(source))
            {
                var cases = new List<TestCase>();
                WorkerProcess.Run(source, variant, new Request { Source = Path.GetFullPath(source) }, cancellation, value =>
                {
                    if (value.Kind == "case")
                    {
                        var test = new TestCase(value.Method, new Uri(ExecutorUri), source)
                        {
                            CodeFilePath = value.File, LineNumber = value.Line,
                            DisplayName = $"{value.Name} [{variant.Key}]",
                            Id = new Guid(SHA256.HashData(Encoding.UTF8.GetBytes(framework + "|" + Path.GetFileName(source) + "|" + variant.Key + "|" + value.Id))[..16])
                        };
                        test.SetPropertyValue(XunitId, value.Id);
                        test.SetPropertyValue(Profile, variant.Profile.Id);
                        test.SetPropertyValue(Rmw, variant.Rmw);
                        test.SetPropertyValue(Distro, variant.Profile.Distro);
                        test.Traits.Add(new Trait("RosProfile", variant.Profile.Id));
                        test.Traits.Add(new Trait("RosEnvironment", variant.Key));
                        test.Traits.Add(new Trait("RMW", variant.Rmw));
                        test.Traits.Add(new Trait("RosDistro", variant.Profile.Distro));
                        foreach (var trait in value.Traits)
                        {
                            foreach (var item in trait.Value)
                            {
                                test.Traits.Add(new Trait(trait.Key, item));
                            }
                        }

                        cases.Add(test);
                    }
                    else if (value.Kind == "fatal")
                    {
                        throw new InvalidOperationException(value.Message + Environment.NewLine + value.Stack);
                    }
                    else if (value.Kind == "log")
                    {
                        logger.SendMessage(TestMessageLevel.Informational, $"[{variant.Key}] {value.Message}");
                    }
                });
                foreach (var test in cases)
                {
                    yield return test;
                }
            }
        }
    }
}

[FileExtension(".dll")]
[DefaultExecutorUri(TestCases.ExecutorUri)]
public sealed class Discoverer : ITestDiscoverer
{
    public void DiscoverTests(IEnumerable<string> sources, IDiscoveryContext context, IMessageLogger logger, ITestCaseDiscoverySink sink)
    {
        foreach (var test in TestCases.Discover(sources, logger, CancellationToken.None))
        {
            sink.SendTestCase(test);
        }
    }
}

[ExtensionUri(TestCases.ExecutorUri)]
public sealed class Executor : ITestExecutor
{
    private readonly object cancellationGate = new();
    private CancellationTokenSource cancellation = new();

    public void Cancel()
    {
        lock (cancellationGate)
        {
            cancellation.Cancel();
        }
    }

    private void BeginRun()
    {
        lock (cancellationGate)
        {
            cancellation.Dispose();
            cancellation = new CancellationTokenSource();
        }
    }

    public void RunTests(IEnumerable<string>? sources, IRunContext? context, IFrameworkHandle? handle)
    {
        ArgumentNullException.ThrowIfNull(handle);
        BeginRun();
        Execute(TestCases.Discover(sources ?? [], handle, cancellation.Token), context, handle);
    }

    public void RunTests(IEnumerable<TestCase>? tests, IRunContext? context, IFrameworkHandle? handle)
    {
        ArgumentNullException.ThrowIfNull(handle);
        BeginRun();
        Execute(tests, context, handle);
    }

    private void Execute(IEnumerable<TestCase>? tests, IRunContext? context, IFrameworkHandle handle)
    {
        ArgumentNullException.ThrowIfNull(handle);
        var available = (tests ?? []).ToArray();
        var supportedProperties = available.SelectMany(t => t.Traits.Select(trait => trait.Name))
            .Concat(["FullyQualifiedName", "Name", "DisplayName", "RosProfile", "RosDistro", "RMW"]).Distinct().ToArray();
        var filter = context?.GetTestCaseFilter(supportedProperties, property => property switch
        {
            "RosProfile" => TestCases.Profile,
            "RosDistro" => TestCases.Distro,
            "RMW" => TestCases.Rmw,
            _ => null
        });
        object? Property(TestCase test, string name)
        {
            return name switch
            {
                "FullyQualifiedName" => test.FullyQualifiedName,
                "Name" or "DisplayName" => test.DisplayName,
                _ => test.Traits.Where(t => t.Name == name).Select(t => t.Value).ToArray()
            };
        }

        var selected = available.Where(t => filter == null || filter.MatchTestCase(t, name => Property(t, name))).ToArray();
        foreach (var group in selected.GroupBy(t => (t.Source, Profile: t.GetPropertyValue(TestCases.Profile, ""), Rmw: t.GetPropertyValue(TestCases.Rmw, ""))))
        {
            if (cancellation.IsCancellationRequested)
            {
                break;
            }

            var pending = group.ToDictionary(t => t.GetPropertyValue(TestCases.XunitId, ""));
            var started = new HashSet<string>();
            void Start(string id)
            {
                if (started.Add(id))
                {
                    handle.RecordStart(pending[id]);
                }
            }

            void Record(string id, TestOutcome outcome, string message, string stack, string output, double seconds)
            {
                Start(id);
                var test = pending[id];
                var result = new TestResult(test)
                {
                    Outcome = outcome, ErrorMessage = message, ErrorStackTrace = stack,
                    Duration = TimeSpan.FromSeconds(seconds)
                };
                result.Messages.Add(new TestResultMessage(TestResultMessage.StandardOutCategory, output));
                handle.RecordResult(result);
                handle.RecordEnd(test, outcome);
                pending.Remove(id);
            }

            try
            {
                if (!TestCases.IsVariantSource(group.Key.Source))
                {
                    throw new InvalidOperationException("ROS variant mode is not enabled for this source. Rebuild and rediscover tests.");
                }

                var variant = Profiles.Load(group.Key.Source).Single(v => v.Profile.Id == group.Key.Profile && v.Rmw == group.Key.Rmw);
                WorkerProcess.Run(group.Key.Source, variant, new Request
                {
                    Source = Path.GetFullPath(group.Key.Source), Mode = "run", Debug = context?.IsBeingDebugged == true,
                    TestIds = pending.Keys.ToArray()
                }, cancellation.Token, value =>
                {
                    switch (value.Kind)
                    {
                        case "debug":
                            if (handle is not IFrameworkHandle2 debugger || !debugger.AttachDebuggerToProcess(value.ProcessId))
                            {
                                throw new InvalidOperationException("The test platform could not attach to the ROS worker.");
                            }

                            break;
                        case "start":
                            Start(value.Id);
                            break;
                        case "result":
                            Record(value.Id, Enum.Parse<TestOutcome>(value.Outcome), value.Message, value.Stack, value.Output, value.Seconds);
                            break;
                        case "fatal":
                            throw new InvalidOperationException(value.Message + Environment.NewLine + value.Stack);
                        case "log":
                            handle.SendMessage(TestMessageLevel.Informational, $"[{variant.Key}] {value.Message}");
                            break;
                    }
                });
                if (pending.Count != 0)
                {
                    throw new InvalidOperationException("Worker completed without reporting all selected tests.");
                }
            }
            catch (Exception exception)
            {
                handle.SendMessage(cancellation.IsCancellationRequested ? TestMessageLevel.Warning : TestMessageLevel.Error, exception.Message);
                foreach (var id in pending.Keys.ToArray())
                {
                    Record(id, cancellation.IsCancellationRequested ? TestOutcome.Skipped : TestOutcome.Failed,
                        exception.Message, exception.StackTrace ?? "", "", 0);
                }
            }
        }
    }
}
