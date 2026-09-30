using System.Reflection;
using System.Runtime.Loader;
using System.Text.Json;
using Rcl.NET.Testing;
using Xunit;
using Xunit.Abstractions;

var wireOutput = Console.Out;
var writeGate = new object();
void Emit(Event value)
{
    lock (writeGate)
    {
        wireOutput.WriteLine(Wire.Prefix + JsonSerializer.Serialize(value, Wire.Json));
        wireOutput.Flush();
    }
}

try
{
    var request = JsonSerializer.Deserialize<Request>(File.ReadAllText(args[0]), Wire.Json)!;
    if (request.AutoDetectRmw)
    {
        var rmw = Environment.GetEnvironmentVariable("RMW_IMPLEMENTATION") ?? "";
        var prefixes = (Environment.GetEnvironmentVariable("AMENT_PREFIX_PATH") ?? "")
            .Split(Path.PathSeparator, StringSplitOptions.RemoveEmptyEntries);
        if (!prefixes.Any(prefix => File.Exists(Path.Combine(prefix, "share", "ament_index", "resource_index", "packages", rmw))))
        {
            var message = $"RMW package {rmw} was not found in AMENT_PREFIX_PATH after setup and overlays.";
            if (request.Mode != "discover")
            {
                throw new InvalidOperationException(message + " Rebuild and rediscover tests.");
            }

            Emit(new Event { Kind = "log", Message = "Skipping variant: " + message });
            Emit(new Event { Kind = "complete" });
            return 0;
        }
    }

    AppContext.SetSwitch("Rcl.NET.Testing.Worker", true);
    var directory = Path.GetDirectoryName(request.Source)!;
    var resolver = new AssemblyDependencyResolver(request.Source);
    AssemblyLoadContext.Default.Resolving += (context, name) =>
    {
        var path = resolver.ResolveAssemblyToPath(name) ?? Path.Combine(directory, name.Name + ".dll");
        return File.Exists(path) ? context.LoadFromAssemblyPath(path) : null;
    };
    AssemblyLoadContext.Default.ResolvingUnmanagedDll += (assembly, name) =>
    {
        var path = resolver.ResolveUnmanagedDllToPath(name);
        return path == null ? IntPtr.Zero : System.Runtime.InteropServices.NativeLibrary.Load(path);
    };
    Directory.SetCurrentDirectory(directory);
    if (request.Debug)
    {
        Emit(new Event { Kind = "debug", ProcessId = Environment.ProcessId });
        if (Console.ReadLine() != "continue")
        {
            throw new InvalidOperationException("Debugger attachment failed or was cancelled.");
        }
    }

    Emit(new Event { Kind = "log", Message = $"Worker PID={Environment.ProcessId}, ROS_DISTRO={Environment.GetEnvironmentVariable("ROS_DISTRO")}, RMW_IMPLEMENTATION={Environment.GetEnvironmentVariable("RMW_IMPLEMENTATION")}" });
    var sink = new Sink(Emit);
    using var controller = new XunitFrontController(AppDomainSupport.Denied, request.Source, shadowCopy: false, diagnosticMessageSink: sink);
    var settings = ConfigReader.Load(request.Source);
    var discoveryOptions = TestFrameworkOptions.ForDiscovery(settings);
    discoveryOptions.SetPreEnumerateTheories(true);
    controller.Find(false, sink, discoveryOptions);
    sink.DiscoveryDone.WaitOne();
    if (sink.Fatal)
    {
        return 1;
    }

    if (request.Mode == "discover")
    {
        var locations = SourceLocations.Read(request.Source);
        foreach (var test in sink.Cases)
        {
            var method = test.TestMethod.TestClass.Class.Name + "." + test.TestMethod.Method.Name;
            locations.TryGetValue(method, out var location);
            Emit(new Event
            {
                Kind = "case", Id = test.UniqueID, Name = test.DisplayName,
                Method = test.TestMethod.TestClass.Class.Name + "." + test.TestMethod.Method.Name,
                Traits = test.Traits, File = location.File ?? "", Line = location.Line
            });
        }
    }
    else
    {
        var ids = request.TestIds.ToHashSet();
        var selected = sink.Cases.Where(c => ids.Contains(c.UniqueID)).ToArray();
        var missing = ids.Except(selected.Select(c => c.UniqueID)).ToArray();
        if (missing.Length > 0)
        {
            throw new InvalidOperationException($"{missing.Length} selected xUnit cases no longer exist. Rebuild and rediscover tests.");
        }

        var executionOptions = TestFrameworkOptions.ForExecution(settings);
        executionOptions.SetDisableParallelization(true);
        controller.RunTests(selected, sink, executionOptions);
        sink.ExecutionDone.WaitOne();
    }

    Emit(new Event { Kind = "complete" });
    return sink.Fatal ? 1 : 0;
}
catch (Exception exception)
{
    Emit(new Event { Kind = "fatal", Message = exception.ToString() });
    return 1;
}

internal sealed class Sink(Action<Event> emit) : IMessageSink
{
    public readonly ManualResetEvent DiscoveryDone = new(false);
    public readonly ManualResetEvent ExecutionDone = new(false);
    public List<ITestCase> Cases { get; } = [];
    public bool Fatal { get; private set; }
    private readonly Dictionary<string, Event> results = [];

    public bool OnMessage(IMessageSinkMessage message)
    {
        switch (message)
        {
            case ITestCaseDiscoveryMessage found:
                Cases.Add(found.TestCase);
                break;
            case IDiscoveryCompleteMessage:
                DiscoveryDone.Set();
                break;
            case ITestCaseStarting starting:
                results[starting.TestCase.UniqueID] = new Event { Kind = "result", Id = starting.TestCase.UniqueID, Outcome = "Skipped" };
                emit(new Event { Kind = "start", Id = starting.TestCase.UniqueID });
                break;
            case ITestResultMessage result:
                var aggregate = results[result.TestCase.UniqueID];
                aggregate.Seconds += (double)result.ExecutionTime;
                aggregate.Output += result.Output;
                if (result is ITestFailed failed)
                {
                    aggregate.Outcome = "Failed";
                    aggregate.Message += string.Join(Environment.NewLine, failed.Messages) + Environment.NewLine;
                    aggregate.Stack += string.Join(Environment.NewLine, failed.StackTraces) + Environment.NewLine;
                }
                else if (result is ITestPassed && aggregate.Outcome != "Failed")
                {
                    aggregate.Outcome = "Passed";
                }
                else if (result is ITestSkipped skipped)
                {
                    aggregate.Output += skipped.Reason + Environment.NewLine;
                }

                break;
            case ITestCaseFinished finished:
                emit(results[finished.TestCase.UniqueID]);
                break;
            case ITestAssemblyFinished:
                ExecutionDone.Set();
                break;
            case IErrorMessage error:
                Fatal = true;
                emit(new Event { Kind = "fatal", Message = string.Join(Environment.NewLine, error.Messages), Stack = string.Join(Environment.NewLine, error.StackTraces) });
                break;
            case IDiagnosticMessage diagnostic:
                emit(new Event { Kind = "log", Message = diagnostic.Message });
                break;
        }

        return true;
    }
}
