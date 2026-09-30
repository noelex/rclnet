using System.Diagnostics;
using System.Text;

if (args.Length > 1 || (args.Length == 1 && args[0] != "--dry-run"))
{
    Console.Error.WriteLine("Usage: dotnet tools/clean_vsut.cs [--dry-run]");
    return 2;
}

var dryRun = args.Length == 1;
try
{
    var containers = Lines(await DockerAsync("ps", "-aq", "--filter", "status=exited"));
    var targets = new List<(string Container, string Image)>();
    foreach (var container in containers)
    {
        var details = await DockerAsync("container", "inspect", "--format", "{{.Config.Image}} {{.Image}}", container);
        var parts = details.Trim().Split(' ', 2);
        if (parts.Length == 2 && parts[0] is "vsut_dockerfile" or "vsut_dockerfile:latest")
        {
            targets.Add((container, parts[1]));
        }
    }

    var currentImages = Lines(await DockerAsync("image", "ls", "--no-trunc", "-q", "--filter", "reference=vsut_dockerfile:latest"))
        .ToHashSet(StringComparer.Ordinal);
    var oldImages = new HashSet<string>(StringComparer.Ordinal);
    var failed = false;
    foreach (var target in targets)
    {
        if (await RemoveAsync("container", target.Container))
        {
            if (!currentImages.Contains(target.Image))
            {
                oldImages.Add(target.Image);
            }
        }
        else
        {
            failed = true;
        }
    }

    foreach (var image in oldImages)
    {
        if (!await RemoveAsync("image", image))
        {
            failed = true;
        }
    }

    if (targets.Count == 0)
    {
        Console.WriteLine("No exited Visual Studio test containers found.");
    }

    return failed ? 1 : 0;
}
catch (Exception exception)
{
    Console.Error.WriteLine(exception.Message);
    return 1;
}

async Task<bool> RemoveAsync(string kind, string id)
{
    if (dryRun)
    {
        Console.WriteLine($"Would remove {kind} {id}");
        return true;
    }

    var result = await RunDockerAsync(kind, "rm", id);
    Console.Write(result.Output);
    if (result.ExitCode != 0)
    {
        Console.Error.WriteLine(result.Error.Trim());
        return false;
    }

    return true;
}

static string[] Lines(string output)
{
    return output.Split(['\r', '\n'], StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
}

static async Task<string> DockerAsync(params string[] arguments)
{
    var result = await RunDockerAsync(arguments);
    if (result.ExitCode != 0)
    {
        throw new InvalidOperationException($"docker {string.Join(' ', arguments)} failed: {result.Error.Trim()}");
    }

    return result.Output;
}

static async Task<(int ExitCode, string Output, string Error)> RunDockerAsync(params string[] arguments)
{
    var start = new ProcessStartInfo("docker")
    {
        UseShellExecute = false,
        RedirectStandardOutput = true,
        RedirectStandardError = true,
        StandardOutputEncoding = Encoding.UTF8,
        StandardErrorEncoding = Encoding.UTF8
    };
    foreach (var argument in arguments)
    {
        start.ArgumentList.Add(argument);
    }

    using var process = Process.Start(start) ?? throw new InvalidOperationException("Could not start Docker.");
    var output = process.StandardOutput.ReadToEndAsync();
    var error = process.StandardError.ReadToEndAsync();
    await process.WaitForExitAsync();
    return (process.ExitCode, await output, await error);
}
