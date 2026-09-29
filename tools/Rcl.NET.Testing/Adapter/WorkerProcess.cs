using System.Diagnostics;
using System.Text;
using System.Text.Json;

namespace Rcl.NET.Testing;

internal static class WorkerProcess
{
    internal static void Run(string source, Variant variant, Request request, CancellationToken cancellation, Action<Event> receive)
    {
        var temp = Path.Combine(Path.GetTempPath(), "rclnet-tests-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(temp);
        try
        {
            var windows = OperatingSystem.IsWindows();
            var workerDirectory = Path.Combine(Path.GetDirectoryName(Path.GetFullPath(source))!, "ros-worker");
            var requestPath = Path.Combine(temp, "request.json");
            File.WriteAllText(requestPath, JsonSerializer.Serialize(request, Wire.Json));
            var overlaysPath = Path.Combine(temp, "overlays.txt");
            File.WriteAllLines(overlaysPath, variant.Profile.Overlays, new UTF8Encoding(false));
            var start = new ProcessStartInfo(windows ? "cmd.exe" : "/bin/bash")
            {
                WorkingDirectory = workerDirectory,
                UseShellExecute = false,
                CreateNoWindow = true,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                RedirectStandardInput = true
            };
            if (windows)
            {
                start.Arguments = $"/d /s /c \"\"{Path.Combine(workerDirectory, "launch.cmd")}\"\"";
            }
            else
            {
                start.ArgumentList.Add(Path.Combine(workerDirectory, "launch.sh"));
            }

            // A previously selected ROS runsettings must not supply a different profile's state.
            foreach (var key in start.Environment.Keys.ToArray())
            {
                if (key.StartsWith("ROS_") || key.StartsWith("RMW_") || key.StartsWith("AMENT_")
                    || key.StartsWith("COLCON_") || key.StartsWith("RCLNET_") || key == "PYTHONPATH" || key == "LD_LIBRARY_PATH")
                {
                    start.Environment.Remove(key);
                }
            }

            var profile = variant.Profile;
            foreach (var pair in profile.Environment)
            {
                start.Environment[pair.Key] = pair.Value;
            }

            var dotnetRoot = Environment.GetEnvironmentVariable("DOTNET_ROOT");
            var dotnet = windows ? Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ProgramFiles), "dotnet", "dotnet.exe")
                : Path.Combine(dotnetRoot ?? "/usr/share/dotnet", "dotnet");
            start.Environment["RCLNET_DOTNET"] = File.Exists(dotnet) ? dotnet : "dotnet";
            start.Environment["RCLNET_REQUEST"] = requestPath;
            start.Environment["RCLNET_SETUP"] = profile.Setup;
            start.Environment["RCLNET_OVERLAYS"] = overlaysPath;
            start.Environment["RCLNET_RMW"] = variant.Rmw;
            start.Environment["ROS_DISTRO"] = profile.Distro;
            if (profile.PathPrepend.Length > 0)
            {
                start.Environment["PATH"] = string.Join(Path.PathSeparator, profile.PathPrepend) + Path.PathSeparator + start.Environment["PATH"];
            }

            if (windows && profile.PixiManifest.Length > 0)
            {
                start.Environment["RCLNET_PIXI_MANIFEST"] = profile.PixiManifest;
                start.Environment["RCLNET_PIXI_EXE"] = profile.PixiExecutable;
                start.WorkingDirectory = Path.GetDirectoryName(profile.PixiManifest)!;
            }
            else if (profile.Python.Length > 0)
            {
                start.Environment["COLCON_PYTHON_EXECUTABLE"] = profile.Python;
                start.Environment["PATH"] = Path.GetDirectoryName(profile.Python) + Path.PathSeparator + start.Environment["PATH"];
            }

            using var process = new Process { StartInfo = start };
            var diagnostics = new StringBuilder();
            var gate = new object();
            void AddDiagnostic(string line)
            {
                lock (gate)
                {
                    if (diagnostics.Length < 32768)
                    {
                        diagnostics.AppendLine(line);
                    }
                }
            }

            process.ErrorDataReceived += (_, e) =>
            {
                if (e.Data != null)
                {
                    AddDiagnostic(e.Data);
                }
            };
            cancellation.ThrowIfCancellationRequested();
            process.Start();
            process.BeginErrorReadLine();
            void Kill()
            {
                try
                {
                    if (!process.HasExited)
                    {
                        process.Kill(entireProcessTree: true);
                    }
                }
                catch (InvalidOperationException)
                {
                    // The worker may exit between the state check and Kill.
                }
            }

            using var registration = cancellation.Register(Kill);
            var timedOut = 0;
            using var timer = new Timer(_ =>
            {
                Interlocked.Exchange(ref timedOut, 1);
                Kill();
            }, null, TimeSpan.FromSeconds(profile.TimeoutSeconds), Timeout.InfiniteTimeSpan);
            var completed = false;
            try
            {
                while (process.StandardOutput.ReadLine() is { } line)
                {
                    if (!line.StartsWith(Wire.Prefix, StringComparison.Ordinal))
                    {
                        AddDiagnostic(line);
                        continue;
                    }

                    var value = JsonSerializer.Deserialize<Event>(line[Wire.Prefix.Length..], Wire.Json)
                        ?? throw new InvalidDataException("Invalid worker event.");
                    completed |= value.Kind == "complete";
                    receive(value);
                    if (value.Kind == "debug")
                    {
                        process.StandardInput.WriteLine("continue");
                        process.StandardInput.Flush();
                    }
                }

                process.WaitForExit();
                cancellation.ThrowIfCancellationRequested();
                if (Volatile.Read(ref timedOut) != 0)
                {
                    throw new TimeoutException($"Profile {variant.Key} exceeded {profile.TimeoutSeconds}s. {diagnostics}");
                }

                if (process.ExitCode != 0 || !completed)
                {
                    throw new InvalidOperationException($"Profile {variant.Key}: worker exited with code {process.ExitCode}, completed={completed}. {diagnostics}");
                }
            }
            finally
            {
                Kill();
                process.WaitForExit();
            }
        }
        finally
        {
            Directory.Delete(temp, recursive: true);
        }
    }
}
