# Testing

Use the .NET 10 SDK. Tests target .NET 8, 9 and 10; install their runtimes or select one with `-f net10.0`. Run commands from the repository root unless shown otherwise. Use Release for full validation; Debug skips allocation-sensitive tests.

## Windows

### 1. Install ROS

Follow the official Windows installation guide, including dependencies and PATH setup: [Foxy](https://docs.ros.org/en/foxy/Installation/Windows-Install-Binary.html), [Humble](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html), [Iron](https://docs.ros.org/en/iron/Installation/Windows-Install-Binary.html), [Jazzy](https://docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html), [Kilted](https://docs.ros.org/en/kilted/Installation/Windows-Install-Binary.html), or [Lyrical](https://docs.ros.org/en/lyrical/Installation/Windows-Install-Binary.html).

### 2. Build the native test interfaces

Open an x64 Visual Studio Developer Command Prompt and activate the distribution's dependency environment. Ensure colcon, CMake and Ninja are available. Adjust these example paths:

```bat
set PYTHONUTF8=1
call C:\ros\lyrical\setup.bat
if not exist C:\ros\test-workspaces\lyrical mkdir C:\ros\test-workspaces\lyrical
cd /d C:\ros\test-workspaces\lyrical
colcon build --merge-install --packages-select ros2cs_abi_test_msgs --base-paths C:\dev\rclnet\src\CodegenTests\packages\ros2cs_abi_test_msgs --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release
```

Use a separate workspace for each distribution. Rebuild it when the test interface definitions change; `dotnet build` does not rebuild native interfaces.

### 3. Configure local environments

Create `src/Rcl.NET.Tests/ros-environments.local.json`, or copy `ros-environments.example.json` from the same directory. This file is gitignored. Windows profiles are disabled in the shared configuration; enable the ones you have installed.

**Legacy: Foxy, Humble, Iron**

After the official installation, only the ROS setup and test overlay paths are needed:

```json
{
  "version": 1,
  "profiles": [
    {
      "id": "windows-humble",
      "enabled": true,
      "setup": "C:\\ros\\humble\\setup.bat",
      "overlays": [ "C:\\ros\\test-workspaces\\humble\\install\\local_setup.bat" ]
    }
  ]
}
```

`setup` is in the ROS installation; `overlays` points to the `local_setup.bat` built in step 2. For Foxy or Iron, change the ID and paths accordingly. Restart Visual Studio after installation or PATH changes.

**Pixi: Jazzy, Kilted, Lyrical**

Point `pixiManifest` at the dependency workspace prepared during installation:

```json
{
  "version": 1,
  "profiles": [
    {
      "id": "windows-lyrical",
      "enabled": true,
      "setup": "C:\\ros\\lyrical\\setup.bat",
      "overlays": [ "C:\\ros\\test-workspaces\\lyrical\\install\\local_setup.bat" ],
      "pixiManifest": "C:\\ros\\dependencies\\lyrical\\pixi.toml"
    }
  ]
}
```

If Pixi is not on PATH, add `"pixiExecutable": "C:\\path\\to\\pixi.exe"`. For Jazzy or Kilted, use that distribution's ID, setup, overlay and Pixi workspace.

Add multiple profiles to the same array to test multiple distributions. Other fields, including the Fast DDS and Cyclone DDS selection, inherit from the shared configuration.

### 4. Run tests

Start Visual Studio normally, open `src/rclnet.sln`, build, and run tests in Test Explorer using the local Windows environment.

## Linux / WSL

Install ROS and its build dependencies, then build the native test interfaces. Example for Humble:

```bash
source /opt/ros/humble/setup.bash
sudo install -d -o "$(id -un)" -g "$(id -gn)" /opt/rclnet-test-ws
cd /opt/rclnet-test-ws
colcon build --merge-install --packages-select ros2cs_abi_test_msgs \
  --base-paths /absolute/path/to/rclnet/src/CodegenTests/packages/ros2cs_abi_test_msgs
```

With ROS under `/opt/ros/<distro>` and this overlay location, no local configuration is needed. Discovery detects installed RMW packages from the configured list.

For multiple distributions or a custom workspace location, build a separate overlay per distribution and set its path in `src/Rcl.NET.Tests/ros-environments.local.json`:

```json
{
  "version": 1,
  "profiles": [
    {
      "id": "linux-humble",
      "overlays": [ "/home/your-user/rclnet-test-ws/humble/install/local_setup.bash" ]
    }
  ]
}
```

Use absolute paths; `$HOME` and `~` are not expanded. Override `setup` too if ROS is installed outside `/opt/ros/<distro>`.

Run `dotnet test` as shown below, or select the WSL test environment in Visual Studio.

## Docker through Visual Studio

1. Open `src/rclnet.sln` and build.
2. Select `Container - <distro>` and run tests.

The environments in `src/testEnvironments.json` include ROS, .NET and the native test interfaces. No host ROS installation or local profile configuration is needed.

### Clean up Visual Studio test containers and images

Switching container environments in Visual Studio can leave behind unused containers and images. Use this command to quickly clean them up when needed:

```bash
dotnet tools/clean_vsut.cs
```

## Run and select tests

```powershell
dotnet test src/Rcl.NET.Tests/Rcl.NET.Tests.csproj -c Release -f net10.0
```

Omit `-f net10.0` to run all target frameworks. Each test appears for the available ROS/RMW combinations. In Test Explorer, filter by one `RosEnvironment` trait, such as `windows-lyrical/rmw_fastrtps_cpp`. Selecting separate `RosDistro` and `RMW` traits forms a union, not an intersection.

To combine filters on the command line:

```powershell
dotnet test src/Rcl.NET.Tests/Rcl.NET.Tests.csproj -c Release -f net10.0 --filter 'RosProfile=windows-lyrical&RMW=rmw_cyclonedds_cpp&FullyQualifiedName~ContextLifecycleTests'
```

For other ROS-related projects, activate the dependency environment and source ROS plus the test overlay in your shell first:

```powershell
dotnet test src/Rosidl.Runtime.Tests/Rosidl.Runtime.Tests.csproj -c Release
dotnet test src/CodegenTests/CodegenTests.csproj -c Release
```

The test infrastructure suite needs no ROS installation:

```powershell
dotnet test tools/Rcl.NET.Testing/Tests/Rcl.NET.Testing.Tests.csproj -c Release
```

## Troubleshooting

- **No tests or missing variants:** check `enabled`, setup/overlay paths and installed RMWs; rebuild and rediscover after changing configuration.
- **Missing native test DLL or entry point:** rebuild the native test interfaces for that distribution and check `overlays`.
- **Missing Python or dependency DLL:** check the official PATH setup or the selected Pixi environment. Restart Visual Studio after PATH changes.
- **Timeout while running or debugging:** increase `timeoutSeconds`. Use Visual Studio's Debug Test command to debug tests.
- **Concurrent test sessions interfere:** assign different `ROS_DOMAIN_ID` values through each profile's `environment` field.

## CI, coverage and blame

Build and test with `-p:UseRosTestVariants=false` to use the standard xUnit adapter. Prepare ROS and the native test overlay in the calling shell first. Use the same property for build and test, and rebuild when switching modes.

Ordinary requests, discovery and synchronization waits have no test-side watchdog timeout. Keep finite timeouts only when they are part of the behavior under test, including cancellation, result retention and allocation measurements of timer setup. CI uses `blame.runsettings` to collect a mini dump after one minute of test inactivity before terminating the test host.

On Humble and Iron with Fast DDS, tests that keep nodes or endpoints alive in multiple contexts skip because native endpoint teardown can crash in `StatefulWriter::deliver_sample_to_intraprocesses`. This includes queued action shutdown, retained native node lifetime, and concurrent timer creation/disposal across contexts. Context-only tests and tests whose endpoints all belong to one context remain enabled.

Local runs do not enable dump collection by default. Debug a hanging test, or opt in when rerunning it:

```powershell
dotnet test src/Rcl.NET.Tests/Rcl.NET.Tests.csproj -c Release -f net9.0 -p:UseRosTestVariants=false --settings src/Rcl.NET.Tests/blame.runsettings
```

## Configuration reference

`ros-environments.local.json` overrides supplied fields of matching IDs in `ros-environments.json`; omitted fields inherit. Arrays and dictionaries replace the whole field. Use `[]`, `{}` or `""` to clear values; `null` is invalid. Rebuild after configuration changes.

| Field | Meaning |
| --- | --- |
| `id` | Profile ID, such as `windows-humble` |
| `enabled` | Enable or disable the profile |
| `os`, `distro` | Platform and ROS distribution; inherited for existing IDs |
| `setup` | Absolute path to the ROS setup script |
| `overlays` | Ordered absolute paths to overlay setup scripts |
| `rmw` | Override to limit which RMW implementations are tested; with `autoDetect: true`, unavailable packages are filtered out |
| `pixiManifest` | Path to an existing Pixi workspace manifest |
| `pixiExecutable` | Pixi executable; defaults to `pixi.exe` on PATH |
| `python`, `pathPrepend` | Optional interpreter and extra DLL directories for custom installations |
| `environment` | Additional environment variables, e.g. `ROS_DOMAIN_ID` |
| `autoDetect` | Skip missing setup/overlays and unavailable RMW packages; defaults to false |
| `timeoutSeconds` | Setup and test timeout; defaults to 600 |
