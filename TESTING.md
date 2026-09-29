# Testing

This guide explains how to prepare the test environment, build the native test interfaces, and run rclnet tests on Windows, Linux/WSL, or in Visual Studio Docker environments. Run repository-relative commands from the repository root unless a step explicitly changes directory.

## Prerequisites and test projects

Use the .NET 10 SDK. `Rcl.NET.Tests` targets .NET 8, 9 and 10; install the matching runtimes to run all targets, or select one with `-f net10.0`. The existing Docker test images already include the SDK and these runtimes.

| Project | Purpose |
| --- | --- |
| `src/Rcl.NET.Tests` | ROS integration, ABI, lifecycle and concurrency tests; use the platform setup below |
| `src/Rosidl.Runtime.Tests` | ROSIDL runtime tests |
| `src/CodegenTests` | Generated C# interface tests |
| `tools/Rcl.NET.Testing/Tests` | Test infrastructure regression tests; no ROS installation required |

Use Release for full validation. Debug is also supported; allocation-sensitive tests are explicitly skipped in Debug. Existing tests may skip unsupported ROS/RMW features, including the repository's Foxy/Cyclone DDS exclusion.

## Choose an execution environment

| Execution environment | Local profile configuration | Native test interfaces |
| --- | --- | --- |
| Windows | Required: ROS and dependency paths vary by installation | Build with colcon for each ROS distribution |
| Linux / WSL, standard `/opt/ros/<distro>` installation | Not needed when using the default test overlay location | Build with colcon into `/opt/rclnet-test-ws/install` |
| Existing VS Docker test environment | Not needed | Already built by the Dockerfile |

`dotnet build` generates C# bindings, but does not compile the native message/service/action support libraries. The package `src/CodegenTests/packages/ros2cs_abi_test_msgs` must also be built with colcon. Rebuild it when its definitions change. Use separate native workspaces for different ROS distributions; the two RMW variants of one distribution can share a workspace.

## Windows

### 1. Prepare the selected ROS distribution's dependencies

Open an x64 Visual Studio Developer Command Prompt. For a Pixi-based installation, enter the corresponding dependency workspace and activate it. For example:

```bat
cd /d C:\ros\dependencies\lyrical
C:\ros\tools\pixi.exe shell
```

The dependency environment must already provide Python, colcon, CMake, Ninja and the ROS interface-generation dependencies. Use the dependency definition supplied for the selected ROS distribution. Do not share environments created from different ROS distributions' dependency definitions. Installations without Pixi can use their existing dependency environment instead.

### 2. Compile the native test interfaces

The examples use `C:\dev\rclnet` for the repository and `C:\ros` for ROS installations and dependency workspaces. Replace these paths with your own. In the activated command prompt, run:

```bat
set PYTHONUTF8=1
call C:\ros\lyrical\setup.bat
if not exist C:\ros\test-workspaces\lyrical mkdir C:\ros\test-workspaces\lyrical
cd /d C:\ros\test-workspaces\lyrical
colcon build --merge-install --packages-select ros2cs_abi_test_msgs --base-paths C:\dev\rclnet\src\CodegenTests\packages\ros2cs_abi_test_msgs --cmake-args -G Ninja -DCMAKE_BUILD_TYPE=Release
```

`PYTHONUTF8=1` avoids locale-dependent Python template decoding errors. Repeat with a separate workspace for every distribution you want to test.

### 3. Configure this machine's profiles

Copy `src/Rcl.NET.Tests/ros-test-profiles.example.json` to `src/Rcl.NET.Tests/ros-test-profiles.local.json`. For each Windows profile, set:

- `setup`: the distribution's `setup.bat`.
- `overlays`: the `install/local_setup.bat` produced by step 2.
- `pixiManifest` and, if necessary, `pixiExecutable`: the matching dependency workspace and Pixi executable.
- For a non-Pixi installation, omit `pixiManifest` and supply `python` / `pathPrepend` only where needed.
- `rmw`: the installed RMW implementations you want to test.

This file is gitignored. The repository example includes a Lyrical Pixi profile and a disabled Humble direct-setup example; adapt them to your installation.

### 4. Build and run

Open `src/rclnet.sln`, ensure no ROS environment-injecting `.runsettings` is selected, and build. Choose the local Windows test environment in Test Explorer and run the desired variants. Use the Traits column to filter the available combinations.

The worker activates Pixi when configured, calls ROS setup and the overlays, then sets the selected RMW. You do not need to start Visual Studio from a Pixi shell or manually source setup before each run. Pixi uses `run --as-is`, so testing does not install dependencies or update its lockfile.

## Linux / WSL

### 1. Use the standard ROS installation

For ROS installed under `/opt/ros/<distro>`, **no local JSON configuration, Pixi configuration or Python path is required for the base ROS environment**. The shared `ros-test-profiles.json` automatically detects installed setup scripts and supplies the Fast DDS / Cyclone DDS variants. Ensure both RMW implementations and the ROS build dependencies are installed, or override the profile to list only the RMW implementations you use.

### 2. Compile the native test interfaces at the default location

For an environment containing one ROS distribution, use `/opt/rclnet-test-ws` to match the shared profile without editing configuration. The following Humble example creates a dedicated workspace writable by the current user; adjust the repository path:

```bash
source /opt/ros/humble/setup.bash
sudo install -d -o "$(id -un)" -g "$(id -gn)" /opt/rclnet-test-ws
cd /opt/rclnet-test-ws
colcon build --merge-install --packages-select ros2cs_abi_test_msgs \
  --base-paths /absolute/path/to/rclnet/src/CodegenTests/packages/ros2cs_abi_test_msgs
```

After a successful build, `/opt/rclnet-test-ws/install/local_setup.bash` matches the default profile, so no local profile configuration is needed.

If you prefer a workspace in your home directory, or test multiple ROS distributions in the same Linux installation, use a separate workspace per distribution. Only in that case, create `ros-test-profiles.local.json` to override each corresponding `linux-<distro>` profile's overlay location. An override replaces the entire profile, so retain its other fields. For example:

```json
{
  "version": 1,
  "profiles": [
    {
      "id": "linux-humble",
      "os": "linux",
      "distro": "humble",
      "setup": "/opt/ros/humble/setup.bash",
      "overlays": ["/home/your-user/rclnet-test-ws/humble/install/local_setup.bash"],
      "rmw": ["rmw_fastrtps_cpp", "rmw_cyclonedds_cpp"]
    }
  ]
}
```

Use actual absolute paths: JSON does not expand `$HOME` or `~`. A nonstandard ROS installation also needs an override for `setup`.

### 3. Build and run

From the repository root:

```bash
dotnet test src/Rcl.NET.Tests/Rcl.NET.Tests.csproj -c Release -f net10.0
```

For WSL through Visual Studio, build the solution and choose the corresponding WSL test environment. The adapter runs the base setup and overlay inside Linux automatically. Keep ROS environment-injecting `.runsettings` unselected; switching between Windows and Linux then uses the environment selector and variants.

## Docker through Visual Studio

1. Open `src/rclnet.sln` and build it.
2. Ensure no ROS environment-injecting `.runsettings` is selected.
3. Select the desired `Container - <distro>` environment and run its variants.

`src/testEnvironments.json` defines a Docker test environment for each ROS distribution using its Dockerfile. The Dockerfiles install dependencies and run colcon for `ros2cs_abi_test_msgs` under `/opt/rclnet-test-ws/install`. No Windows ROS installation, local profile file or manual colcon step is required for container testing.

## Run and select tests

After completing the platform-specific setup above, run the ROS integration tests from the repository root:

```powershell
dotnet test src/Rcl.NET.Tests/Rcl.NET.Tests.csproj -c Release -f net10.0
```

Omit `-f net10.0` to run all three target frameworks. The project runs its target frameworks sequentially. In Visual Studio, open `src/rclnet.sln`, choose the build configuration and test environment, build, then use Test Explorer.

The same test appears for each available ROS/RMW combination. In Test Explorer, clear the Traits selections and select one `RosEnvironment` value, such as `windows-lyrical/rmw_fastrtps_cpp`, to show exactly that combination. Selecting multiple values includes their union. Selecting `RosDistro` and `RMW` separately in this menu also forms a union, not an intersection. You can alternatively filter from the command line:

```powershell
dotnet test src/Rcl.NET.Tests/Rcl.NET.Tests.csproj -c Release -f net10.0 --filter 'RosProfile=windows-lyrical&RMW=rmw_cyclonedds_cpp&FullyQualifiedName~ContextLifecycleTests'
```

For Linux, replace `windows-lyrical` with the appropriate `linux-<distro>` profile. Selected combinations run in separate processes.

To run the other ROS-related projects, first activate the appropriate dependency environment and source ROS plus the native interface overlay in your shell, then run:

```powershell
dotnet test src/Rosidl.Runtime.Tests/Rosidl.Runtime.Tests.csproj -c Release
dotnet test src/CodegenTests/CodegenTests.csproj -c Release
```

Automatic ROS profile setup applies to `Rcl.NET.Tests`; these other projects use their existing test runners and inherit the shell environment.

The test infrastructure can be checked without ROS:

```powershell
dotnet test tools/Rcl.NET.Testing/Tests/Rcl.NET.Testing.Tests.csproj -c Release
```

The adjacent `Fixture` project is input for those regression tests, not a test suite to run directly.

## Troubleshooting and debugging

- **No ROS profiles found:** on Windows, check the local profile file and rebuild; on Linux, check the standard setup path or provide an override.
- **Missing native test DLL or entry point:** rebuild `ros2cs_abi_test_msgs` with colcon for that distribution, and check `overlays` points to the resulting install directory. A .NET build does not update native interfaces.
- **Python or dependency DLL not found on Windows:** check the distribution-specific Pixi workspace or direct-setup dependency paths. Ensure the environment was created from the selected ROS distribution's dependency definition.
- **Profiles changed but the test list is stale:** rebuild and rediscover tests. If an old selected test no longer exists, select it again from the refreshed list.
- **Worker timeout:** `timeoutSeconds` covers environment setup, discovery and execution. Increase it for long runs or debugging. Cancellation terminates the active worker process tree.

Debug Test asks Visual Studio to attach to the worker before test code runs. If the platform cannot attach, the run reports an error.

Combinations run sequentially within one test run. When running multiple VS or CLI sessions concurrently, assign different `ROS_DOMAIN_ID` values to keep their DDS communication separate.

## CI and the standard xUnit runner

The CI matrix selects one ROS/RMW environment per job and uses testhost blame diagnostics. It builds and runs with `-p:UseRosTestVariants=false` to use the standard xUnit Visual Studio adapter. Use the same property for both build and test, and rebuild after switching modes.

In this mode, prepare the ROS environment and native interface overlay in the calling shell before running tests; no profile configuration is required. Use this mode for coverage or blame tooling that requires tests to execute in the standard testhost. `Rosidl.Runtime.Tests` and `CodegenTests` use their standard runners independently of this setting.

## Profile reference

Shared profiles live in `ros-test-profiles.json`; optional machine-local overrides live in `ros-test-profiles.local.json`. Local profiles replace shared profiles with the same ID. Rebuild and rediscover tests after changing or removing local configuration. Configuration version is 1.

| Field | Meaning |
| --- | --- |
| `id` | Unique, stable profile name used in test identity |
| `os` | `windows` or `linux` |
| `distro` | ROS distribution |
| `setup` | Absolute path to the ROS setup script |
| `overlays` | Ordered absolute setup paths, including the test interface workspace |
| `rmw` | List of RMW implementations to expose |
| `pixiManifest` | Optional Windows dependency workspace manifest |
| `pixiExecutable` | Optional Pixi executable path; defaults to `pixi.exe` on PATH |
| `python` | Optional Python executable for a non-Pixi installation |
| `pathPrepend` | Additional native dependency directories |
| `environment` | Extra worker environment variables, e.g. `ROS_DOMAIN_ID` |
| `enabled` | Defaults to true; false disables a profile |
| `autoDetect` | Defaults to false; true skips a profile whose base setup is absent |
| `timeoutSeconds` | Worker timeout including setup, discovery and execution; defaults to 600 |

You can use `.runsettings` for options such as loggers. Configure ROS environment variables through profiles. For coverage and blame diagnostics, use the standard xUnit mode described above.
