# rclnet

**🚀Modern ROS 2 for .NET.**

rclnet is a high-performance, asynchronous .NET client library for ROS 2, designed to feel like a native part of the .NET ecosystem rather than a thin port of the C++ API.

* **One assembly, multiple ROS 2 distributions** — run the same application binaries across supported ROS 2 releases.
* **NuGet and MSBuild first** — use ROS 2 from a normal .NET project with interface generation integrated into `dotnet build`.
* **Async by design** — ROS communication integrates naturally with `async`/`await`, `Task`, and `IAsyncEnumerable<T>`.
* **High-level and low-level APIs** — use managed messages or access native message buffers directly when performance matters.
* **Cross-platform by default** — use the same managed APIs and assemblies across Linux and Windows.

## What's New in 3.0
- ROS 2 Lyrical support
- Portable ROSIDL ABI support
- Incremental MSBuild support for generated interfaces

## ROS 2 Feature Support
| Feature                 | Support | Notes                                                     |
| ----------------------- | ------- | --------------------------------------------------------- |
| Topics                  | ✅       |                                                           |
| Services                | ✅       |                                                           |
| Actions                 | ✅       | Managed implementation                                    |
| Clocks & Timers         | ✅       | `use_sim_time` supported                                  |
| Guard Conditions        | ✅       |                                                           |
| ROS Graph               | ✅       | Managed graph API                                         |
| Logging                 | ✅       | stdout, `/rosout`, log files                              |
| [Content Filtered Topics](https://github.com/ros2/design/blob/918c09758ed4c0854aa128b9c8ed0051c21a6590/articles/content_filtering.md) | ✅       | Humble+                                                   |
| [Network Flow Endpoints](https://design.ros2.org/articles/unique_network_flows.html)  | ✅       | Humble+                                                 |
| [Service Introspection](https://github.com/ros-infrastructure/rep/blob/jacob/service_introspection/rep-2012.rst)   | ✅       | Iron+                                                     |
| [Parameters](https://design.ros2.org/articles/ros_parameters.html)              | ⚠️       | Local parameters and parameter files; no parameter client |
| [Lifecycle Nodes](https://design.ros2.org/articles/node_lifecycle.html)         | ❌       |                                                           |

✅Supported ⚠️Partial support ❌Not supported ⏳In development

## Supported Platforms

### ROS 2 distributions

| Distribution | ROSIDL ABI | Support |
| --- | --- | --- |
| Lyrical Luth | V2 | ✅ Supported |
| Kilted Kaiju | V1 | ✅ Supported |
| Jazzy Jalisco | V1 | ✅ Supported |
| Humble Hawksbill | V1 | ✅ Supported |
| Iron Irwini | V1 | 🟡 Legacy |
| Foxy Fitzroy | V1 | 🟡 Legacy |

Legacy distributions remain compatible where practical, but new features may require a currently supported ROS 2 distribution.

### .NET and operating systems

- **.NET:** 8, 9, 10
- **Operating systems:** Linux and Windows

macOS is not currently tested or officially supported.

## Installing

Install rclnet from NuGet:

```bash
dotnet add package Rcl.NET
```

rclnet uses the native ROS 2 runtime installed on the target machine. The NuGet package contains the managed .NET libraries, but does not bundle ROS 2 itself.

## Quick Start

### Project setup

Generated ROS interface types use unsafe native interop. Enable unsafe code in your project:

```xml
<PropertyGroup>
  <AllowUnsafeBlocks>true</AllowUnsafeBlocks>
</PropertyGroup>
```

Create an `AssemblyInfo.cs` file in the project and add:

```csharp
[assembly: System.Runtime.CompilerServices.DisableRuntimeMarshalling]
```

Create a `ros2cs.spec` file in the project directory and select the ROS interface packages used by the examples below:

```text
from-ament-index

include geometry_msgs
include sensor_msgs
include std_srvs
include example_interfaces
```

`ros2cs` runs automatically during build and generates the corresponding C# message, service and action types.

When using `from-ament-index`, build from a configured ROS 2 environment.

Linux:

```bash
source /opt/ros/lyrical/setup.bash
dotnet build
```

Windows:

```bat
call C:\dev\ros2_lyrical\ros2-windows\setup.bat
dotnet build
```

Replace the ROS distribution and installation path with those installed on your machine.

### Create a node

```csharp
await using var context = new RclContext(args);
using var node = context.CreateNode("my_node");
```

Publishers, subscriptions, services and actions are created from the node.

### Publish and subscribe

Create a publisher:

```csharp
using var publisher =
    node.CreatePublisher<Vector3>("/vector");

publisher.Publish(new Vector3(
    x: 1,
    y: 2,
    z: 3));
```

Create a subscription:

```csharp
using var subscription =
    node.CreateSubscription<Vector3>("/vector");

await foreach (var message in subscription.ReadAllAsync())
{
    Console.WriteLine(
        $"{message.X}, {message.Y}, {message.Z}");
}
```

`ReadAllAsync` returns an `IAsyncEnumerable<T>`, so normal .NET asynchronous code can be used directly:

```csharp
await foreach (var message in subscription.ReadAllAsync(cancellationToken))
{
    await ProcessMessageAsync(message, cancellationToken);
}
```

### QoS

QoS can be configured through `PublisherOptions` and `SubscriptionOptions`.

For example, sensor topics commonly use `QosProfile.SensorData`:

```csharp
using var subscription = node.CreateSubscription<LaserScan>(
    "/scan",
    new SubscriptionOptions(
        qos: QosProfile.SensorData));
```

Publisher and subscription QoS settings must be compatible for the endpoints to communicate.

### Services

Create a service:

```csharp
using var server = node.CreateService<
    EmptyService,
    EmptyServiceRequest,
    EmptyServiceResponse>(
        "/reset",
        (request, state) =>
        {
            ResetSomething();
            return new EmptyServiceResponse();
        });
```

`CreateService` handlers run synchronously on the `RclContext` event loop. If a handler performs asynchronous I/O or may take a significant amount of time, use `CreateConcurrentService` instead:

```csharp
using var server = node.CreateConcurrentService<
    EmptyService,
    EmptyServiceRequest,
    EmptyServiceResponse>(
        "/reset",
        async (request, state, cancellationToken) =>
        {
            await ResetSomethingAsync(cancellationToken);
            return new EmptyServiceResponse();
        });
```

Concurrent handlers start on the event loop, but multiple requests can remain in progress while the handlers are awaiting. Blocking or CPU-intensive work should be offloaded with `Task.Run` instead of blocking the event loop.

Call a service:

```csharp
using var client = node.CreateClient<
    EmptyService,
    EmptyServiceRequest,
    EmptyServiceResponse>("/reset");

await client.WaitForServerAsync();

var response = await client.InvokeAsync(
    new EmptyServiceRequest());
```

### Actions

Create an action client:

```csharp
using var client = node.CreateActionClient<
    FibonacciAction,
    FibonacciActionGoal,
    FibonacciActionResult,
    FibonacciActionFeedback>("/fibonacci");

await client.WaitForServerAsync();
```

Send a goal:

```csharp
using var goal = await client.SendGoalAsync(
    new FibonacciActionGoal(order: 10));
```

Read feedback:

```csharp
await foreach (var feedback in goal.ReadFeedbacksAsync())
{
    Console.WriteLine(
        string.Join(", ", feedback.PartialSequence));
}
```

Wait for the result:

```csharp
var result = await goal.GetResultAsync();

Console.WriteLine(
    string.Join(", ", result.Sequence));
```

Action servers can be created with `CreateActionServer`.

### ROS graph

The discovered ROS graph is available through `node.Graph`.

For example, wait for a service:

```csharp
await node.Graph.WaitForServiceServerAsync("/my/service");
```

Or observe graph changes:

```csharp
node.Graph
    .OfType<NodeAppearedEvent>()
    .Subscribe(e =>
    {
        Console.WriteLine(
            $"Node {e.Node.Name} is online.");
    });
```

## Running and Debugging

rclnet loads ROS 2 native libraries at runtime, so the application must inherit a configured ROS environment.

### Command line

Linux:

```bash
source /opt/ros/lyrical/setup.bash
dotnet run
```

Windows:

```bat
call C:\dev\ros2_lyrical\ros2-windows\setup.bat
dotnet run
```

Optional ROS settings can be configured in the same shell:

```bat
set ROS_DOMAIN_ID=10
set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
dotnet run
```

Simply setting `ROS_DISTRO` is not sufficient. The ROS setup script also configures native library search paths and other runtime settings.

### Visual Studio

Visual Studio inherits its environment when it starts.

If Visual Studio is launched normally, debugging an rclnet application may fail with `DllNotFoundException` or errors about loading `rcl`, an RMW implementation, or a type support library.

Start Visual Studio from a configured ROS shell instead:

```bat
call C:\dev\ros2_lyrical\ros2-windows\setup.bat
devenv MySolution.sln
```

Applications launched with F5 will then inherit the ROS environment.

If Visual Studio was already running, close it and restart it from the configured shell.

### VS Code

The same applies to VS Code.

Linux:

```bash
source /opt/ros/lyrical/setup.bash
code .
```

Windows:

```bat
call C:\dev\ros2_lyrical\ros2-windows\setup.bat
code .
```

## ROS Interface Code Generation

`ros2cs` converts ROS `.msg`, `.srv` and `.action` definitions into C# types.

A `ros2cs.spec` file in the project directory is detected automatically by the `Rcl.NET` MSBuild integration.

### Using an installed ROS environment

The simplest configuration is:

```text
from-ament-index

include geometry_msgs
include sensor_msgs
include std_srvs
```

`from-ament-index` searches interface packages available through the current `AMENT_PREFIX_PATH`.

Package dependencies are resolved automatically, so dependencies such as `builtin_interfaces` usually do not need to be listed explicitly.

Without any `include` directive, all discovered interface packages are generated.

### Loading packages from directories

Interface packages can also be loaded directly:

```text
from-directory ./ros-packages

include geometry_msgs
include my_robot_msgs
```

The specified directory should contain normal ROS packages:

```text
ros-packages/
├── geometry_msgs/
│   ├── package.xml
│   └── msg/
└── my_robot_msgs/
    ├── package.xml
    ├── msg/
    ├── srv/
    └── action/
```

This allows C# bindings to be generated without a ROS installation on the build machine.

It does not replace native ROS type support. At runtime, the corresponding ROS interface packages must still be built and installed in the ROS environment.

`from-directory` is mainly useful for CI, reproducible builds and shared message assemblies.

It can also be combined with `from-ament-index`:

```text
from-ament-index
from-directory ./my-ros-packages

include geometry_msgs
include my_robot_msgs
```

### MSBuild integration

Generated sources are written to the intermediate output directory, normally:

```text
obj/Ros2csGeneratedInterfaces/
```

and are included in the compilation automatically.

Changes to relevant interface files or `package.xml` files trigger regeneration on the next build.

A different spec file can be selected with:

```xml
<PropertyGroup>
  <Ros2csSpecFile>path/to/ros2cs.spec</Ros2csSpecFile>
</PropertyGroup>
```

Additional command-line arguments can be passed with:

```xml
<PropertyGroup>
  <Ros2csArgs>--abi=v1</Ros2csArgs>
</PropertyGroup>
```

Command-line options override values specified in `ros2cs.spec`.

### ROSIDL native ABI

`ros2cs` uses portable ABI mode by default:

```text
abi portable
```

Currently supported layouts are:

```text
v1    ROS 2 Foxy through Kilted
v2    ROS 2 Lyrical
```

Portable mode generates both layouts and automatically selects the correct one when using the normal managed APIs.

If an application only targets one ABI, generation can be restricted to:

```text
abi v1
```

or:

```text
abi v2
```

The ABI can also be selected from the active ROS distribution at generation time:

```text
abi native
```

Native mode reads `ROS_DISTRO`, generates only the corresponding ABI layout, and fails if
the variable is missing or names an unsupported distribution.

Portable output exposes:

```text
Priv / PrivSequence        ABI V1
PrivV2 / PrivSequenceV2    ABI V2
```

These types only matter when using the low-level native message APIs.

Lyrical `rosidl::Buffer`-backed non-CPU sequences are not currently supported.

### Standalone ros2cs

`ros2cs` can also be installed as a standalone .NET tool:

```bash
dotnet tool install -g ros2cs
```

Generate interfaces with:

```bash
ros2cs /path/to/ros2cs.spec
```

Run:

```bash
ros2cs --help
```

for the complete list of options.

## Native Message Buffers

The normal rclnet APIs convert ROS messages into managed .NET objects and should be preferred for most application code.

For performance-sensitive paths, native message buffers can be consumed directly:

```csharp
using var subscription =
    node.CreateNativeSubscription<Twist>("/cmd_vel");

await foreach (RosMessageBuffer buffer in subscription.ReadAllAsync())
{
    using (buffer)
    {
        ProcessMessage(buffer);
    }
}
```

When using portable message bindings, select the native structure matching the active ROSIDL ABI:

```csharp
static void ProcessMessage(RosMessageBuffer buffer)
{
    if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
    {
        ref var message =
            ref buffer.AsRef<Twist.Priv>();

        Console.WriteLine(message.Linear.X);
    }
    else
    {
        ref var message =
            ref buffer.AsRef<Twist.PrivV2>();

        Console.WriteLine(message.Linear.X);
    }
}
```

Raw `AsRef<T>()` access does not perform ABI conversion or validation.

## Execution and Scheduling

Unlike `rclcpp` and `rclpy`, rclnet does not require applications to explicitly spin an executor.

Each `RclContext` owns a dedicated event loop that waits for ROS events and dispatches work. A single context is sufficient for most applications, although multiple contexts can be created when needed.

### Synchronous callbacks

Synchronous callbacks are executed directly on the `RclContext` event loop.

This includes synchronous subscription observers and other synchronous event handlers. These callbacks should therefore finish quickly and avoid blocking operations or long-running CPU work.

For example:

```csharp
using var subscription = node
    .CreateSubscription<Twist>("/cmd_vel")
    .Subscribe(message =>
    {
        // Runs on the RclContext event loop.
        ProcessMessage(message);
    });
```

If expensive synchronous work is required, offload it instead of blocking the event loop.

### Asynchronous APIs

Asynchronous APIs allow the event loop to remain focused on receiving and dispatching ROS events while application code uses normal .NET asynchronous scheduling.

For example:

```csharp
using var subscription =
    node.CreateSubscription<Twist>("/cmd_vel");

await foreach (var message in subscription.ReadAllAsync())
{
    await ProcessMessageAsync(message);
}
```

The ROS message is received by the `RclContext` event loop, but rclnet does not normally execute the asynchronous consumer on that event loop. Continuation scheduling follows the normal .NET rules for the current `SynchronizationContext` and `TaskScheduler`.

In a typical console or server application, asynchronous processing therefore normally continues on thread-pool threads. In a GUI application, an existing UI `SynchronizationContext` may instead cause continuations to resume on the UI thread.

Subscription delivery queues disable synchronous continuations by default. For performance-sensitive workloads, this can be changed with `SubscriptionOptions.AllowSynchronousContinuations`, allowing a waiting consumer to continue directly from the event loop.

### Running asynchronous code on the event loop

For applications that benefit from single-threaded cooperative concurrency, an `RclContext` can install a `SynchronizationContext` on its event loop:

```csharp
await using var context =
    new RclContext(useSynchronizationContext: true);

await context.Yield();

// Running on the RclContext event loop.

await SomeAsyncOperation();

// Resumes on the event loop by default.
```

Once execution has entered the context with `Yield()`, normal `await` expressions capture the context and resume on the event loop by default.

This can be useful when multiple asynchronous operations need to access shared application state without explicit locking.

CPU-intensive or blocking work can still be moved to the thread pool:

```csharp
await Task.Run(() =>
{
    ProcessCpuIntensiveWork();
});

// Back on the RclContext event loop.
```

Execution can explicitly leave the event loop with either `ConfigureAwait(false)` or `RclContext.YieldBackground()`:

```csharp
await SomeAsyncOperation().ConfigureAwait(false);

// Running outside the RclContext event loop.

await context.Yield();

// Back on the event loop.

await RclContext.YieldBackground();

// Running on a background thread.
```

`context.Yield()` always transitions execution to that context's event loop, regardless of the currently captured `SynchronizationContext`.

### Low-level wait objects

Timers, guard conditions, and other `IRclWaitObject` implementations expose `WaitOneAsync` for directly awaiting an RCL event.

The default overload forces the continuation to run asynchronously:

```csharp
await waitObject.WaitOneAsync(cancellationToken);
```

Low-level code can instead allow the continuation to execute directly on the `RclContext` event loop:

```csharp
await waitObject
    .WaitOneAsync(
        runContinuationAsynchronously: false,
        cancellationToken)
    .ConfigureAwait(false);
```

The two scheduling controls interact as follows:

| `runContinuationAsynchronously` | Await behavior          | Continuation runs on                                                                    |
| ------------------------------- | ----------------------- | --------------------------------------------------------------------------------------- |
| `true`                          | Normal `await`          | Captured `SynchronizationContext` / `TaskScheduler`, or thread pool if none             |
| `true`                          | `ConfigureAwait(false)` | Thread pool                                                                             |
| `false`                         | Normal `await`          | Captured `SynchronizationContext` / `TaskScheduler`, or `RclContext` event loop if none |
| `false`                         | `ConfigureAwait(false)` | `RclContext` event loop                                                                 |

The `WaitOneAsync(CancellationToken)` overload is equivalent to `WaitOneAsync(true, cancellationToken)`.

Most application code does not need to control continuation scheduling at this level, but the table above can be useful when implementing low-level event-driven code.
