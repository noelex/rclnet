# rclnet
rclnet is a fast and easy-to-use .NET wrapper over ROS 2 client library, allowing .NET applications to interact with other ROS applications.

## What's New in 3.0
 - ROS 2 Lyrical Support
 - Added portable ROSIDL ABI support
 - Added MSBuild incremental build support for generated interfaces

## What's New in 2.0
 - Added support for .NET 10 and changed minimum supported .NET version to 8.0
 - ROS 2 Kilted Support
 - ROS 2 Jazzy Support by @AlrayQiu ([#39](https://github.com/noelex/rclnet/pull/39))
 - Simplified message generation workflow by @ha-ves ([#38](https://github.com/noelex/rclnet/pull/38))
 - String pooling is now disabled by default

## Features
- Completely asynchronous and `async`/`await` friendly.
- Flexible asynchronous scheduling control to fit rclnet into existing applications.
- Unified message generation for POCOs and blittable structures.
- Intuitive ROS graph querying and monitoring APIs.
- Easy-to-use POCO-based APIs.
- Fast and zero managed heap allocation APIs operating directly on native message buffers.
- Single package with runtime support for different ROS 2 distros.
- Builtin support for querying topic messages and ROS graph events with [Reactive Extensions](https://github.com/dotnet/reactive).

### Supported ROS Features
| Feature                 | Status | Additional Information                                                                                                                                                                                                                       |
| ----------------------- | ------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Topics                  | ✅      | N/A                                                                                                                                                                                                                                          |
| Services                | ✅      | N/A                                                                                                                                                                                                                                          |
| Actions                 | ✅      | Managed implementation.                                                                                                                                                                                                                      |
| Clocks                  | ✅      | Supports external time source by setting `use_sim_time` to `true`.<br/>`CancellationTokenSource`s can also be configured to cancel with timeout measured by external clock.                                                                  |
| Timers                  | ✅      | N/A                                                                                                                                                                                                                                          |
| Guard Conditions        | ✅      | N/A                                                                                                                                                                                                                                          |
| Events                  | ✅      | Event handlers can be registered via `SubscriptionOptions` or `PublisherOptions` when creating the subscirption or publisher.                                                                                                                |
| ROS Graph               | ✅      | Managed implementation.                                                                                                                                                                                                                      |
| Logging                 | ✅      | Supports logging to stdout, /rosout and log files. Configurable with `--ros-args`.                                                                                                                                                           |
| [Content Filtered Topics](https://github.com/ros2/design/blob/918c09758ed4c0854aa128b9c8ed0051c21a6590/articles/content_filtering.md) | ✅      | Available since humble. 
| [Network Flow Endpoints](https://design.ros2.org/articles/unique_network_flows.html)  | ✅      | Available since galactic.<br/>Network flow endpoints of publishers and subscriptions can be retrieved via `IRclPublisher.Endpoints` and `IRclSubscription.Endpoints` property.<br/>Unique network flow endpoints requirement can be configured when creating `SubscriptionOptions` and `PublisherOptions`.|
| [Service Introspection](https://github.com/ros-infrastructure/rep/blob/jacob/service_introspection/rep-2012.rst)   | ✅      | Available since iron. |
| [Parameter Service](https://design.ros2.org/articles/ros_parameters.html)       | ⚠️      | Supports loading parameters from command-line arguments and parameter files.<br/>Locally declared parameters are exposed via Parameter API.<br/>Parameter client is not implemented. |
| [Lifecycle Nodes](https://design.ros2.org/articles/node_lifecycle.html)         | ❌      | N/A                                                                                                                                                                                                                                          |

✅Supported ⚠️Partial support ❌Not supported ⏳In development

## Supported Platforms
Supported .NET Versions:
- .NET 8
- .NET 9
- .NET 10

Supported ROS 2 Distributions:
- Foxy Fitzroy
- Humble Hawksbill
- Iron Irwini
- Jazzy Jalisco
- Kilted Kaiju
- Lyrical Luth

Supported Operating Systems:
- Ubuntu
- Windows

Should also work on macOS but untested.

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

## Asynchronous Execution Model
Unlike rclcpp and rclpy, rclnet doesn't have the concept of executors. Each `RclContext` runs its
own event loop for waiting on signals and dispatching callbacks, which is essentialy a single-threaded
executor.

Although rclnet does not provide multi-threaded executors, it doesn't mean that you can't process messages or handle
service requests using multiple threads. All communication primitives in rclnet provide both synchronous
and asynchronous APIs for different needs and scenarios.

Synchronous APIs are simpler and faster if the work need to be done is simple enough, e.g. neither CPU-intensive nor needs to issue blocking calls. Asynchronous APIs, in contrast, are for scenarios where you need to perform asynchronous calls or
offload blocking operations into background threads.

Take subscriptions for example, you can receive messages synchronously using `IRclSubscription<T>.Subscribe`,
or asynchronously using `IRclSubscription<T>.ReadAllAsync`. Synchronous subscriptions always handle messages
on the event loop. While for asynchronous subscriptions, you can choose where you'd like to process the received
messages:

```csharp
await foreach (var msg in sub.ReadAllAsync())
{
    // Perform asynchronous operation.
    await SomeAsyncOperation(msg);

    // Perform synchronous operation and wait for its completion without blocking the event loop.
    await Task.Run(() => SomeOffloadedSyncOperation(msg));
}
```

In the above example, the event loop of the `RclContext` is used for listening to events only. Where are the messages
handled depends on the `SynchronizationContext` currently captured.

If there's no `SynchronizationContext` in use, event handling happens in background threads by default. Otherwise, the
events will be handled in the captured `SynchronizationContext`. If you are using rclnet inside a GUI application,
this usually means that the events are handled on the UI thread.

`RclContext`s can also have their own `SynchronizationContext`s, which always schedule asynchronous operations on the event loop.
This is extremely helpful if you want to introduce single-threaded concurrency into your application:

```csharp
await using var context = new RclContext(useSynchronizationContext: true);

...

// Enforce execution on the event loop so that we can capture its SynchronizationContext.
await context.Yield();

// All following awaits will resume on the event loop by default.
await foreach (var msg in sub.ReadAllAsync())
{
    // On event loop.
    await SomeAsyncOperation(msg);
    // On event loop.
    await Task.Run(() => {
        // On thread pool.
        SomeOffloadedSyncOperation(msg);
    });
    // On event loop.
    await Task.Yield();
    // On event loop.

    // We can also spin up multiple coroutines to run concurrently on the event loop.
    Task task1 = Coroutine1Async(msg),
         task2 = Coroutine2Async(msg);

    // Or asynchronously wait for all coroutines to complete.
    await Task.WhenAll(task1, task2);

    ...

    // The execution of current async method will stay on the event
    // loop unless we break out of the SynchronizationContext using
    // ConfigureAwait(false), or RclContext.YieldBackground().

    await AnotherAsyncOperation(msg).ConfigureAwait(false);
    // On thread pool thread.

    // We can still transition back to the event loop with context.Yield().

    await context.Yield();
    // On event loop.

    await RclContext.YieldBackground();
    // On thread pool thread.
}
```

As shown in the above example, besides of `SynchronizationContext`, you can also use `RclContext.Yield`, `RclContext.YieldBackground` and `ConfigureAwait(false)` to perform fine-grained control
over the asynchronous exection flow.

### Additional Notes about `IRclWaitObject.WaitOneAsync`
Timers and guard conditions created by `RclContext` implements `IRclWaitObject` interface,
which allow the caller to asynchronously wait for the signal.

`IRclWaitObject` interface exposes the following two overloads of `WaitOneAsync`:
```csharp
ValueTask WaitOneAsync(bool runContinuationAsynchronously, CancellationToken cancellationToken = default);
ValueTask WaitOneAsync(CancellationToken cancellationToken = default);
```
The latter overload simply calls another one with `runContinuationAsynchronously` set to `true`.

`WaitOneAsync` allows the caller to explicitly control the execution of the continuation via `runContinuationAsynchronously`
parameter. Assuming there's no captured `SynchronizationContext` or `TaskScheduler`, when `runContinuationAsynchronously`
is set to `true`, the continuation will be scheduled to execute in thread pool. And if `runContinuationAsynchronously`
is set to `false`, the continuation is guaranteed to execute on the event loop.

However, when a `SynchronizationContext` or `TaskScheduler` is captured, the continuation of the call to `WaitOneAsync`
will always execute in the captured context, regardless of the value of `runContinuationAsynchronously`.

Since context capture can be suppressed by calling `ConfigureAwait` with `continueOnCapturedContext` set to `false`,
execution of the continuation can be precisely controlled using `runContinuationAsynchronously` in conjunction with
`continueOnCapturedContext`.


| `runContinuationAsynchronously` | `continueOnCapturedContext` | Continuation Execution                                                             |
| ------------------------------- | --------------------------- | ---------------------------------------------------------------------------------- |
| `true`                          | `true`                      | Captured `SynchronizationContext` or `TaskScheduler` if any, thread pool otherwise |
| `true`                          | `false`                     | Thread pool                                                                        |
| `false`                         | `true`                      | Captured `SynchronizationContext` or `TaskScheduler` if any, event loop otherwise  |
| `false`                         | `false`                     | Event loop                                                                         |

## Building and Running Examples
### Install dependencies
The following instruction assumes that you've already installed ROS 2 foxy or humble in your system.

You'll need .NET 8.0 SDK to build and run the examples, see instructions 
[here](https://learn.microsoft.com/dotnet/core/install/linux-ubuntu).

Make sure you have all dependencies installed by running:
```
rosdep install -i --from-paths examples
```

### Run with `dotnet run`
Now you can run example projects using `dotnet run`, e.g.
```
dotnet run --project examples/turtle_rotate
```

### Run with `ros2 run`
Or you can build and install examples as colcon packages:
```
colcon build --executor sequential --merge-install --paths examples/*
source install/setup.bash
```

To run an example node, use `ros2 run`, e.g.
```
ros2 run graph_monitor graph_monitor
```
