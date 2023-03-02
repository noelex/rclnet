# rclnet
rclnet is a fast and easy-to-use .NET wrapper over ROS 2 client library, allowing .NET applications to interact with other ROS applications.
## Supported Platforms
Supported .NET Versions:
- .NET 7

Supported ROS 2 Distributions:
- Humble Hawksbill
- Foxy Fitzroy

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
|  Feature                 |  Status |  Additional Information       |
|------------------------- |-------- |------------------------------ | 
|  Topics                  | ✅      | N/A                           |
|  Services                | ✅      | N/A                           | 
|  Actions                 | ✅      | Managed implementation.       | 
|  Clocks                  | ✅      | Supports external time source by setting `use_sim_time` to `true`.<br/>`CancellationTokenSource`s can also be configured to cancel with timeout measured by external clock.  |
|  Timers                  | ✅      | N/A                           | 
|  Guard Conditions        | ✅      | N/A                           |
|  Events                  | ✅      | Event handlers can be registered via `SubscriptionOptions` or `PublisherOptions` when creating the subscirption or publisher.  | 
|  ROS Graph               | ✅      | Managed implementation.       | 
|  Logging                 | ✅      | Supports logging to stdout, /rosout and log files. Configurable with `--ros-args`.| 
|  Parameter Service       | ⚠️      | Supports loading parameters from command-line arguments and parameter files.<br/>Locally declared parameters are exposed via [Parameter API](https://design.ros2.org/articles/ros_parameters.html).<br/>Parameter client is not implemented.                          | 
|  Lifecycle               | ❌      | N/A                           | 
|  Network Flow Endpoints  | ❌      | Available since galactic.     |
|  Content Filtered Topics | ❌      | Available since humble.       |

✅Supported ⚠️Partial support ❌Not supported ⏳In development

## Execution Model
Unlike `rclcpp` an `rclpy`, `rclnet` doesn't have the concept of executors. Each `RclContext` runs its
own event loop for waiting on signals and dispatching callbacks, which is essentialy a single-threaded
executor.

Although `rclnet` does not provide multi-threaded executors, it doesn't mean that you can't process messages or handle
service requests using multiple threads. All communication primitives in `rclnet` provide both synchronous
and asynchronous APIs for different needs and scenarios. Synchronous APIs are simpler and faster if the work need to be done is simple enough, e.g. neither CPU-intensive nor needs to issue blocking calls. Asynchronous APIs, in contrast, are for scenarios where you need to perform asynchronous calls or
offload blocking operations into background threads.

Take subscriptions for example, you can receive messages synchronously using `IRclSubscription<T>.Subscribe`,
or asynchronously using `IRclSubscription<T>.ReadAllAsync`. To process messages asynchronously, one 
would typically write:
```csharp
await foreach (var msg in sub.ReadAllAsync())
{
    // Perform asynchronous operation.
    await SomeAsyncOperation(msg);

    // Perform synchronous operation and wait for its completion without blocking the event loop.
    await Task.Run(() => SomeOffloadedSyncOperation(msg));
}
```

`RclContext` always executes user codes on its event loop. Take the above code for example:

```csharp
await foreach (var msg in sub.ReadAllAsync())
{
    // On event loop.
    await SomeAsyncOperation(msg);
    // On thread pool.
    await Task.Run(() => SomeOffloadedSyncOperation(msg));
    // On thread pool.
}
```

You can use `RclContext.SynchronizationContext`, `RclContext.Yield()`, `Task.Yield()` and
`ConfigureAwait(false)` to control asynchronous execution flow more precisely:

```csharp
SynchronizationContext
    .SetSynchronizationContext(rclContext.SynchronizationContext);
await foreach (var msg in sub.ReadAllAsync())
{
    // On event loop.
    await SomeAsyncOperation(msg);
    // On event loop.
    await Task.Run(() => SomeOffloadedSyncOperation(msg));
    // On event loop.
    await AnotherAsyncOperation(msg).ConfigureAwait(false);
    // On thread pool.
}
```

```csharp
await foreach (var msg in sub.ReadAllAsync())
{
    // On event loop.
    await Task.Yield();
    // On thread pool.
    await rclContext.Yield();
    // On event loop.
}
```

## Installing
Stable releases of rclnet are hosted on NuGet. You can install them using the following command:
```
dotnet add package Rcl.NET
```
To try out latest features being developed, you can install the preview packages with the following command:
```
dotnet add package Rcl.NET --prerelease
```

## Building and Running Examples
### Install dependencies
The following instruction assumes that you've already installed ROS 2 foxy or humble in your system.

You'll need .NET 7.0 SDK to build and run the examples, see instructions 
[here](https://learn.microsoft.com/dotnet/core/install/linux-ubuntu).

Make sure you have all dependencies installed by running:
```
rosdep install -i --from-paths examples
```

### Run with `dotnet run`
Now you can run example projects using `dotnet run`, e.g.
```
dotnet run examples/turtle_rotate
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

## Showcase
### Subscribing
```csharp
using var ctx = new RclContext(args);
using var node = ctx.CreateNode("hellow_world");
using var sub = node.CreateSubscription<Twist>("/cmd_vel");
await foreach (Twist msg in sub.ReadAllAsync())
{
    ...
}
```
### Publishing
```csharp
using var pub = node.CreatePublisher<Vector3>("/vec");
pub.Publish(new Vector3(x: 1, y: 2, z: 3));
```
### Handling Service Calls
```csharp
using var server = node.CreateService<
    EmptyService,
    EmptyServiceRequest,
    EmptyServiceResponse>("/vec",
        (request, state) =>
        {
            return new EmptyServiceResponse();
        });
await Task.Delay(-1);
```
### Calling Services
```csharp
using var client = node.CreateClient<
    EmptyService,
    EmptyServiceRequest,
    EmptyServiceResponse>("/vec");
await client.InvokeAsync(new EmptyServiceRequest());
``` 
### Monitoring ROS Graph Changes
```csharp
node.Graph
    .OfType<NodeAppearedEvent>()
    .Subscribe(x =>
    {
        Console.WriteLine($"Node {x.Node.Name} is online.");
    });

await node.Graph.WaitForServiceServerAsync("/my/service");
```
### Calling Action Servers
```csharp
using var client = node.CreateActionClient<
    SpinAction,
    SpinActionGoal,
    SpinActionResult,
    SpinActionFeedback>("/spin");

using var goal = await client.SendGoalAsync(
        new SpinActionGoal(targetYaw: Math.PI));

await foreach (var feedback in goal.ReadFeedbacksAsync())
{
    Console.WriteLine(feedback.AngularDistanceTraveled);
}

var result = await goal.GetResultAsync();
```
### Zero (Managed Heap) Allocation APIs
```csharp
using var sub = node.CreateNativeSubscription<Twist>("/cmd_vel");
await foreach (RosMessageBuffer msg in sub.ReadAllAsync())
{
    using (msg) ProcessMessage(msg);

    static void ProcessMessage(RosMessageBuffer buffer)
    {
        ref var twist = ref buffer.AsRef<Twist.Priv>();
        ...
    }
}
```

## How to Generate Messages
1. Create a new .NET library project targeting .NET 7.

2. Add the following lines to the `.csproj` file:
   ```xml
   <PropertyGroup>
      <AllowUnsafeBlocks>true</AllowUnsafeBlocks>
   </PropertyGroup>
   ```

3. Add NuGet package `Rcl.NET` to the project.
   If the library doesn't need to depend on `Rcl.NET`
   (e.g. contains only message definitions), you can install just `Rosidl.Runtime` package.

4. Add a `ros2cs.spec` file to the root of the project.
    See [here](https://github.com/noelex/rclnet/blob/main/src/ros2cs/ros2cs.spec) for detailed explanations on how to write a `ros2cs.spec` file.

5. Add a `_AssemblyAttributes.cs` to the root of the project with the following contents:
    ```csharp
    [assembly: System.Runtime.CompilerServices.DisableRuntimeMarshalling]
    ```

6. Install `ros2cs` with the following command:
   ```
   dotnet tool install -g ros2cs
   ```

7. Run `ros2cs /path/to/ros2cs.spec` to generate message classes / structs.