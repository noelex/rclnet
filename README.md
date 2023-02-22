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

### Supported ROS Features
|  Feature                 |  Status |  Additional Information       |
|------------------------- |-------- |------------------------------ | 
|  Topics                  | ✅      | N/A                           |
|  Services                | ✅      | N/A                           | 
|  Actions                 | ✅      | Managed implementation.       | 
|  Clocks                  | ⚠️      | ROS time abstraction override and time jump observation are not supported.   |
|  Timers                  | ⚠️      | Changing period of timers is not supported.    | 
|  Guard Conditions        | ✅      | N/A                           | 
|  Events                  | ❌      | N/A                           | 
|  Lifecycles              | ❌      | N/A                           | 
|  Parameter Services      | ⏳      | N/A                           | 
|  ROS Graph               | ✅      | Managed implementation.       | 
|  Logging                 | ✅      | Configurable via `--ros-args`.| 
|  Network Flow Endpoints  | ❌      | Available since galactic.     |
|  Content Filtered Topics | ❌      | Available since humble.       |

✅ Fully supported
⚠️Partial support
❌ Not supported
⏳ In development

## Installing
rclnet is being actively developed currently, thus no stable package available for now.

You can try out by installing preview packages with the following command:
```
dotnet add package Rcl.NET --prerelease
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
        (request, cancellationToken, state) =>
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
    .OfType<NodeEstablishedEvent>()
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
      <RootNamespace>Rosidl.Messages</RootNamespace>
   </PropertyGroup>

   <ItemGroup>
      <!--Uncomment the following line if generated code depends on common messages included in Rosidl.CommonMessages-->
      <!--<PackageReference Include="Rosidl.CommonMessages" Version="1.0.0-preview.5" />-->
      <PackageReference Include="Rosidl.Runtime" Version="1.0.0-preview.5" />
   </ItemGroup>
   ```
3. Add a `ros2cs.spec` file to the root of the project.
    See [here](https://github.com/noelex/rclnet/blob/main/src/ros2cs/ros2cs.spec) for detailed explanations on how to write a `ros2cs.spec` file.

    You may want to exclude packages that are already included in [Rosidl.CommonMessages](https://github.com/noelex/rclnet/tree/main/src/Rosidl.CommonMessages/ros2cs.spec).
4. Add a `_AssemblyAttributes.cs` to the root of the project with the following contents:
    ```csharp
    [assembly: System.Runtime.CompilerServices.DisableRuntimeMarshalling]
    ```
4. Install `ros2cs` with the following command:
   ```
   dotnet tool install -g ros2cs
   ```
5. Run `ros2cs /path/to/ros2cs.spec` to generate message classes / structs.