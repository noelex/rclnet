// //Tests.TestPool();
using Rcl;
using Rcl.Graph;
using Rosidl.Messages.Tf2;
using Rosidl.Runtime.Interop;

var s = new CString();
s.CopyFrom("dsadsad");
s.CopyFrom("");

using var ctx = new RclContext(args);
using var node = ctx.CreateNode("test");

_ = Task.Run(async () =>
{
    try
    {
        Console.WriteLine();

        await foreach (var item in node.Graph.ReadEventsAsync(default))
        {
            switch (item)
            {
                case NodeEstablishedEvent nodeEstablished:
                    Console.WriteLine("NodeEstablished: " + nodeEstablished.Node.Name);
                    break;
                case NodeRemovedEvent nodeRemoved:
                    Console.WriteLine("NodeRemoved: " + nodeRemoved.Node.Name);
                    break;
                case TopicEstablishedEvent topicEstablished:
                    Console.WriteLine("TopicEstablished: " + topicEstablished.Topic.Name);
                    break;
                case TopicRemovedEvent topicRemoved:
                    Console.WriteLine("TopicRemoved: " + topicRemoved.Topic.Name);
                    break;
                case ServiceEstablishedEvent serviceEstablished:
                    Console.WriteLine("ServiceEstablished: " + serviceEstablished.Service.Name);
                    break;
                case ServiceRemovedEvent serviceRemoved:
                    Console.WriteLine("ServiceRemoved: " + serviceRemoved.Service.Name);
                    break;
                case PublisherEstablishedEvent publisherEstablished:
                    Console.WriteLine("PublisherEstablished: " + publisherEstablished.Publisher.Topic.Name +
                     ", Type = " + publisherEstablished.Publisher.Type + ", Node = " + publisherEstablished.Publisher.Node);
                    break;
                case PublisherRemovedEvent publisherRemoved:
                    Console.WriteLine("PublisherRemoved: " + publisherRemoved.Publisher.Topic.Name +
                     ", Type = " + publisherRemoved.Publisher.Type + ", Node = " + publisherRemoved.Publisher.Node);
                    break;
                case SubscriberEstablishedEvent subscriberEstablished:
                    Console.WriteLine("SubscriberEstablished: " + subscriberEstablished.Subscriber.Topic.Name +
                     ", Type = " + subscriberEstablished.Subscriber.Type + ", Node = " + subscriberEstablished.Subscriber.Node);
                    break;
                case SubscriberRemovedEvent subscriberRemoved:
                    Console.WriteLine("SubscriberRemoved: " + subscriberRemoved.Subscriber.Topic.Name +
                     ", Type = " + subscriberRemoved.Subscriber.Type + ", Node = " + subscriberRemoved.Subscriber.Node);
                    break;
                case ServerEstablishedEvent serverEstablished:
                    Console.WriteLine("ServerEstablished: " + serverEstablished.Server.Service.Name +
                    ", Type = " + serverEstablished.Server.Type + ", Node = " + serverEstablished.Server.Node);
                    break;
                case ServerRemovedEvent serverRemoved:
                    Console.WriteLine("ServerRemoved: " + serverRemoved.Server.Service.Name +
                    ", Type = " + serverRemoved.Server.Type + ", Node = " + serverRemoved.Server.Node);
                    break;
                case ClientEstablishedEvent clientEstablished:
                    Console.WriteLine("ClientEstablished: " + clientEstablished.Client.Service.Name +
                    ", Type = " + clientEstablished.Client.Type + ", Node = " + clientEstablished.Client.Node);
                    break;
                case ClientRemovedEvent clientRemoved:
                    Console.WriteLine("ClientRemoved: " + clientRemoved.Client.Service.Name + ", Type = " + clientRemoved.Client.Type);
                    break;
                case ActionEstablishedEvent serviceEstablished:
                    Console.WriteLine("ActionEstablished: " + serviceEstablished.Action.Name);
                    break;
                case ActionRemovedEvent serviceRemoved:
                    Console.WriteLine("ActionRemoved: " + serviceRemoved.Action.Name);
                    break;
                case ActionServerEstablishedEvent serverEstablished:
                    Console.WriteLine("ActionServerEstablished: " + serverEstablished.ActionServer.Action.Name +
                    ", Type = " + serverEstablished.ActionServer.Type + ", Node = " + serverEstablished.ActionServer.Node);
                    break;
                case ActionServerRemovedEvent serverRemoved:
                    Console.WriteLine("ActionServerRemoved: " + serverRemoved.ActionServer.Action.Name +
                    ", Type = " + serverRemoved.ActionServer.Type + ", Node = " + serverRemoved.ActionServer.Node);
                    break;
                case ActionClientEstablishedEvent clientEstablished:
                    Console.WriteLine("ActionClientEstablished: " + clientEstablished.ActionClient.Action.Name +
                    ", Type = " + clientEstablished.ActionClient.Type + ", Node = " + clientEstablished.ActionClient.Node);
                    break;
                case ActionClientRemovedEvent clientRemoved:
                    Console.WriteLine("ActionClientRemoved: " + clientRemoved.ActionClient.Action.Name + ", Type = " + clientRemoved.ActionClient.Type);
                    break;
            }
        }
    }
    finally
    {
        Console.WriteLine("Graph signal listener task completed.");
    }
});

var priv = new LookupTransformActionGoal.Priv();
priv.FixedFrame.CopyFrom("");

using var client = node.CreateActionClient<
    LookupTransformAction,
    LookupTransformActionGoal,
    LookupTransformActionResult,
    LookupTransformActionFeedback>("/lookup_transform");
Console.WriteLine("Server available: " + client.IsServerAvailable);
try
{
    var c = await client.SendGoalAsync(new LookupTransformActionGoal(), TimeSpan.FromSeconds(10));
    await c.Completion;
}
catch (Exception e)
{
    Console.WriteLine(e);
}

Console.WriteLine("Done.");

#if false
Console.WriteLine("MainThread: " + Thread.CurrentThread.ManagedThreadId + ", current context is " + SynchronizationContext.Current);

using var ctx = new RclContext(args);
using var gc = ctx.CreateGuardCondition();
using var timer = ctx.CreateTimer(RclClock.Ros, TimeSpan.FromMilliseconds(100000));

using var node = ctx.CreateNode("test");

Console.WriteLine(node.FullyQualifiedName);

var cts = new CancellationTokenSource();
var rsp = new EmptyServiceResponse();

var res=Task.FromResult(new EmptyServiceResponse());
using var sub = node.CreateNativeService<EmptyService, EmptyServiceRequest, EmptyServiceResponse>("/service1", (request, response, state) =>
{
    Console.WriteLine("Received request");
});

_ = Task.Run(async () =>
{
    try
    {
        await Task.Yield();
        var req=new EmptyServiceRequest();
        using var reqBuffer = RosMessageBuffer.Create<EmptyServiceRequest>();
        using var client = node.CreateClient<EmptyService, EmptyServiceRequest, EmptyServiceResponse>("/service1");
        while (!cts.IsCancellationRequested)
        {
            try
            {
                await Task.Delay(10);
                using var resp = await client.InvokeAsync(reqBuffer);
                Console.WriteLine("Done.");
            }
            catch (Exception e)
            {
                Console.WriteLine("Request failed: " + e.Message);
            }

        }
    }
    finally
    {
        Console.WriteLine("Client task completed.");
    }
});

_ = Task.Run(async () =>
{
    try
    {
        using var sub = node.CreateSubscription<Vector3>("/turtle1/cmd_vel");

        while (!cts.IsCancellationRequested)
        {
            await foreach (var msg in sub.ReadAllAsync(cts.Token))
            {
                HandleMessageBuffer(msg);

                static void HandleMessageBuffer(Vector3 p)
                {
                    Console.WriteLine($"Received message: x = {p.X:F1}, y = {p.Y:F1}, z = {p.Z:F1}");
                }

            }
        }
    }
    finally
    {
        Console.WriteLine("Subscription listener task completed.");
    }
});

_ = Task.Run(async () =>
{
    try
    {
        using var pub = node.CreatePublisher<Vector3>("/turtle1/cmd_vel");

        var msg = new Vector3();
        while (!cts.IsCancellationRequested)
        {
            msg.X += 1.1;
            msg.X += 2.2;
            msg.X += 3.3;

            pub.Publish(msg);
            await Task.Delay(100000);
        }
    }
    finally
    {
        Console.WriteLine("Publisher listener task completed.");
    }
});



_ = Task.Run(async () =>
{
    try
    {
        while (!cts.IsCancellationRequested)
        {
            await timer.WaitOneAsync();
            gc.Trigger();
            Console.WriteLine("Triggering GC on thread: " + Thread.CurrentThread.ManagedThreadId);
        }
    }
    finally
    {
        Console.WriteLine("Trigger task completed.");
    }
});

_ = Task.Run(async () =>
{
    try
    {
        while (!cts.IsCancellationRequested)
        {
            await gc.WaitOneAsync();
            Console.WriteLine("[Background-1]GC resumed on thread: " + Thread.CurrentThread.ManagedThreadId);
        }
    }
    finally
    {
        Console.WriteLine("Background task 1 completed.");
    }
});

// This will cause following lines to be executed on RCL event loop,
// which means the RCL event loop will be blocked by Console.ReadLine indefinitely.
//await Task.Yield(); 

Console.ReadLine();
Console.WriteLine("Exiting.");
cts.Cancel();
#endif