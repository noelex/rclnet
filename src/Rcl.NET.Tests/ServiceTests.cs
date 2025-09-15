using Rosidl.Messages.Rcl;
using Rosidl.Messages.Service;
using Rosidl.Messages.Tf2;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Rcl.NET.Tests;

[Collection("Sequential")]
public class ServiceTests
{
    private const int RequestTimeout = 10_000, ServerOnlineTimeout = 1000;

    [Fact]
    public async Task ClientRequestTimeout()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = node.CreateClient<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(NameGenerator.GenerateServiceName());

        await Assert.ThrowsAsync<TimeoutException>(() =>
            client.InvokeAsync(new ListParametersServiceRequest(), 100));
    }

    [Fact]
    public async Task NormalServiceCall()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var response = new ListParametersServiceResponse(new(new[] { "n1", "n2" }, new[] { "p1", "p2" }));
        var service = NameGenerator.GenerateServiceName();

        using var server = node.CreateService<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(service, (request, state) =>
            {
                Assert.True(context.IsCurrent);
                return response;
            });

        using var clientNode = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = clientNode.CreateClient<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(service);

        await client.TryWaitForServerAsync(ServerOnlineTimeout);
        var actualResponse = await client.InvokeAsync(new ListParametersServiceRequest(), RequestTimeout);

        Assert.True(response.Result.Names.SequenceEqual(actualResponse.Result.Names));
        Assert.True(response.Result.Prefixes.SequenceEqual(actualResponse.Result.Prefixes));
    }

    [Fact]
    public Task ContinuationOfInvokeAsyncShouldNotExecuteOnEventLoopWhenContextIsCaptured()
    {
        return Task.Run(async () =>
        {
            await using var anotherContext = new RclContext(TestConfig.DefaultContextArguments, useSynchronizationContext: true);

            await using var context = new RclContext(TestConfig.DefaultContextArguments);
            using var node = context.CreateNode(NameGenerator.GenerateNodeName());

            var response = new ListParametersServiceResponse();
            var service = NameGenerator.GenerateServiceName();

            using var server = node.CreateService<
                ListParametersService,
                ListParametersServiceRequest,
                ListParametersServiceResponse>(service, (request, state) =>
                {
                    return response;
                });

            using var clientNode = context.CreateNode(NameGenerator.GenerateNodeName());
            using var client = clientNode.CreateClient<
                ListParametersService,
                ListParametersServiceRequest,
                ListParametersServiceResponse>(service);

            var result = await client.TryWaitForServerAsync(ServerOnlineTimeout);
            await anotherContext.Yield();
            // Now we are on the event loop of anotherContext.

            // Captured the sync context of the anotherContext, we should be on the event loop of anotherContext.
            await client.InvokeAsync(new ListParametersServiceRequest(), RequestTimeout);
            Assert.True(anotherContext.IsCurrent);

            // Suppressing the sync context.
            await client.InvokeAsync(new ListParametersServiceRequest(), RequestTimeout).ConfigureAwait(false);

            // No captured sync context, we should be on thread pool thread.
            await client.InvokeAsync(new ListParametersServiceRequest(), RequestTimeout);
            Assert.False(context.IsCurrent);

            await client.InvokeAsync(new ListParametersServiceRequest(), RequestTimeout).ConfigureAwait(false);
            Assert.False(context.IsCurrent);
        });
    }

    [Fact]
    public async Task ConcurrentServiceCallHandlerShouldStartOnEventLoop()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var response = new ListParametersServiceResponse(new(new[] { "n1", "n2" }, new[] { "p1", "p2" }));
        var service = NameGenerator.GenerateServiceName();

        using var server = node.CreateConcurrentService<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(service, (request, state, ct) =>
            {
                Assert.True(context.IsCurrent);
                return Task.FromResult(response);
            }, null);

        using var clientNode = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = clientNode.CreateClient<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(service);

        await client.TryWaitForServerAsync(ServerOnlineTimeout);
        var actualResponse = await client.InvokeAsync(new ListParametersServiceRequest(), RequestTimeout);
    }

    [Fact]
    public async Task InvokeWithZeroTimeout()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = node.CreateClient<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(NameGenerator.GenerateServiceName());

        await Assert.ThrowsAsync<TimeoutException>(() =>
            client.InvokeAsync(new ListParametersServiceRequest(), 0));
    }

    [Fact]
    public async Task InvokeShouldBeInterruptedWhenClientIsDisposed()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        var client = node.CreateClient<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(NameGenerator.GenerateServiceName());

        var assertTask = Assert.ThrowsAsync<ObjectDisposedException>(() =>
            client.InvokeAsync(new ListParametersServiceRequest(), 1000));
        await Task.Delay(100).ContinueWith(x => client.Dispose());

        await assertTask;
    }

    [Fact]
    public async Task InvokeWithManualCancel()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = node.CreateClient<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(NameGenerator.GenerateServiceName());
        using var cts = new CancellationTokenSource(100);

        var ex = await Assert.ThrowsAsync<OperationCanceledException>(() =>
            client.InvokeAsync(new ListParametersServiceRequest(), 1000, cts.Token));

        Assert.Equal(cts.Token, ex.CancellationToken);
    }

    [SkippableFact]
    public async Task ClientGid()
    {
        Skip.If(!RosEnvironment.IsSupported(RosEnvironment.Iron), "Service introspection is only supported on iron and later.");

        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = node.CreateClient<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(NameGenerator.GenerateServiceName());

        Assert.NotEqual(GraphId.Empty, client.Gid);
    }

    [SkippableFact]
    public async Task ServiceIntrospectionForClients()
    {
        Skip.If(!RosEnvironment.IsSupported(RosEnvironment.Iron), "Service introspection is only supported on iron and later.");

        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var name = NameGenerator.GenerateServiceName();
        var introspectionTopic = "/" + name + "/_service_event";
        var eventType = "rcl_interfaces/srv/ListParameters_Event";

        using var client = node.CreateClient<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(name);

        client.ConfigureIntrospection(ServiceIntrospectionState.Disabled);
        var success = await node.Graph.TryWatchAsync((g, e) =>
            !g.Topics.Any(x => x.Name == introspectionTopic), 1000);
        Assert.True(success);

        client.ConfigureIntrospection(ServiceIntrospectionState.MetadataOnly);
        success = await node.Graph.TryWatchAsync((g, e) =>
            g.Topics.Any(x => x.Name == introspectionTopic &&
            x.Publishers.Count == 1 &&
            x.Publishers.First().Type == eventType), 1000);
        Assert.True(success);

        client.ConfigureIntrospection(ServiceIntrospectionState.Disabled);
        success = await node.Graph.TryWatchAsync((g, e) =>
            !g.Topics.Any(x => x.Name == introspectionTopic), 1000);
        Assert.True(success);

        client.ConfigureIntrospection(ServiceIntrospectionState.Full);
        success = await node.Graph.TryWatchAsync((g, e) =>
            g.Topics.Any(x => x.Name == introspectionTopic &&
            x.Publishers.Count == 1 &&
            x.Publishers.First().Type == eventType), 1000);
        Assert.True(success);
    }

    [SkippableFact]
    public async Task ServiceIntrospectionForServers()
    {
        Skip.If(!RosEnvironment.IsSupported(RosEnvironment.Iron), "Service introspection is only supported on iron and later.");

        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var name = NameGenerator.GenerateServiceName();
        var introspectionTopic = "/" + name + "/_service_event";
        var eventType = "rcl_interfaces/srv/ListParameters_Event";

        using var server = node.CreateService<
            ListParametersService,
            ListParametersServiceRequest,
            ListParametersServiceResponse>(name, (req, state) => new());

        server.ConfigureIntrospection(ServiceIntrospectionState.Disabled);
        var success = await node.Graph.TryWatchAsync((g, e) =>
            !g.Topics.Any(x => x.Name == introspectionTopic), 1000);
        Assert.True(success);

        server.ConfigureIntrospection(ServiceIntrospectionState.MetadataOnly);
        success = await node.Graph.TryWatchAsync((g, e) =>
            g.Topics.Any(x => x.Name == introspectionTopic &&
            x.Publishers.Count == 1 &&
            x.Publishers.First().Type == eventType), 1000);
        Assert.True(success);

        server.ConfigureIntrospection(ServiceIntrospectionState.Disabled);
        success = await node.Graph.TryWatchAsync((g, e) =>
            !g.Topics.Any(x => x.Name == introspectionTopic), 1000);
        Assert.True(success);

        server.ConfigureIntrospection(ServiceIntrospectionState.Full);
        success = await node.Graph.TryWatchAsync((g, e) =>
            g.Topics.Any(x => x.Name == introspectionTopic &&
            x.Publishers.Count == 1 &&
            x.Publishers.First().Type == eventType), 1000);
        Assert.True(success);
    }
    public static IEnumerable<object[]> Cases =>
           new List<object[]>
           {
            new object[] { ServiceIntrospectionState.MetadataOnly },
            new object[] { ServiceIntrospectionState.Full         },
           };
    [SkippableTheory]
    [MemberData(nameof(Cases))]
    public async Task PerformServiceIntrospection(ServiceIntrospectionState state)
    {
        Skip.If(!RosEnvironment.IsSupported(RosEnvironment.Iron), "Service introspection is only supported on iron and later.");

        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var name = NameGenerator.GenerateServiceName();
        var introspectionTopic = "/" + name + "/_service_event";

        using var server = node.CreateService<
            FrameGraphService,
            FrameGraphServiceRequest,
            FrameGraphServiceResponse>(name, (req, state) => new());

        using var client = node.CreateClient<
            FrameGraphService,
            FrameGraphServiceRequest,
            FrameGraphServiceResponse>(name);

        server.ConfigureIntrospection(state);
        client.ConfigureIntrospection(state);

        var events = new Dictionary<byte, FrameGraphServiceEvent>();

        using var cts = new CancellationTokenSource(1000);
        var introspectTask = IntrospectService(4, cts.Token);
        await client.InvokeAsync(new FrameGraphServiceRequest());

        await introspectTask;
        Assert.Equal(4, events.Count);

        Assert.Equal(ServiceEventInfo.REQUEST_SENT, events.ElementAt(0).Key);
        Assert.Equal(ServiceEventInfo.REQUEST_RECEIVED, events.ElementAt(1).Key);
        Assert.Equal(ServiceEventInfo.RESPONSE_SENT, events.ElementAt(2).Key);
        Assert.Equal(ServiceEventInfo.RESPONSE_RECEIVED, events.ElementAt(3).Key);

        Assert.Equal(client.Gid, new(MemoryMarshal.Cast<sbyte, byte>(events[ServiceEventInfo.REQUEST_SENT].Info.ClientGid)));
        Assert.Equal(client.Gid, new(MemoryMarshal.Cast<sbyte, byte>(events[ServiceEventInfo.RESPONSE_RECEIVED].Info.ClientGid)));

        if (state == ServiceIntrospectionState.Full)
        {
            Assert.Single(events[ServiceEventInfo.REQUEST_SENT].Request);
            Assert.Empty(events[ServiceEventInfo.REQUEST_SENT].Response);
            Assert.Single(events[ServiceEventInfo.REQUEST_RECEIVED].Request);
            Assert.Empty(events[ServiceEventInfo.REQUEST_RECEIVED].Response);
            Assert.Empty(events[ServiceEventInfo.RESPONSE_SENT].Request);
            Assert.Single(events[ServiceEventInfo.RESPONSE_SENT].Response);
            Assert.Empty(events[ServiceEventInfo.RESPONSE_RECEIVED].Request);
            Assert.Single(events[ServiceEventInfo.RESPONSE_RECEIVED].Response);
        }
        else
        {
            Assert.Empty(events[ServiceEventInfo.REQUEST_SENT].Request);
            Assert.Empty(events[ServiceEventInfo.REQUEST_SENT].Response);
            Assert.Empty(events[ServiceEventInfo.REQUEST_RECEIVED].Request);
            Assert.Empty(events[ServiceEventInfo.REQUEST_RECEIVED].Response);
            Assert.Empty(events[ServiceEventInfo.RESPONSE_SENT].Request);
            Assert.Empty(events[ServiceEventInfo.RESPONSE_SENT].Response);
            Assert.Empty(events[ServiceEventInfo.RESPONSE_RECEIVED].Request);
            Assert.Empty(events[ServiceEventInfo.RESPONSE_RECEIVED].Response);
        }

        async Task IntrospectService(int expectedEvents, CancellationToken cancellationToken)
        {
            using var sub = node.CreateSubscription<FrameGraphServiceEvent>(introspectionTopic, new(queueSize: 10));
            await foreach (var item in sub.ReadAllAsync(cancellationToken))
            {
                events[item.Info.EventType] = item;
                if (events.Count == expectedEvents)
                {
                    break;
                }
            }
        }
    }
}
