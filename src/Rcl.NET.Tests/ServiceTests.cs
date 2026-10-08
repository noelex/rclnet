using Rosidl.Messages.Rcl;
using Rosidl.Messages.Service;
using Rosidl.Messages.Tf2;
using Rosidl.Runtime;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Rcl.NET.Tests;

public class ServiceTests
{
    // These finite deadlines exercise request timer setup and cleanup.
    private const int RequestTimeout = 10_000;

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task ClientFailuresAreReportedThroughReturnedTasks(bool native)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = node.CreateClient<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(
            NameGenerator.GenerateServiceName());
        using var buffer = RosMessageBuffer.Create<ListParametersServiceRequest>();
        using var cancellation = new CancellationTokenSource();

        // Invoke outside ThrowsAsync so a synchronous throw fails the test.
        var invalid = Invoke(-2);
        await Assert.ThrowsAsync<ArgumentOutOfRangeException>(() => invalid);
        Assert.True(invalid.IsFaulted);

        var canceled = Invoke(RequestTimeout, cancellation.Token);
        cancellation.Cancel();
        var error = await Assert.ThrowsAnyAsync<OperationCanceledException>(() => canceled);
        Assert.Equal(cancellation.Token, error.CancellationToken);
        Assert.True(canceled.IsCanceled);

        var preCanceled = Invoke(RequestTimeout, cancellation.Token);
        error = await Assert.ThrowsAnyAsync<OperationCanceledException>(() => preCanceled);
        Assert.Equal(cancellation.Token, error.CancellationToken);
        Assert.True(preCanceled.IsCanceled);

        var timedOut = Invoke(0);
        await Assert.ThrowsAsync<TimeoutException>(() => timedOut);
        Assert.True(timedOut.IsFaulted);

        client.Dispose();
        var closed = Invoke(RequestTimeout);
        await Assert.ThrowsAsync<ObjectDisposedException>(() => closed);
        Assert.True(closed.IsFaulted);

        Task Invoke(int timeout, CancellationToken token = default)
        {
            return native
                ? client.InvokeAsync(buffer, timeout, token)
                : client.InvokeAsync(new ListParametersServiceRequest(), timeout, token);
        }
    }

    [Fact]
    public async Task ClientTimeoutAndCancellationDoNotStopOtherRequestTimers()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = node.CreateClient<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(
            NameGenerator.GenerateServiceName());
        using var cancellation = new CancellationTokenSource();
        var canceled = client.InvokeAsync(new ListParametersServiceRequest(), RequestTimeout, cancellation.Token);
        var timedOut = client.InvokeAsync(new ListParametersServiceRequest(), 500);
        cancellation.Cancel();

        var error = await Assert.ThrowsAsync<OperationCanceledException>(() => canceled);
        Assert.Equal(cancellation.Token, error.CancellationToken);
        await Assert.ThrowsAsync<TimeoutException>(() => timedOut);

        // Completing either request must leave the shared provider usable for the next one.
        var next = client.InvokeAsync(new ListParametersServiceRequest(), 0);
        await Assert.ThrowsAsync<TimeoutException>(() => next);
    }

    [Fact]
    public async Task ClientTimeoutSurvivesNodeClose()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var clock = new RclClock(RclClockType.Steady);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName(), clockOverride: clock);
        using var client = node.CreateClient<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(
            NameGenerator.GenerateServiceName());

        var pending = client.InvokeAsync(new ListParametersServiceRequest(), 500);
        Assert.False(pending.IsCompleted);
        node.Dispose();

        await Assert.ThrowsAsync<TimeoutException>(() => pending);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task ClientAfterNodeClosePreparesTimeoutBeforeSending(bool closeClock)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var clock = new RclClock(RclClockType.Steady);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName(), clockOverride: clock);
        using var serverNode = context.CreateNode(NameGenerator.GenerateNodeName());
        var name = NameGenerator.GenerateServiceName();
        var received = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        using var server = serverNode.CreateService<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(
            name, (request, state) =>
            {
                received.TrySetResult();
                return new ListParametersServiceResponse();
            });
        using var client = node.CreateClient<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(name);
        Assert.True(await client.TryWaitForServerAsync(Timeout.Infinite));
        node.Dispose();

        if (closeClock)
        {
            clock.Dispose();
            await Assert.ThrowsAsync<ObjectDisposedException>(() =>
                client.InvokeAsync(new ListParametersServiceRequest(), RequestTimeout));
            await Task.Delay(500);
            Assert.False(received.Task.IsCompleted);
        }
        else
        {
            await client.InvokeAsync(new ListParametersServiceRequest(), RequestTimeout);
            Assert.True(received.Task.IsCompletedSuccessfully);
        }
    }

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

        Assert.True(await client.TryWaitForServerAsync(Timeout.Infinite));
        var actualResponse = await client.InvokeAsync(new ListParametersServiceRequest());

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

            var result = await client.TryWaitForServerAsync(Timeout.Infinite);
            Assert.True(result);
            await anotherContext.Yield();
            // Now we are on the event loop of anotherContext.

            // Captured the sync context of the anotherContext, we should be on the event loop of anotherContext.
            await client.InvokeAsync(new ListParametersServiceRequest());
            Assert.True(anotherContext.IsCurrent);

            // Suppressing the sync context.
            await client.InvokeAsync(new ListParametersServiceRequest()).ConfigureAwait(false);

            // No captured sync context, we should be on thread pool thread.
            await client.InvokeAsync(new ListParametersServiceRequest());
            Assert.False(context.IsCurrent);

            await client.InvokeAsync(new ListParametersServiceRequest()).ConfigureAwait(false);
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

        Assert.True(await client.TryWaitForServerAsync(Timeout.Infinite));
        var actualResponse = await client.InvokeAsync(new ListParametersServiceRequest());

        Assert.Equal(response.Result.Names, actualResponse.Result.Names);
        Assert.Equal(response.Result.Prefixes, actualResponse.Result.Prefixes);
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
            client.InvokeAsync(new ListParametersServiceRequest()));
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
            client.InvokeAsync(new ListParametersServiceRequest(), cts.Token));

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
            !g.Topics.Any(x => x.Name == introspectionTopic), Timeout.Infinite);
        Assert.True(success);

        client.ConfigureIntrospection(ServiceIntrospectionState.MetadataOnly);
        success = await node.Graph.TryWatchAsync((g, e) =>
            g.Topics.Any(x => x.Name == introspectionTopic &&
            x.Publishers.Count == 1 &&
            x.Publishers.First().Type == eventType), Timeout.Infinite);
        Assert.True(success);

        client.ConfigureIntrospection(ServiceIntrospectionState.Disabled);
        success = await node.Graph.TryWatchAsync((g, e) =>
            !g.Topics.Any(x => x.Name == introspectionTopic), Timeout.Infinite);
        Assert.True(success);

        client.ConfigureIntrospection(ServiceIntrospectionState.Full);
        success = await node.Graph.TryWatchAsync((g, e) =>
            g.Topics.Any(x => x.Name == introspectionTopic &&
            x.Publishers.Count == 1 &&
            x.Publishers.First().Type == eventType), Timeout.Infinite);
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
            !g.Topics.Any(x => x.Name == introspectionTopic), Timeout.Infinite);
        Assert.True(success);

        server.ConfigureIntrospection(ServiceIntrospectionState.MetadataOnly);
        success = await node.Graph.TryWatchAsync((g, e) =>
            g.Topics.Any(x => x.Name == introspectionTopic &&
            x.Publishers.Count == 1 &&
            x.Publishers.First().Type == eventType), Timeout.Infinite);
        Assert.True(success);

        server.ConfigureIntrospection(ServiceIntrospectionState.Disabled);
        success = await node.Graph.TryWatchAsync((g, e) =>
            !g.Topics.Any(x => x.Name == introspectionTopic), Timeout.Infinite);
        Assert.True(success);

        server.ConfigureIntrospection(ServiceIntrospectionState.Full);
        success = await node.Graph.TryWatchAsync((g, e) =>
            g.Topics.Any(x => x.Name == introspectionTopic &&
            x.Publishers.Count == 1 &&
            x.Publishers.First().Type == eventType), Timeout.Infinite);
        Assert.True(success);
    }

    [SkippableTheory]
    [InlineData(ServiceIntrospectionState.MetadataOnly)]
    [InlineData(ServiceIntrospectionState.Full)]
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

        var introspectTask = IntrospectService(4);
        Assert.True(await client.TryWaitForServerAsync(Timeout.Infinite));
        await client.InvokeAsync(new FrameGraphServiceRequest());

        await introspectTask;
        Assert.Equal(4, events.Count);

        // Client and server publish through separate DDS writers. Cross-writer arrival
        // order is not guaranteed; RCL also emits REQUEST_SENT after sending the request.
        var eventTypes = events.Keys.ToArray();
        Assert.Equal(new[]
        {
            ServiceEventInfo.REQUEST_SENT, ServiceEventInfo.REQUEST_RECEIVED,
            ServiceEventInfo.RESPONSE_SENT, ServiceEventInfo.RESPONSE_RECEIVED
        }.OrderBy(type => type), eventTypes.OrderBy(type => type));
        Assert.True(Array.IndexOf(eventTypes, ServiceEventInfo.REQUEST_SENT)
            < Array.IndexOf(eventTypes, ServiceEventInfo.RESPONSE_RECEIVED));
        Assert.True(Array.IndexOf(eventTypes, ServiceEventInfo.REQUEST_RECEIVED)
            < Array.IndexOf(eventTypes, ServiceEventInfo.RESPONSE_SENT));

        Assert.Equal(client.Gid, new(MemoryMarshal.Cast<sbyte, byte>(events[ServiceEventInfo.REQUEST_SENT].Info.ClientGid)));
        Assert.Equal(client.Gid, new(MemoryMarshal.Cast<sbyte, byte>(events[ServiceEventInfo.RESPONSE_RECEIVED].Info.ClientGid)));
        Assert.Equal(events[ServiceEventInfo.REQUEST_RECEIVED].Info.ClientGid,
            events[ServiceEventInfo.RESPONSE_SENT].Info.ClientGid);

        foreach (var item in events.Values)
        {
            Assert.Equal(events[ServiceEventInfo.REQUEST_SENT].Info.SequenceNumber, item.Info.SequenceNumber);
        }

        if(state == ServiceIntrospectionState.Full)
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

        async Task IntrospectService(int expectedEvents)
        {
            using var sub = node.CreateSubscription<FrameGraphServiceEvent>(introspectionTopic, new(queueSize: 10));
            while (sub.Publishers < 2)
            {
                await Task.Delay(10);
            }
            Assert.True(sub.Publishers >= 2,
                $"Expected both service introspection publishers, but found {sub.Publishers}.");

            await foreach (var item in sub.ReadAllAsync())
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
