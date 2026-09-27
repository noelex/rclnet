using Rcl.Actions;
using Rcl.Graph;
using Rosidl.Messages.Builtin;
using Rosidl.Messages.Tf2;
using Xunit.Abstractions;

namespace Rcl.NET.Tests;

public class RosGraphTests(ITestOutputHelper output)
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    public async Task EndpointChangesOnlyReplaceAffectedCollections(int removed)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        var topicName = NameGenerator.GenerateTopicName();
        var serviceName = NameGenerator.GenerateServiceName();
        var actionName = NameGenerator.GenerateActionName();
        using var publisher = node.CreatePublisher<Time>(topicName);
        using var subscriber = node.CreateSubscription<Time>(topicName);
        using var server = node.CreateService<FrameGraphService, FrameGraphServiceRequest, FrameGraphServiceResponse>(
            serviceName, static (request, state) => new());
        using var client = node.CreateClient<FrameGraphService, FrameGraphServiceRequest, FrameGraphServiceResponse>(serviceName);
        using var actionServer = node.CreateActionServer<LookupTransformAction>(actionName, new DummyActionServer());
        var actionClient = node.CreateActionClient<LookupTransformAction, LookupTransformActionGoal,
            LookupTransformActionResult, LookupTransformActionFeedback>(actionName);
        var clientDisposed = false;

        try
        {
            var graph = new RosGraph((Rcl.Internal.RclNodeImpl)node, name => name.Name == node.Name);
            await context.Yield();
            graph.Build();
            var graphNode = Assert.Single(graph.Nodes);
            var topic = graph.Topics.Single(x => x.Name == publisher.Name);
            var service = graph.Services.Single(x => x.Name == client.Name);
            var action = Assert.Single(graph.Actions);
            Func<object>[] nodeGetters =
            [
                () => graphNode.Publishers, () => graphNode.Subscribers,
                () => graphNode.Servers, () => graphNode.Clients,
                () => graphNode.ActionServers, () => graphNode.ActionClients
            ];
            Func<object>[] endpointGetters =
            [
                () => topic.Publishers, () => topic.Subscribers,
                () => service.Servers, () => service.Clients,
                () => action.Servers, () => action.Clients
            ];
            var nodeSnapshots = nodeGetters.Select(get => get()).ToArray();
            var endpointSnapshots = endpointGetters.Select(get => get()).ToArray();
            IDisposable[] endpoints = [publisher, subscriber, server, client, actionServer, actionClient];
            clientDisposed = removed == 5;
            endpoints[removed].Dispose();
            await context.Yield();
            graph.Build();

            for (var i = 0; i < endpointGetters.Length; i++)
            {
                if (i == removed)
                {
                    Assert.NotSame(endpointSnapshots[i], endpointGetters[i]());
                    Assert.Empty((System.Collections.IEnumerable)endpointGetters[i]());
                    Assert.NotSame(nodeSnapshots[i], nodeGetters[i]());
                }
                else
                {
                    Assert.Same(endpointSnapshots[i], endpointGetters[i]());
                    // Action removal also removes its underlying topic and service endpoints.
                    if (removed < 4 || i >= 4)
                    {
                        Assert.Same(nodeSnapshots[i], nodeGetters[i]());
                    }
                }

                Assert.Single((System.Collections.IEnumerable)endpointSnapshots[i]);
            }
        }
        finally
        {
            if (!clientDisposed)
            {
                actionClient.Dispose();
            }
        }
    }

    [Fact]
    public async Task RemovedNodePublishesEmptyActionCollections()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var owner = context.CreateNode(NameGenerator.GenerateNodeName());
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        var actionName = NameGenerator.GenerateActionName();
        using var server = node.CreateActionServer<LookupTransformAction>(actionName, new DummyActionServer());
        var client = node.CreateActionClient<LookupTransformAction, LookupTransformActionGoal,
            LookupTransformActionResult, LookupTransformActionFeedback>(actionName);
        var clientDisposed = false;

        try
        {
            var graph = new RosGraph((Rcl.Internal.RclNodeImpl)owner, name => name.Name == node.Name);
            await context.Yield();
            graph.Build();
            var graphNode = Assert.Single(graph.Nodes);
            var action = Assert.Single(graph.Actions);
            var oldServers = graphNode.ActionServers;
            var oldClients = graphNode.ActionClients;
            server.Dispose();
            clientDisposed = true;
            client.Dispose();
            node.Dispose();
            await context.Yield();
            graph.Build();
            Assert.Empty(graph.Nodes);
            Assert.Empty(graph.Actions);
            Assert.Empty(graphNode.ActionServers);
            Assert.Empty(graphNode.ActionClients);
            Assert.Empty(action.Servers);
            Assert.Empty(action.Clients);
            Assert.Single(oldServers);
            Assert.Single(oldClients);
        }
        finally
        {
            if (!clientDisposed)
            {
                client.Dispose();
            }
        }
    }

    [Fact]
    public async Task CollectionsAreCachedReadOnlySnapshots()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        var topicName = NameGenerator.GenerateTopicName();
        var serviceName = NameGenerator.GenerateServiceName();
        var actionName = NameGenerator.GenerateActionName();
        using var publisher = node.CreatePublisher<Time>(topicName);
        using var subscriber = node.CreateSubscription<Time>(topicName);
        using var server = node.CreateService<FrameGraphService, FrameGraphServiceRequest, FrameGraphServiceResponse>(
            serviceName, static (request, state) => new());
        using var client = node.CreateClient<FrameGraphService, FrameGraphServiceRequest, FrameGraphServiceResponse>(serviceName);
        using var actionServer = node.CreateActionServer<LookupTransformAction>(actionName, new DummyActionServer());
        var graph = new RosGraph((Rcl.Internal.RclNodeImpl)node, name => name.Name == node.Name);
        RosNode graphNode;
        RosTopic topic;
        RosService service;
        RosAction action;
        IReadOnlyCollection<RosTopicEndPoint> oldPublishers;
        IReadOnlyCollection<RosServiceEndPoint> oldServers;
        IReadOnlyCollection<RosActionEndPoint> oldActionClients;
        using (var actionClient = node.CreateActionClient<LookupTransformAction, LookupTransformActionGoal,
            LookupTransformActionResult, LookupTransformActionFeedback>(actionName))
        {
            await context.Yield();
            graph.Build();
            graphNode = Assert.Single(graph.Nodes);
            topic = graph.Topics.Single(x => x.Name == publisher.Name);
            service = graph.Services.Single(x => x.Name == client.Name);
            action = Assert.Single(graph.Actions);
            Func<object>[] getters =
            [
                () => graph.Nodes, () => graph.Topics, () => graph.Services, () => graph.Actions,
                () => graphNode.Publishers, () => graphNode.Subscribers,
                () => graphNode.Servers, () => graphNode.Clients,
                () => graphNode.ActionServers, () => graphNode.ActionClients,
                () => topic.Publishers, () => topic.Subscribers,
                () => service.Servers, () => service.Clients,
                () => action.Servers, () => action.Clients,
            ];
            var meter = new AllocationMeter(output);
            var snapshots = getters.Select(get => get()).ToArray();
            graph.Build();

            for (int i = 0; i < getters.Length; i++)
            {
                Assert.NotEmpty((System.Collections.IEnumerable)snapshots[i]);
                Assert.Same(snapshots[i], getters[i]());
                object? snapshot = null;

                meter.Measure($"graph-collection-{i}-read", 1000, () => snapshot = getters[i](), zeroAllocation: true);
                GC.KeepAlive(snapshot);
            }

            oldPublishers = topic.Publishers;
            oldServers = service.Servers;
            oldActionClients = action.Clients;
            Assert.Single(oldPublishers);
            Assert.Single(oldServers);
            Assert.Single(oldActionClients);
            Assert.Throws<NotSupportedException>(() => ((ICollection<RosTopicEndPoint>)oldPublishers).Clear());
        }

        publisher.Dispose();
        subscriber.Dispose();
        server.Dispose();
        client.Dispose();
        actionServer.Dispose();
        await context.Yield();
        Assert.Same(oldPublishers, topic.Publishers);
        bool notified = false;
        graph.GraphChanged += change =>
        {
            notified = true;
            Assert.Empty(topic.Publishers);
            Assert.Empty(topic.Subscribers);
            Assert.Empty(service.Servers);
            Assert.Empty(service.Clients);
            Assert.Empty(action.Servers);
            Assert.Empty(action.Clients);
            Assert.Empty(graphNode.ActionServers);
            Assert.Empty(graphNode.ActionClients);
        };
        graph.Build();

        Assert.True(notified);
        Assert.DoesNotContain(topic, graph.Topics);
        Assert.DoesNotContain(service, graph.Services);
        Assert.Empty(graph.Actions);
        Assert.Single(oldPublishers);
        Assert.Single(oldServers);
        Assert.Single(oldActionClients);
    }

    [Fact]
    public async Task FailedBuildKeepsPublishedCollectionsUntilRecovery()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        bool fail = false;
        string? addedNodeName = null;
        var includedNodes = new HashSet<string> { node.Name };
        await context.Yield();
        var graph = new RosGraph((Rcl.Internal.RclNodeImpl)node, name =>
        {
            if (!includedNodes.Contains(name.Name))
            {
                return false;
            }

            if (fail && addedNodeName is not null)
            {
                throw new InvalidOperationException("Simulated graph build failure.");
            }

            if (fail && name.Name != node.Name)
            {
                addedNodeName = name.Name;
            }

            return true;
        });
        graph.Build();
        var published = graph.Nodes;
        using var first = context.CreateNode(NameGenerator.GenerateNodeName());
        using var second = context.CreateNode(NameGenerator.GenerateNodeName());
        var serviceName = NameGenerator.GenerateServiceName();
        using var firstServer = first.CreateService<FrameGraphService, FrameGraphServiceRequest, FrameGraphServiceResponse>(
            serviceName, static (request, state) => new());
        using var secondServer = second.CreateService<FrameGraphService, FrameGraphServiceRequest, FrameGraphServiceResponse>(
            serviceName, static (request, state) => new());
        includedNodes.Add(first.Name);
        includedNodes.Add(second.Name);
        fail = true;
        Assert.Throws<InvalidOperationException>(graph.Build);
        Assert.Same(published, graph.Nodes);

        // Leave only the node already staged by the failed build, so recovery has no new node to add.
        var stagedNode = addedNodeName == first.Name ? first : second;
        var unstagedNode = addedNodeName == first.Name ? second : first;
        if (unstagedNode == first)
        {
            firstServer.Dispose();
        }
        else
        {
            secondServer.Dispose();
        }

        unstagedNode.Dispose();
        await context.Yield();
        fail = false;
        graph.Build();
        var recoveredNode = graph.Nodes.Single(x => x.Name.Name == stagedNode.Name);
        Assert.Contains(recoveredNode.Servers, x => x.Service.Name == firstServer.Name);
        var recoveredService = graph.Services.Single(x => x.Name == firstServer.Name);
        Assert.Contains(recoveredService.Servers, x => x.Node == recoveredNode);
        Assert.DoesNotContain(graph.Nodes, x => x.Name.Name == unstagedNode.Name);
        Assert.DoesNotContain(published, x => x.Name.Name == first.Name || x.Name.Name == second.Name);
    }

    [Fact]
    public async Task ServiceChangesReachEveryNodeInTheSameContext()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var first = context.CreateNode(NameGenerator.GenerateNodeName());
        using var second = context.CreateNode(NameGenerator.GenerateNodeName());
        using var third = context.CreateNode(NameGenerator.GenerateNodeName());
        var nodes = new[] { first, second, third };
        var serviceName = "/" + NameGenerator.GenerateServiceName().TrimStart('/');

        // Register every watcher before changing the graph. A shared native guard must
        // notify every node, even when no subsequent graph change can wake a missed waiter.
        await context.Yield();
        var appeared = nodes.Select(node => node.Graph.TryWaitForServiceServerAsync(serviceName, 5000)).ToArray();
        using var server = first.CreateService<
            Rosidl.Messages.Rcl.ListParametersService,
            Rosidl.Messages.Rcl.ListParametersServiceRequest,
            Rosidl.Messages.Rcl.ListParametersServiceResponse>(serviceName,
            (request, state) => new Rosidl.Messages.Rcl.ListParametersServiceResponse());

        Assert.All(await Task.WhenAll(appeared), found => Assert.True(found));

        await context.Yield();
        var disappeared = nodes.Select(node => node.Graph.TryWatchAsync(
            (graph, change) => !graph.IsServiceServerAvailable(serviceName), 5000)).ToArray();
        server.Dispose();

        Assert.All(await Task.WhenAll(disappeared), removed => Assert.True(removed));
    }

    [Fact]
    public async Task TestWaitForNode()
    {
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var nodeNameToBeWaited = NameGenerator.GenerateNodeName();
        var fullyQualifiedName = "/" + nodeNameToBeWaited;

        var isOnline = await node.Graph.TryWaitForNodeAsync(fullyQualifiedName, 0);
        Assert.False(isOnline);

        // Looks like node discovery is much slower on foxy, need to set to larger timeout here.
        var watcher = node.Graph.TryWaitForNodeAsync(fullyQualifiedName, 5000);
        using var cts = new CancellationTokenSource();
        var t = RunInSeparateContext(async ctx =>
        {
            using var node = ctx.CreateNode(nodeNameToBeWaited);
            await Task.Delay(-1, cts.Token);
        });

        try
        {
            Assert.True(await watcher);
        }
        finally
        {
            cts.Cancel();
            await Task.WhenAny(t);
        }

        // Wait until node disappears
        await node.Graph.TryWatchAsync((graph, e) =>
            graph.Nodes.All(x => x.Name.FullyQualifiedName != fullyQualifiedName), 5000);

        isOnline = await node.Graph.TryWaitForNodeAsync(fullyQualifiedName, 0);
        Assert.False(isOnline);
    }

    [Fact]
    public async Task WatchForTopic()
    {
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var targetTopic = "/" + NameGenerator.GenerateTopicName();
        var appearWatcher = node.Graph.TryWatchAsync((graph, e) => graph.Topics.Any(x => x.Name == targetTopic), 1000);
        var disappearWatcher = node.Graph.TryWatchAsync((graph, e) => !graph.Topics.Any(x => x.Name == targetTopic), 1000);
        {
            using var pub = node.CreatePublisher<Time>(targetTopic);
            Assert.True(await appearWatcher);
        }
        Assert.True(await disappearWatcher);
    }

    [Fact]
    public async Task WatchForTopicPublisher()
    {
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var targetTopic = "/" + NameGenerator.GenerateTopicName();
        var appearWatcher = node.Graph.TryWatchAsync((graph, e) => e is PublisherAppearedEvent s && s.Publisher.Node.Name.FullyQualifiedName == node.FullyQualifiedName, 1000);
        var disappearWatcher = node.Graph.TryWatchAsync((graph, e) => e is PublisherDisappearedEvent s && s.Publisher.Node.Name.FullyQualifiedName == node.FullyQualifiedName, 1000);

        await ctx.Yield();
        {
            using var pub = node.CreatePublisher<Time>(targetTopic);
            Assert.True(await appearWatcher);
        }
        Assert.True(await disappearWatcher);
    }

    [Fact]
    public async Task WatchForTopicSubscriber()
    {
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var targetTopic = "/" + NameGenerator.GenerateTopicName();
        var appearWatcher = node.Graph.TryWatchAsync((graph, e) => e is SubscriberAppearedEvent s && s.Subscriber.Node.Name.FullyQualifiedName == node.FullyQualifiedName, 1000);
        var disappearWatcher = node.Graph.TryWatchAsync((graph, e) => e is SubscriberDisappearedEvent s && s.Subscriber.Node.Name.FullyQualifiedName == node.FullyQualifiedName, 1000);

        await ctx.Yield();
        {
            using var pub = node.CreateSubscription<Time>(targetTopic);
            Assert.True(await appearWatcher);
        }
        Assert.True(await disappearWatcher);
    }

    [Fact]
    public async Task WatchForServiceServer()
    {
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var serviceName = "/" + NameGenerator.GenerateServiceName();

        var serverAppearWatcher = node.Graph.TryWatchAsync(
            (graph, e) => graph.IsServiceServerAvailable(serviceName), 1000);
        var serviceAppearWatcher = node.Graph.TryWatchAsync(
            (graph, e) => graph.Services.Any(x => x.Name == serviceName), 1000);

        Task<bool> serverDisappearWatcher, serviceDisappearWatcher;
        {
            using var server = node.CreateService<
                FrameGraphService,
                FrameGraphServiceRequest,
                FrameGraphServiceResponse>(serviceName, (req, state) => new());

            Assert.All(await Task.WhenAll(serverAppearWatcher, serviceAppearWatcher), Assert.True);

            serverDisappearWatcher = node.Graph.TryWatchAsync(
                (graph, e) => !graph.IsServiceServerAvailable(serviceName), 1000);
            serviceDisappearWatcher = node.Graph.TryWatchAsync(
               (graph, e) => !graph.Services.Any(x => x.Name == serviceName), 1000);
        }

        Assert.All(await Task.WhenAll(serverDisappearWatcher, serviceDisappearWatcher), Assert.True);
    }

    [Fact]
    public async Task WatchForServiceClient()
    {
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var serviceName = "/" + NameGenerator.GenerateServiceName();

        var clientAppearWatcher = node.Graph.TryWatchAsync(
            (graph, e) => e is ClientAppearedEvent s && s.Client.Service.Name == serviceName, 1000);
        var serviceAppearWatcher = node.Graph.TryWatchAsync(
            (graph, e) => e is ServiceAppearedEvent s && s.Service.Name == serviceName, 1000);

        // Yield here to make sure that CreateClient is called
        // after TryWatchAsync internally sets up the subscriber to receive events on the event loop.
        // Otherwise we might miss those events.
        await ctx.Yield();

        Task<bool> clientDisappearWatcher, serviceDisappearWatcher;
        {
            using var server = node.CreateClient<
                FrameGraphService,
                FrameGraphServiceRequest,
                FrameGraphServiceResponse>(serviceName);

            Assert.All(await Task.WhenAll(clientAppearWatcher, serviceAppearWatcher), Assert.True);

            clientDisappearWatcher = node.Graph.TryWatchAsync(
                (graph, e) => e is ClientDisappearedEvent s && s.Client.Service.Name == serviceName, 1000);
            serviceDisappearWatcher = node.Graph.TryWatchAsync(
               (graph, e) => e is ServiceDisappearedEvent s && s.Service.Name == serviceName, 1000);
        }

        Assert.All(await Task.WhenAll(clientDisappearWatcher, serviceDisappearWatcher), Assert.True);
    }

    [Fact]
    public async Task WatchForActionServer()
    {
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var serviceName = "/" + NameGenerator.GenerateActionName();

        var serverAppearWatcher = node.Graph.TryWatchAsync(
            (graph, e) => graph.IsActionServerAvailable(serviceName), 1000);
        var serviceAppearWatcher = node.Graph.TryWatchAsync(
            (graph, e) => graph.Actions.Any(x => x.Name == serviceName), 1000);

        await ctx.Yield();
        Task<bool> serverDisappearWatcher, serviceDisappearWatcher;
        {
            using var server = node.CreateActionServer<LookupTransformAction>(serviceName, new DummyActionServer());

            Assert.All(await Task.WhenAll(serverAppearWatcher, serviceAppearWatcher), Assert.True);

            serverDisappearWatcher = node.Graph.TryWatchAsync(
                (graph, e) => !graph.IsActionServerAvailable(serviceName), 1000);
            serviceDisappearWatcher = node.Graph.TryWatchAsync(
               (graph, e) => !graph.Actions.Any(x => x.Name == serviceName), 1000);
        }

        Assert.All(await Task.WhenAll(serverDisappearWatcher, serviceDisappearWatcher), Assert.True);
    }

    [Fact]
    public async Task WatchForActionClient()
    {
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());
        var serviceName = "/" + NameGenerator.GenerateActionName();

        var clientAppearWatcher = node.Graph.TryWatchAsync(
            (graph, e) => e is ActionClientAppearedEvent s && s.ActionClient.Action.Name == serviceName, 1000);
        var serviceAppearWatcher = node.Graph.TryWatchAsync(
            (graph, e) => e is ActionAppearedEvent s && s.Action.Name == serviceName, 1000);
        var clientDisappearWatcher = node.Graph.TryWatchAsync(
           (graph, e) => e is ActionClientDisappearedEvent s && s.ActionClient.Action.Name == serviceName, 1000);
        var serviceDisappearWatcher = node.Graph.TryWatchAsync(
          (graph, e) => e is ActionDisappearedEvent s && s.Action.Name == serviceName, 1000);

        await ctx.Yield();
        using (var server = node.CreateActionClient<
                LookupTransformAction,
                LookupTransformActionGoal,
                LookupTransformActionResult,
                LookupTransformActionFeedback>(serviceName))
        {
            Assert.All(await Task.WhenAll(clientAppearWatcher, serviceAppearWatcher), Assert.True);
        }

        Assert.True(await clientDisappearWatcher);
        Assert.True(await serviceDisappearWatcher);
    }

    [Fact]
    public async Task WatchForTopicInSeparateContext()
    {
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var targetTopic = "/" + NameGenerator.GenerateTopicName();
        var watcher = node.Graph.TryWatchAsync((graph, e) => graph.Topics.Any(x => x.Name == targetTopic), 1000);

        using var cts = new CancellationTokenSource();
        var t = RunInSeparateContext(async ctx =>
        {
            using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());
            using var pub = node.CreatePublisher<Time>(targetTopic);
            await Task.Delay(-1, cts.Token);
        });

        try
        {
            Assert.True(await watcher);
        }
        finally
        {
            cts.Cancel();
            await Task.WhenAny(t);
        }
    }

    [Fact]
    public async Task WatchForNode()
    {
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var targetNodeName = NameGenerator.GenerateNodeName();
        var watcher = node.Graph.TryWatchAsync((graph, e) => graph.Nodes.Any(x => x.Name.Name == targetNodeName), 1000);

        using var targetNode = ctx.CreateNode(targetNodeName);

        Assert.True(await watcher);
    }

    [Fact]
    public async Task WatchForNodeInSeparateContext()
    {
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var targetNodeName = NameGenerator.GenerateNodeName();
        var watcher = node.Graph.TryWatchAsync((graph, e) => graph.Nodes.Any(x => x.Name.Name == targetNodeName), 1000);

        using var cts = new CancellationTokenSource();
        var t = RunInSeparateContext(async ctx =>
        {
            using var node = ctx.CreateNode(targetNodeName);
            await Task.Delay(-1, cts.Token);
        });

        try
        {
            Assert.True(await watcher);
        }
        finally
        {
            cts.Cancel();
            await Task.WhenAny(t);
        }
    }

    [Fact]
    public async Task WatchTimeout()
    {
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var targetTopic = "/" + NameGenerator.GenerateNodeName();
        var ok = await node.Graph.TryWatchAsync((graph, e) => graph.Topics.Any(x => x.Name == targetTopic), 100);

        Assert.False(ok);
    }

    [Fact]
    public async Task WatchTimeoutThrows()
    {
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var targetTopic = "/" + NameGenerator.GenerateNodeName();
        await Assert.ThrowsAsync<TimeoutException>(() =>
            node.Graph.WatchAsync((graph, e) => graph.Topics.Any(x => x.Name == targetTopic), 100));
    }

    [SkippableFact]
    public async Task PublisherGid()
    {
        // TODO: Track https://github.com/ros2/rmw_cyclonedds/issues/446
        Skip.If(RosEnvironment.RmwImplementationIdentifier == "rmw_cyclonedds_cpp", "rmw_get_gid_for_publisher is broken in 'rmw_cyclonedds_cpp'.");

        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var topic = "/" + NameGenerator.GenerateTopicName();
        using var pub = node.CreatePublisher<Time>(topic);

        var found = await node.Graph.TryWatchAsync((graph, e) =>
            graph.Topics.FirstOrDefault(x => x.Name == topic)?.Publishers?.Any() == true, 5000);

        Assert.True(found);
        Assert.Equal(pub.Gid, node.Graph.Topics.Single(x => x.Name == topic).Publishers.Single().Gid);
    }

    private async Task RunInSeparateContext(Func<RclContext, Task> action)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        await action(context);
    }

    private class DummyActionServer : INativeActionGoalHandler
    {
        public bool CanAccept(Guid id, RosMessageBuffer goal)
        {
            throw new NotImplementedException();
        }

        public Task ExecuteAsync(INativeActionGoalController controller, RosMessageBuffer goal, RosMessageBuffer result, CancellationToken cancellationToken)
        {
            throw new NotImplementedException();
        }

        public void OnAccepted(INativeActionGoalController controller)
        {
            throw new NotImplementedException();
        }

        public void OnCompleted(INativeActionGoalController controller)
        {
            throw new NotImplementedException();
        }
    }
}
