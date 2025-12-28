using Rcl.Actions;
using Rcl.Graph;
using Rosidl.Messages.Builtin;
using Rosidl.Messages.Tf2;

namespace Rcl.NET.Tests;

public class RosGraphTests
{
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

        var serviceName = "/" + NameGenerator.GenerateServiceName();

        var serverAppearWatcher = node.Graph.TryWatchAsync(
            (graph, e) => graph.IsActionServerAvailable(serviceName), 1000);
        var serviceAppearWatcher = node.Graph.TryWatchAsync(
            (graph, e) => graph.Actions.Any(x => x.Name == serviceName), 1000);

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

        var serviceName = "/action_client_test";

        var clientAppearWatcher = node.Graph.TryWatchAsync(
            (graph, e) => e is ActionClientAppearedEvent s && s.ActionClient.Action.Name == serviceName, 1000);
        var serviceAppearWatcher = node.Graph.TryWatchAsync(
            (graph, e) => e is ActionAppearedEvent s && s.Action.Name == serviceName, 1000);

        await ctx.Yield();
        Task<bool> clientDisappearWatcher, serviceDisappearWatcher;
        {
            using var server = node.CreateActionClient<
                LookupTransformAction,
                LookupTransformActionGoal,
                LookupTransformActionResult,
                LookupTransformActionFeedback>(serviceName);

            clientDisappearWatcher = node.Graph.TryWatchAsync(
               (graph, e) => e is ActionClientDisappearedEvent s && s.ActionClient.Action.Name == serviceName, 1000);
            serviceDisappearWatcher = node.Graph.TryWatchAsync(
              (graph, e) => e is ActionDisappearedEvent s && s.Action.Name == serviceName, 1000);

            Assert.All(await Task.WhenAll(clientAppearWatcher, serviceAppearWatcher), Assert.True);


        }

        Assert.All(await Task.WhenAll(clientDisappearWatcher, serviceDisappearWatcher), Assert.True);
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

        var watcher = node.Graph.TryWatchAsync((graph, e) =>
            graph.Topics.FirstOrDefault(x => x.Name == topic)?.Publishers?.Any() == true, -1);
        using var cts = new CancellationTokenSource();

        var gidTask = new TaskCompletionSource<GraphId>();
        var t = RunInSeparateContext(async ctx =>
        {
            using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());
            using var pub = node.CreatePublisher<Time>(topic);
            gidTask.SetResult(pub.Gid);
            await Task.Delay(-1, cts.Token);
        });

        try
        {
            Assert.True(await watcher);
            Assert.Equal(node.Graph.Topics.Single(x => x.Name == topic).Publishers.Single().Gid, await gidTask.Task);
        }
        finally
        {
            cts.Cancel();
            await Task.WhenAny(t);
        }
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
