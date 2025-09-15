using Rcl.Graph;
using Rosidl.Messages.Builtin;
using System.Threading;
using Xunit.Abstractions;

namespace Rcl.NET.Tests;

[Collection("Sequential")]
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
        var watcher = node.Graph.TryWatchAsync((graph, e) => graph.Topics.Any(x => x.Name == targetTopic), 1000);

        using var pub = node.CreatePublisher<Time>(targetTopic);

        Assert.True(await watcher);
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
}
