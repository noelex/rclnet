using Rcl.Graph;
using Rosidl.Messages.Builtin;

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

        using var cts = new CancellationTokenSource();
        var t = RunAnotherNodeAsync(nodeNameToBeWaited, cts.Token);

        // Looks like node discovery is much slower on foxy, need to set to larger timeout here.
        isOnline = await node.Graph.TryWaitForNodeAsync(fullyQualifiedName, 5000);
        Assert.True(isOnline);

        isOnline = await node.Graph.TryWaitForNodeAsync(fullyQualifiedName, 0);
        Assert.True(isOnline);

        cts.Cancel();
        await Task.WhenAny(t);

        // Wait until node disappears
        await node.Graph.TryWatchAsync((graph, e) =>
            e is NodeDisappearedEvent nde &&
            nde.Node.Name.FullyQualifiedName == nodeNameToBeWaited, 5000);

        isOnline = await node.Graph.TryWaitForNodeAsync(fullyQualifiedName, 0);
        Assert.False(isOnline);

        static async Task RunAnotherNodeAsync(string nodeName, CancellationToken cancellationToken)
        {
            await Task.Delay(10, cancellationToken);

            await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
            using var node = ctx.CreateNode(nodeName);

            await Task.Delay(-1, cancellationToken);
        }
    }

    [Fact]
    public async Task WatchForTopic()
    {
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var targetTopic = "/" + NameGenerator.GenerateNodeName();
        var watcher = node.Graph.TryWatchAsync((graph, e) => graph.Topics.Any(x => x.Name == targetTopic), 1000);

        using var pub = node.CreatePublisher<Time>(targetTopic);

        Assert.True(await watcher);
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
}
