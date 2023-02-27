namespace Rcl.NET.Tests;

using Rosidl.Messages.Builtin;

public class PubSubTests
{
    [Fact]
    public async Task TestInterNodePubSub()
    {
        using var ctx = new RclContext(Array.Empty<string>());
        using var node1 = ctx.CreateNode("node1");
        using var node2 = ctx.CreateNode("node2");

        using var pub = node1.CreatePublisher<Time>("__test_publish_topic");
        using var sub = node2.CreateSubscription<Time>("__test_publish_topic");

        var task = ReadOneAsync();
        pub.Publish(new(sec: 1, nanosec: 2));
        var result = await task;

        Assert.Equal(1, result.Sec);
        Assert.Equal(2u, result.Nanosec);

        async Task<Time> ReadOneAsync()
        {
            await foreach (var m in sub.ReadAllAsync())
            {
                return m;
            }

            Assert.Fail("No message received from topic.");
            throw new NotImplementedException();
        }
    }
}