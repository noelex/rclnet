namespace Rcl.NET.Tests;

using Rosidl.Messages.Builtin;

public class PubSubTests
{
    [Fact]
    public async Task PubSubStronglyTypedMessages()
    {
        using var ctx = new RclContext(Array.Empty<string>());
        using var node1 = ctx.CreateNode("node1");
        using var node2 = ctx.CreateNode("node2");

        using var pub = node1.CreatePublisher<Time>("__test_publish_topic");
        using var sub = node2.CreateSubscription<Time>("__test_publish_topic");

        var task = ReadOneAsync(sub.ReadAllAsync());
        pub.Publish(new(sec: 1, nanosec: 2));
        var result = await task;

        Assert.Equal(1, result.Sec);
        Assert.Equal(2u, result.Nanosec);

        static async Task<T> ReadOneAsync<T>(IAsyncEnumerable<T> subscription)
        {
            await foreach (var m in subscription)
            {
                return m;
            }

            Assert.Fail("No message received from topic.");
            throw new NotImplementedException();
        }
    }

    [Fact]
    public async Task PubSubNativeMessages()
    {
        using var ctx = new RclContext(Array.Empty<string>());
        using var node1 = ctx.CreateNode("node1");
        using var node2 = ctx.CreateNode("node2");

        using var pub = node1.CreatePublisher<Time>("__test_publish_topic");
        using var sub = node2.CreateNativeSubscription<Time>("__test_publish_topic");

        using var buffer = RosMessageBuffer.Create<Time>();
        buffer.AsRef<Time.Priv>().Sec = 1;
        buffer.AsRef<Time.Priv>().Nanosec = 2;

        var task = ReadOneAsync(sub.ReadAllAsync());
        pub.Publish(buffer);
        var result = await task;

        Assert.Equal(1, result.Sec);
        Assert.Equal(2u, result.Nanosec);

        static async Task<Time.Priv> ReadOneAsync(IAsyncEnumerable<RosMessageBuffer> subscription)
        {
            await foreach (var m in subscription)
            {
                using (m) return m.AsRef<Time.Priv>();
            }

            Assert.Fail("No message received from topic.");
            throw new NotImplementedException();
        }
    }
}