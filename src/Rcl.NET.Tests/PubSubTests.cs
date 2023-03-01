namespace Rcl.NET.Tests;

using Rcl.Qos;
using Rosidl.Messages.Builtin;

public class PubSubTests
{
    [Fact]
    public async Task PubSubStronglyTypedMessages()
    {
        using var ctx=new RclContext(Array.Empty<string>());
        using var node1 = ctx.CreateNode(NameGenerator.GenerateNodeName());
        using var node2 = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var topic = NameGenerator.GenerateTopicName();
        using var pub = node1.CreatePublisher<Time>(topic);
        using var sub = node2.CreateSubscription<Time>(topic);

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
        using var node1 = ctx.CreateNode(NameGenerator.GenerateNodeName());
        using var node2 = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var topic = NameGenerator.GenerateTopicName();
        using var pub = node1.CreatePublisher<Time>(topic);
        using var sub = node2.CreateNativeSubscription<Time>(topic);

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

    [Fact]
    public async Task ConcurrentCallsToReadAllAsync()
    {
        using var ctx = new RclContext(Array.Empty<string>());
        using var node1 = ctx.CreateNode(NameGenerator.GenerateNodeName());
        using var node2 = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var topic = NameGenerator.GenerateTopicName();

        // Request a large queue size to prevent messages being dropped,
        // because we'll be publishing messages really fast.
        var qos = new QosProfile(Reliability: ReliabilityPolicy.Reliable,Å@Depth: 2000);

        Task<int[]> aggregateTask;
        using var pub = node1.CreatePublisher<Time>(topic, new(qos: qos));       
        using (var sub = node2.CreateSubscription<Time>(topic, new(qos: qos)))
        {
            aggregateTask = Task.WhenAll(
                CountAsync(sub.ReadAllAsync()),
                CountAsync(sub.ReadAllAsync()),
                CountAsync(sub.ReadAllAsync()),
                CountAsync(sub.ReadAllAsync()));

            var msg = new Time(sec: 1, nanosec: 2);
            for (var i = 0; i < 1000; i++)
            {
                pub.Publish(msg);
            }

            // Make sure the subscription has enough time
            // to read all messages.
            await Task.Delay(200);
        }

        var results = await aggregateTask;

        Assert.True(results[0] > 0);
        Assert.True(results[1] > 0);
        Assert.True(results[2] > 0);
        Assert.True(results[3] > 0);
        Assert.Equal(1000, results.Sum());

        static async Task<int> CountAsync<T>(IAsyncEnumerable<T> subscription)
        {
            var cnt = 0;
            await foreach (var m in subscription)
            {
                cnt++;
            }

            return cnt;
        }
    }

    [Fact]
    public async Task IncompatibleQosEvents()
    {
        using var ctx = new RclContext(Array.Empty<string>());
        using var node1 = ctx.CreateNode(NameGenerator.GenerateNodeName());
        using var node2 = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var topic = NameGenerator.GenerateTopicName();
        TaskCompletionSource<QosPolicyKind> offeredQosIncompatible = new(), requestQosIncompatible = new();
        
        // Pub = BestEffort and Sub = Reliable is incompatible.
        using var pub = node1.CreatePublisher<Time>(topic, new(
            qos: new(Reliability: ReliabilityPolicy.BestEffort),
            offeredQosIncompatibleHandler: OnOfferedQosIncompatible));

        using var sub = node2.CreateSubscription<Time>(topic, new(
            qos: new(Reliability: ReliabilityPolicy.Reliable),
            requestedQosIncompatibleHandler: OnRequestedQosIncompatible));

        await Task.WhenAll(offeredQosIncompatible.Task, requestQosIncompatible.Task)
            .WaitAsync(TimeSpan.FromSeconds(1));

        Assert.Equal(QosPolicyKind.Reliability, offeredQosIncompatible.Task.Result);
        Assert.Equal(QosPolicyKind.Reliability, requestQosIncompatible.Task.Result);

        void OnOfferedQosIncompatible(IncompatibleQosEvent e)
        {
            offeredQosIncompatible.TrySetResult(e.LastPolicyKind);
        }

        void OnRequestedQosIncompatible(IncompatibleQosEvent e)
        {
            requestQosIncompatible.TrySetResult(e.LastPolicyKind);
        }
    }
}