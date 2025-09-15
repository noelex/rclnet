namespace Rcl.NET.Tests;

using Rcl.Qos;
using Rosidl.Messages.Builtin;
using Xunit;
using Xunit.Abstractions;

[Collection("Sequential")]
public class PubSubTests
{
    [Fact]
    public async Task PubSubStronglyTypedMessages()
    {
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
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
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
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
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node1 = ctx.CreateNode(NameGenerator.GenerateNodeName());
        using var node2 = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var topic = NameGenerator.GenerateTopicName();

        // Request a large queue size to prevent messages being dropped,
        // because we'll be publishing messages really fast.
        var qos = new QosProfile(Reliability: ReliabilityPolicy.Reliable, Depth: 2000);

        Task<int[]> aggregateTask;
        using var pub = node1.CreatePublisher<Time>(topic, new(qos: qos));
        using (var sub = node2.CreateSubscription<Time>(topic, new(qos: qos, queueSize: 128)))
        {
            aggregateTask = Task.WhenAll(
                CountAsync(sub.ReadAllAsync()),
                CountAsync(sub.ReadAllAsync()),
                CountAsync(sub.ReadAllAsync()),
                CountAsync(sub.ReadAllAsync()));

            var msg = new Time(sec: 1, nanosec: 2);
            for (var i = 0; i < 100; i++)
            {
                pub.Publish(msg);
                await Task.Delay(1);
            }
        }

        var results = await aggregateTask;

        Assert.Equal(100, results.Sum(), TestConfig.GitHubActions ? 1.0 : 0);

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

    [SkippableFact]
    public async Task IncompatibleQosEvents()
    {
        Skip.If(RosEnvironment.IsFoxy && RosEnvironment.RmwImplementationIdentifier == "rmw_fastrtps_cpp",
            "Incompatible QoS event is not supported by rmw_fastrtps_cpp on foxy.");

        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
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
            .WaitAsync(TimeSpan.FromSeconds(5));

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

    [SkippableFact]
    public async Task IgnoreLocalPublications()
    {
        Skip.If(RosEnvironment.IsFoxy && RosEnvironment.RmwImplementationIdentifier == "rmw_fastrtps_cpp",
            "ignore_local_publications is not supported by rmw_fastrtps_cpp on foxy.");

        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var topic = NameGenerator.GenerateTopicName();
        using var pub = node.CreatePublisher<Time>(topic);

        Task<bool> t;
        using (var sub = node.CreateSubscription<Time>(topic, new(ignoreLocalPublications: true)))
        {

            t = ReadOneAsync(sub.ReadAllAsync());
            pub.Publish(new Time());

            await Task.Delay(100);
        }

        var result = await t;
        Assert.False(result);

        static async Task<bool> ReadOneAsync<T>(IAsyncEnumerable<T> subscription)
        {
            await foreach (var m in subscription)
            {
                return true;
            }

            return false;
        }
    }

    [SkippableFact]
    public async Task ContentFilteredSubscriptionsNoArguments()
    {
        Skip.If(RosEnvironment.IsFoxy, "Content filtered topic is only supported on humble or later.");
        Skip.If(RosEnvironment.RmwImplementationIdentifier == "rmw_cyclonedds_cpp", "Content filtered topic is not supported by rmw_cyclonedds_cpp.");

        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var qos = new QosProfile(Depth: 10, Reliability: ReliabilityPolicy.Reliable);
        var topic = NameGenerator.GenerateTopicName();
        using var pub = node.CreatePublisher<Time>(topic, new(qos: qos));

        Task<List<int>> t;
        using (var sub = node.CreateSubscription<Time>(topic,
            new(qos: qos, contentFilter: new("sec < 5"))))
        {
            t = CountMessagesAsync(sub.ReadAllAsync());
            for (var i = 0; i < 10; i++)
            {
                pub.Publish(new Time { Sec = i });
                await Task.Delay(10);
            }
        }

        var result = await t;
        Assert.NotEmpty(result);
        Assert.True(result.All(x => x < 5));

        static async Task<List<int>> CountMessagesAsync(IAsyncEnumerable<Time> subscription)
        {
            var result = new List<int>();
            await foreach (var m in subscription)
            {
                result.Add(m.Sec);
            }

            return result;
        }
    }

    [SkippableFact]
    public async Task ContentFilteredSubscriptions()
    {
        Skip.If(RosEnvironment.IsFoxy, "Content filtered topic is only supported on humble or later.");
        Skip.If(RosEnvironment.RmwImplementationIdentifier == "rmw_cyclonedds_cpp", "Content filtered topic is not supported by rmw_cyclonedds_cpp.");

        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var qos = new QosProfile(Depth: 10, Reliability: ReliabilityPolicy.Reliable);
        var topic = NameGenerator.GenerateTopicName();
        using var pub = node.CreatePublisher<Time>(topic, options: new(qos: qos));

        Task<List<int>> t;
        using (var sub = node.CreateSubscription<Time>(topic,
            new(qos: qos, contentFilter: new("sec > %0 AND sec < %1", "3", "7"))))
        {
            t = CountMessagesAsync(sub.ReadAllAsync());
            for (var i = 0; i < 10; i++)
            {
                pub.Publish(new Time { Sec = i });
                await Task.Delay(10);
            }
        }

        var result = await t;
        Assert.NotEmpty(result);
        Assert.True(result.All(x => x > 3 && x < 7));

        static async Task<List<int>> CountMessagesAsync(IAsyncEnumerable<Time> subscription)
        {
            var result = new List<int>();
            await foreach (var m in subscription)
            {
                result.Add(m.Sec);
            }

            return result;
        }
    }
    public static IEnumerable<object[]> Cases =>
            new List<object[]>
            {
            new object[] { "    "},
            new object[] { ""    },
            new object[] { "\t"  },
            new object[] { null  }
            };
    [Theory]
    [MemberData(nameof(Cases))]
    public void ContentFilteredOptionsEmptyExpression(string expression)
    {
        Assert.Throws<ArgumentException>(() => new ContentFilterOptions(expression));
    }

    [Fact]
    public void ContentFilteredOptionsTooManyArguments()
    {
        Assert.Throws<ArgumentException>(() => new ContentFilterOptions(
            "test", Enumerable.Range(0, 101).Select(x => string.Empty).ToArray()));
    }

    [SkippableFact]
    public async Task PublisherNetworkFlowEndpoints()
    {
        Skip.If(RosEnvironment.IsFoxy, "Network flow endpoints is only supported on humble or later.");
        Skip.If(RosEnvironment.RmwImplementationIdentifier == "rmw_cyclonedds_cpp", "rmw_cyclonedds_cpp does not support network flow endpoints.");

        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var topic = NameGenerator.GenerateTopicName();
        using var pub = node.CreatePublisher<Time>(topic);

        Assert.NotEmpty(pub.Endpoints);
    }

    [SkippableFact]
    public async Task SubscriptionNetworkFlowEndpoints()
    {
        Skip.If(RosEnvironment.IsFoxy, "Network flow endpoints is only supported on humble or later.");
        Skip.If(RosEnvironment.RmwImplementationIdentifier == "rmw_cyclonedds_cpp", "rmw_cyclonedds_cpp does not support network flow endpoints.");

        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var topic = NameGenerator.GenerateTopicName();
        using var sub = node.CreateSubscription<Time>(topic);

        Assert.NotEmpty(sub.Endpoints);
    }

    [SkippableFact]
    public async Task SubscriptionUniqueNetworkFlowEndpoints()
    {
        Skip.If(RosEnvironment.IsFoxy, "Network flow endpoints is only supported on humble or later.");
        Skip.If(RosEnvironment.RmwImplementationIdentifier == "rmw_cyclonedds_cpp", "rmw_cyclonedds_cpp does not support network flow endpoints.");

        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var topic = NameGenerator.GenerateTopicName();
        using var sub1 = node.CreateSubscription<Time>(topic);
        using var sub2 = node.CreateSubscription<Time>(topic, new(uniqueNetworkFlowEndpoints: UniquenessRequirement.StrictlyRequired));

        Assert.Empty(sub1.Endpoints.Intersect(sub2.Endpoints));
    }

    [SkippableFact]
    public async Task PublisherUniqueNetworkFlowEndpoints()
    {
        Skip.If(RosEnvironment.IsFoxy, "Network flow endpoints is only supported on humble or later.");
        Skip.If(RosEnvironment.RmwImplementationIdentifier == "rmw_cyclonedds_cpp", "rmw_cyclonedds_cpp does not support network flow endpoints.");
        Skip.If(RosEnvironment.RmwImplementationIdentifier == "rmw_fastrtps_cpp", "rmw_fastrtps_cpp does not support unique network flow endpoints on publishers.");

        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        using var node = ctx.CreateNode(NameGenerator.GenerateNodeName());

        var topic = NameGenerator.GenerateTopicName();
        using var pub1 = node.CreatePublisher<Time>(topic);
        using var pub2 = node.CreatePublisher<Time>(topic, new(uniqueNetworkFlowEndpoints: UniquenessRequirement.StrictlyRequired));

        Assert.Empty(pub1.Endpoints.Intersect(pub2.Endpoints));
    }
}