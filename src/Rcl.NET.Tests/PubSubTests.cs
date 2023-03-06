namespace Rcl.NET.Tests;

using Rcl.Qos;
using Rosidl.Messages.Builtin;
using Rosidl.Messages.Sensor;
using System.Diagnostics;
using Xunit.Abstractions;

public class PubSubTests
{
    private readonly ITestOutputHelper _output;

    public PubSubTests(ITestOutputHelper output)
    {
        _output = output;
    }

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

    [Fact]
    public async Task IgnoreLocalPublications()
    {
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

    [Fact]
    public async Task VeryLargeMessages()
    {
        await using var ctx = new RclContext(TestConfig.DefaultContextArguments);
        await using var ctx2 = new RclContext(TestConfig.DefaultContextArguments);
        using var node1 = ctx.CreateNode(NameGenerator.GenerateNodeName());
        using var node2 = ctx2.CreateNode(NameGenerator.GenerateNodeName());

        var topic = NameGenerator.GenerateTopicName();
        var qos = new QosProfile(Depth: 2000);

        using var pub = node1.CreatePublisher<Image>(topic, new(qos: qos));

        Task<List<TimeSpan>> task;
        using (var sub = node2.CreateNativeSubscription<Image>(topic, new(qos: qos, queueSize: 128)))
        {
            task = CountAverageLatency(node2.Clock, sub.ReadAllAsync());

            // limit to 100 fps.
            using var timer = new PeriodicTimer(TimeSpan.FromMilliseconds(10));
            var body = new byte[3 * 1920 * 1080];

            var nativeMsg = RosMessageBuffer.Create<Image>();
            nativeMsg.AsRef<Image.Priv>().Data.CopyFrom(body);
            for (var i = 0; i < 100; i++)
            {
                var now = (long)node1.Clock.Elapsed.TotalNanoseconds;
                nativeMsg.AsRef<Image.Priv>().Header.Stamp.Sec = (int)(now / 1_000_000_000);
                nativeMsg.AsRef<Image.Priv>().Header.Stamp.Nanosec = (uint)(now % 1_000_000_000);

                pub.Publish(nativeMsg);
                await timer.WaitForNextTickAsync();
            }

            await Task.Delay(100);
        }

        var results = (await task).Select(x => x.TotalMilliseconds).ToArray();
        Assert.Equal(100, results.Length);

        var mean = results.Average();

        _output.WriteLine("Max: " + results.Max());
        _output.WriteLine("Min: " + results.Min());
        _output.WriteLine("Avg: " + mean);

        var histogram = results.GroupBy(v => (int)Math.Round(v) / 10).OrderBy(x => x.Key).ToDictionary(x => x.Key, x => x.Count());
        _output.WriteLine("Histogram: ");
        foreach (var item in histogram)
        {
            _output.WriteLine($"~ {item.Key * 10 + 10} ms: {item.Value}");
        }

        Assert.True(mean < 100);

        static async Task<List<TimeSpan>> CountAverageLatency(RclClock clock, IAsyncEnumerable<RosMessageBuffer> messages)
        {
            var timings = new List<TimeSpan>();
            await foreach (var message in messages)
            {
                var received = clock.Elapsed;
                var stamp = message.AsRef<Image.Priv>().Header.Stamp;
                var sent = TimeSpan.FromSeconds(stamp.Sec + stamp.Nanosec / 1_000_000_000.0);

                timings.Add(received - sent);
            }

            return timings;
        }
    }
}