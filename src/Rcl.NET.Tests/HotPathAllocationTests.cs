using Rcl.Actions;
using Rcl.Actions.Client;
using Rosidl.Messages.Builtin;
using Rosidl.Messages.Rcl;
using Rosidl.Messages.Tf2;
using System.Diagnostics;
using System.Text;
using Xunit.Abstractions;

namespace Rcl.NET.Tests;

public class HotPathAllocationTests(ITestOutputHelper output)
{
    private readonly AllocationMeter _meter = new(output);

    [Fact]
    [Trait("Category", "PerformanceBaseline")]
    public async Task HotPathAllocationBaselines()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var publisher = node.CreatePublisher<Time>(NameGenerator.GenerateTopicName());
        using var buffer = publisher.CreateBuffer();
        var message = new Time(sec: 1, nanosec: 2);
        _meter.Measure("introspection-buffer-create-release", 10_000, () =>
        {
            using var owned = publisher.CreateBuffer();
        }, zeroAllocation: true);
        _meter.Measure("typed-publish", 10_000, () => publisher.Publish(message), zeroAllocation: true);
        await _meter.MeasureAsync("native-publish-async", 1000, () => publisher.PublishAsync(buffer));
        await _meter.MeasureAsync("typed-publish-async", 1000, () => publisher.PublishAsync(message));

        using var guard = context.CreateGuardCondition();
        await _meter.MeasureAsync("wait-one-async", 1000, async () =>
        {
            var waiting = guard.WaitOneAsync();
            guard.Trigger();
            await waiting;
        });
    }

    [Fact]
    [Trait("Category", "PerformanceBaseline")]
    public async Task TimeProviderTimerAllocationBaseline()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var provider = new RclTimeProvider(context, RclClock.SteadyClock);
        using var tick = new SemaphoreSlim(0);
        using var cancellation = new CancellationTokenSource(TimeSpan.FromSeconds(60));
        using var timer = provider.CreateTimer(static state => ((SemaphoreSlim)state!).Release(),
            tick, Timeout.InfiniteTimeSpan, Timeout.InfiniteTimeSpan);

        // Includes rearming, ThreadPool dispatch and the async acknowledgement of each callback.
        await _meter.MeasureAsync("time-provider-timer-roundtrip", 1000, async () =>
        {
            Assert.True(timer.Change(TimeSpan.Zero, Timeout.InfiniteTimeSpan));
            await tick.WaitAsync(cancellation.Token);
        });
        await timer.DisposeAsync();
    }

    [Fact]
    [Trait("Category", "PerformanceBaseline")]
    public async Task SubscriptionAllocationBaselines()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var publisher = node.CreatePublisher<Time>(NameGenerator.GenerateTopicName());
        using var buffer = publisher.CreateBuffer();
        using var cancellation = new CancellationTokenSource(TimeSpan.FromSeconds(60));

        using (var subscription = node.CreateSubscription<Time>(publisher.Name))
        {
            await WaitForSubscriberAsync(publisher);
            await using var reader = subscription.ReadAllAsync(cancellation.Token).GetAsyncEnumerator();
            await _meter.MeasureAsync("typed-subscription-roundtrip", 1000, ReceiveAsync);
            var observer = new CountingObserver<Time>();
            using var registration = subscription.Subscribe(observer);
            await _meter.MeasureAsync("typed-subscription-one-observer-roundtrip", 1000, ReceiveAsync);
            await context.Yield();
            Assert.Equal(1000 * (AllocationMeter.SampleCount + 1), observer.Count);

            async ValueTask ReceiveAsync()
            {
                var received = reader.MoveNextAsync();
                publisher.Publish(buffer);
                Assert.True(await received);
            }
        }

        // The non-generic overload exercises introspection buffer creation.
        using var native = node.CreateNativeSubscription(publisher.Name, Time.GetTypeSupportHandle());
        await WaitForSubscriberAsync(publisher);
        await using var nativeReader = native.ReadAllAsync(cancellation.Token).GetAsyncEnumerator();
        await _meter.MeasureAsync("native-introspection-subscription-roundtrip", 1000, async () =>
        {
            var received = nativeReader.MoveNextAsync();
            publisher.Publish(buffer);
            Assert.True(await received);
            nativeReader.Current.Dispose();
        });
    }

    [Fact]
    [Trait("Category", "PerformanceBaseline")]
    public async Task ActionFeedbackAllocationBaselines()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var client = node.CreateActionClient<LookupTransformAction, LookupTransformActionGoal,
            LookupTransformActionResult, LookupTransformActionFeedback>(NameGenerator.GenerateActionName());
        var goal = new ActionGoalContext<LookupTransformActionResult, LookupTransformActionFeedback>(
            Guid.NewGuid(), (IActionClientImpl)client, Encoding.UTF8);
        var observer = new CountingObserver<LookupTransformActionFeedback>();

        // Feed the normal receive path directly to isolate it from DDS and async scheduling allocations.
        _meter.Measure("action-feedback-no-observer", 1000, Dispatch);
        using var first = goal.Subscribe(observer);
        _meter.Measure("action-feedback-one-observer", 1000, Dispatch);
        using var second = goal.Subscribe(observer);
        _meter.Measure("action-feedback-two-observers", 1000, Dispatch);
        Assert.Equal(3000 * (AllocationMeter.SampleCount + 1), observer.Count);
        goal.OnStatusChanged(ActionGoalStatus.Succeeded);

        void Dispatch()
        {
            goal.OnFeedbackReceived(RosMessageBuffer.Create<LookupTransformActionFeedback>());
        }
    }

    [Fact]
    [Trait("Category", "PerformanceBaseline")]
    public async Task ServiceAllocationBaselines()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        var name = NameGenerator.GenerateServiceName();
        using var server = node.CreateNativeService<ListParametersService>(name, static (request, response, state) =>
        {
            // The initialized empty response is sufficient for this allocation baseline.
        });
        using var client = node.CreateClient<ListParametersService, ListParametersServiceRequest, ListParametersServiceResponse>(name);
        Assert.True(await client.TryWaitForServerAsync(10_000));
        using var cancellation = new CancellationTokenSource(TimeSpan.FromSeconds(60));
        using var requestBuffer = RosMessageBuffer.Create<ListParametersServiceRequest>();
        var request = new ListParametersServiceRequest();
        await _meter.MeasureAsync("native-service-request-roundtrip", 500, async () =>
        {
            using var response = await client.InvokeAsync(requestBuffer, 10_000, cancellation.Token);
        });
        await _meter.MeasureAsync("client-request-infinite-timeout-roundtrip", 500, async () =>
        {
            await client.InvokeAsync(request, Timeout.Infinite, cancellation.Token);
        });
        await _meter.MeasureAsync("client-request-finite-timeout-roundtrip", 500, async () =>
        {
            await client.InvokeAsync(request, 10_000, cancellation.Token);
        });
    }

    [Fact]
    [Trait("Category", "PerformanceBaseline")]
    public async Task GraphAllocationBaselines()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var publisher = node.CreatePublisher<Time>(NameGenerator.GenerateTopicName());
        // A separate graph on the event loop lets us measure Build without the automatic builder racing it.
        await context.Yield();
        var graph = new Rcl.Graph.RosGraph((Rcl.Internal.RclNodeImpl)node, name => name.Name == node.Name);
        graph.Build();
        var graphNode = Assert.Single(graph.Nodes);
        int initialPublishers = graphNode.Publishers.Count;
        object? snapshot = null;
        _meter.Measure("graph-nodes-read", 10_000, () => snapshot = graph.Nodes, zeroAllocation: true);
        _meter.Measure("node-publishers-read", 10_000, () => snapshot = graphNode.Publishers, zeroAllocation: true);
        // Discovery is asynchronous, so late graph changes can still allocate during this baseline.
        _meter.Measure("graph-refresh-no-change", 1000, graph.Build);

        // Endpoint creation/removal is outside the measured interval; only the refresh is counted.
        for (int sample = -1; sample < AllocationMeter.SampleCount; sample++)
        {
            long bytes = 0;
            long ticks = 0;

            for (int i = 0; i < 100; i++)
            {
                using (var endpoint = node.CreatePublisher<Time>(publisher.Name))
                {
                    long allocated = GC.GetAllocatedBytesForCurrentThread();
                    long start = Stopwatch.GetTimestamp();
                    graph.Build();
                    ticks += Stopwatch.GetTimestamp() - start;
                    bytes += GC.GetAllocatedBytesForCurrentThread() - allocated;
                    Assert.Equal(initialPublishers + 1, graphNode.Publishers.Count);
                }

                // Drain deferred endpoint cleanup before preparing the next sample.
                await context.Yield();
                graph.Build();
                Assert.Equal(initialPublishers, graphNode.Publishers.Count);
            }

            if (sample >= 0)
            {
                _meter.Report("graph-refresh-one-endpoint-change", sample, ticks, 100, bytes);
            }
        }

        GC.KeepAlive(snapshot);
    }

    private static async Task WaitForSubscriberAsync(IRclPublisher publisher)
    {
        using var cancellation = new CancellationTokenSource(TimeSpan.FromSeconds(10));

        while (publisher.Subscribers == 0)
        {
            await Task.Delay(10, cancellation.Token);
        }
    }

    private sealed class CountingObserver<T> : IObserver<T>
    {
        public int Count { get; private set; }

        public void OnNext(T value)
        {
            Count++;
        }

        public void OnCompleted()
        {
        }

        public void OnError(Exception error)
        {
            throw error;
        }
    }
}
