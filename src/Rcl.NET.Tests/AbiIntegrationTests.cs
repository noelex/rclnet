using Rcl.Actions;
using Rosidl.Messages.Ros2csAbiTest;
using Rosidl.Runtime;
using System.Text;

namespace Rcl.NET.Tests;

public class AbiIntegrationTests
{
    private const int Timeout = 10_000;

    [SkippableFact]
    public void NativeMessageAndSequenceEqualityUsesRosidlSymbols()
    {
        RequireTestInterfaces();

        if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
        {
            using var message = new Scalar.Priv();
            using var sequence = new Scalar.PrivSequence(1);
            Assert.True(message.Equals(message));
            Assert.True(sequence.Equals(sequence));
        }
        else
        {
            using var message = new Scalar.PrivV2();
            using var sequence = new Scalar.PrivSequenceV2(1);
            Assert.True(message.Equals(message));
            Assert.True(sequence.Equals(sequence));
        }
    }

    [SkippableFact]
    public async Task PortableSequenceMessagesRoundTrip()
    {
        RequireTestInterfaces();

        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var primitive = await RoundTripAsync(
            node,
            new PrimitiveSequence([1, 2, 3], trailingBool: true, trailingValue: 42));
        Assert.Equal(new byte[] { 1, 2, 3 }, primitive.Data);
        Assert.True(primitive.TrailingBool);
        Assert.Equal(42u, primitive.TrailingValue);

        var bounded = await RoundTripAsync(node, new BoundedPrimitiveSequence([4, 5, 6, 7]));
        Assert.Equal(new ushort[] { 4, 5, 6, 7 }, bounded.Values);

        var complex = await RoundTripAsync(
            node,
            new ComplexSequence([
                new(new PrimitiveSequence([8], trailingBool: false, trailingValue: 9)),
                new(new PrimitiveSequence([10, 11], trailingBool: true, trailingValue: 12)),
            ]));
        Assert.Equal(new byte[] { 8 }, complex.Values[0].Value.Data);
        Assert.Equal(new byte[] { 10, 11 }, complex.Values[1].Value.Data);
        Assert.Equal(12u, complex.Values[1].Value.TrailingValue);

        var strings = await RoundTripAsync(node, new StringSequence(["alpha", "测试", "omega"]));
        Assert.Equal(new[] { "alpha", "测试", "omega" }, strings.Values);
    }

    [SkippableFact]
    public async Task PortableNativeBufferSubscriptionRoundTrip()
    {
        RequireTestInterfaces();

        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());

        var topic = NameGenerator.GenerateTopicName();
        using var publisher = node.CreatePublisher<PrimitiveSequence>(topic);
        using var subscription = node.CreateNativeSubscription<PrimitiveSequence>(topic);

        var readTask = ReadOneAsync(subscription.ReadAllAsync());
        await WaitForSubscribersAsync(publisher);
        publisher.Publish(new PrimitiveSequence([13, 14], trailingBool: true, trailingValue: 15));

        using var buffer = await readTask.WaitAsync(TimeSpan.FromMilliseconds(Timeout));
        var message = (PrimitiveSequence)PrimitiveSequence.CreateFrom(buffer.Data, Encoding.UTF8);
        Assert.Equal(new byte[] { 13, 14 }, message.Data);
        Assert.True(message.TrailingBool);
        Assert.Equal(15u, message.TrailingValue);
    }

    [SkippableFact]
    public async Task PortableSequenceServiceRoundTrip()
    {
        RequireTestInterfaces();

        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        var serviceName = NameGenerator.GenerateServiceName();

        using var server = node.CreateService<
            SequenceServiceService,
            SequenceServiceServiceRequest,
            SequenceServiceServiceResponse>(
                serviceName,
                (request, _) => new(request.Values.Select(x => $"value-{x}").ToArray()));
        using var client = node.CreateClient<
            SequenceServiceService,
            SequenceServiceServiceRequest,
            SequenceServiceServiceResponse>(serviceName);

        Assert.True(await client.TryWaitForServerAsync(Timeout));
        var response = await client.InvokeAsync(new SequenceServiceServiceRequest([16, 17]), Timeout);
        Assert.Equal(new[] { "value-16", "value-17" }, response.Values);
    }

    [SkippableFact]
    public async Task PortableSequenceActionRoundTrip()
    {
        RequireTestInterfaces();

        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        var actionName = NameGenerator.GenerateActionName();
        var handler = new SequenceHandler();

        using var server = node.CreateActionServer<
            SequenceAction,
            SequenceActionGoal,
            SequenceActionResult,
            SequenceActionFeedback>(actionName, handler);
        using var client = node.CreateActionClient<
            SequenceAction,
            SequenceActionGoal,
            SequenceActionResult,
            SequenceActionFeedback>(actionName);

        await client.WaitForServerAsync(Timeout);
        using var goal = await client.SendGoalAsync(new SequenceActionGoal([18, 19]), Timeout);
        var feedbackTask = ReadOneAsync(goal.ReadFeedbacksAsync());
        handler.FeedbackReaderReady.TrySetResult();
        var feedback = await feedbackTask.WaitAsync(TimeSpan.FromMilliseconds(Timeout));
        handler.FeedbackReceived.TrySetResult();
        var result = await goal.GetResultWithStatusAsync(Timeout);

        Assert.True(result.IsSuccessful);
        Assert.NotNull(result.Result);
        Assert.Equal(new[] { "value-18", "value-19" }, result.Result.ResultValues);
        Assert.Equal(new byte[] { 18, 19 }, feedback.FeedbackValues[0].Value.Data);
    }

    private static void RequireTestInterfaces()
    {
        try
        {
            _ = PrimitiveSequence.GetTypeSupportHandle();
        }
        catch (DllNotFoundException)
        {
            Skip.If(true, "The native ros2cs_abi_test_msgs package is not installed.");
        }
    }

    private static async Task<T> RoundTripAsync<T>(IRclNode node, T message)
        where T : IMessage
    {
        var topic = NameGenerator.GenerateTopicName();
        using var publisher = node.CreatePublisher<T>(topic);
        using var subscription = node.CreateSubscription<T>(topic);

        var readTask = ReadOneAsync(subscription.ReadAllAsync());
        await WaitForSubscribersAsync(publisher);
        publisher.Publish(message);
        return await readTask.WaitAsync(TimeSpan.FromMilliseconds(Timeout));
    }

    private static async Task<T> ReadOneAsync<T>(IAsyncEnumerable<T> messages)
    {
        await foreach (var message in messages)
        {
            return message;
        }

        throw new InvalidOperationException("The subscription completed without receiving a message.");
    }

    private static async Task WaitForSubscribersAsync(IRclPublisher publisher)
    {
        for (var retry = 0; publisher.Subscribers == 0 && retry < 500; retry++)
        {
            await Task.Delay(10);
        }

        Assert.True(publisher.Subscribers > 0, "The publisher did not match a subscription.");
    }

    private sealed class SequenceHandler :
        ActionGoalHandler<SequenceActionGoal, SequenceActionResult, SequenceActionFeedback>
    {
        public TaskCompletionSource FeedbackReaderReady { get; } =
            new(TaskCreationOptions.RunContinuationsAsynchronously);
        public TaskCompletionSource FeedbackReceived { get; } =
            new(TaskCreationOptions.RunContinuationsAsynchronously);

        public override bool CanAccept(Guid id, SequenceActionGoal goal)
            => goal.GoalValues.SequenceEqual(new byte[] { 18, 19 });

        public override async Task<SequenceActionResult> ExecuteAsync(
            IActionGoalController<SequenceActionFeedback> controller,
            SequenceActionGoal goal,
            CancellationToken cancellationToken)
        {
            await FeedbackReaderReady.Task.WaitAsync(TimeSpan.FromMilliseconds(Timeout), cancellationToken);
            controller.Report(new SequenceActionFeedback([
                new(new PrimitiveSequence(goal.GoalValues, trailingBool: true, trailingValue: 20)),
            ]));
            // Goal completion can close the feedback stream before its last message is processed.
            await FeedbackReceived.Task.WaitAsync(TimeSpan.FromMilliseconds(Timeout), cancellationToken);
            return new(goal.GoalValues.Select(x => $"value-{x}").ToArray());
        }
    }
}
