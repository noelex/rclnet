using Rcl.Internal;
using Rcl.Interop;
using Rcl.SafeHandles;
using Rosidl.Messages.Builtin;
using Rosidl.Runtime;
using System.Threading.Channels;

namespace Rcl.NET.Tests;

public class LoanedMessageTests
{
    [Fact]
    public async Task PublisherLoansCanBeReturnedWithoutManagedAllocations()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var publisher = node.CreatePublisher<Time>(NameGenerator.GenerateTopicName());

        if (!publisher.CanLoanMessages)
        {
            Assert.Throws<NotSupportedException>(() => publisher.BorrowLoanedMessage());
            return;
        }

        for (var i = 0; i < 64; i++)
        {
            using var message = publisher.BorrowLoanedMessage();
            WriteTime(message, i);
        }

        var allocated = GC.GetAllocatedBytesForCurrentThread();

        for (var i = 0; i < 1024; i++)
        {
            using var message = publisher.BorrowLoanedMessage();
            WriteTime(message, i);
        }

        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - allocated);
    }

    [SkippableFact]
    public async Task PublisherLoanTransfersOwnershipAndReachesManagedSubscription()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var publisher = node.CreatePublisher<Time>(NameGenerator.GenerateTopicName());
        Skip.IfNot(publisher.CanLoanMessages, "Publisher loans are unavailable or disabled.");

        using var subscription = node.CreateSubscription<Time>(publisher.Name);
        using var cancellation = new CancellationTokenSource(TimeSpan.FromSeconds(10));
        await using var reader = subscription.ReadAllAsync(cancellation.Token).GetAsyncEnumerator();
        await WaitForSubscriberAsync(publisher);

        var received = reader.MoveNextAsync();
        var message = publisher.BorrowLoanedMessage();

        try
        {
            WriteTime(message, 42);
            publisher.PublishLoaned(ref message);
            Assert.True(message.IsEmpty);
        }
        finally
        {
            if (!message.IsEmpty)
            {
                message.Dispose();
            }
        }

        Assert.True(await received);
        Assert.Equal(42, reader.Current.Sec);
        Assert.Equal(123u, reader.Current.Nanosec);
    }

    [SkippableTheory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public async Task NativeSubscriptionsReceiveAndReturnLoans(bool introspection, bool loanedPublish)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var publisher = node.CreatePublisher<Time>(NameGenerator.GenerateTopicName());
        Skip.IfNot(SubscriptionCanLoanMessages(node, publisher.Name), "Subscription loans are unavailable.");

        if (loanedPublish)
        {
            Skip.IfNot(publisher.CanLoanMessages, "Publisher loans are unavailable or disabled.");
        }

        var options = new SubscriptionOptions(useLoanedMessages: true);
        using var subscription = introspection
            ? node.CreateNativeSubscription(publisher.Name, Time.GetTypeSupportHandle(), options)
            : node.CreateNativeSubscription<Time>(publisher.Name, options);
        Assert.True(subscription.CanLoanMessages);

        using var cancellation = new CancellationTokenSource(TimeSpan.FromSeconds(10));
        await using var reader = subscription.ReadAllAsync(cancellation.Token).GetAsyncEnumerator();
        await WaitForSubscriberAsync(publisher);

        for (var i = 0; i < 16; i++)
        {
            var received = reader.MoveNextAsync();

            if (loanedPublish)
            {
                var message = publisher.BorrowLoanedMessage();

                try
                {
                    WriteTime(message, i);
                    publisher.PublishLoaned(ref message);
                }
                finally
                {
                    if (!message.IsEmpty)
                    {
                        message.Dispose();
                    }
                }
            }
            else
            {
                publisher.Publish(new Time(sec: i, nanosec: 123));
            }

            Assert.True(await received);
            using var buffer = reader.Current;
            Assert.Equal((i, 123u), ReadTime(buffer));
        }
    }

    [SkippableTheory]
    [InlineData(BoundedChannelFullMode.DropOldest)]
    [InlineData(BoundedChannelFullMode.DropNewest)]
    [InlineData(BoundedChannelFullMode.DropWrite)]
    public async Task NativeSubscriptionsReturnDroppedAndQueuedLoans(BoundedChannelFullMode fullMode)
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var publisher = node.CreatePublisher<Time>(NameGenerator.GenerateTopicName());
        Skip.IfNot(SubscriptionCanLoanMessages(node, publisher.Name), "Subscription loans are unavailable.");

        using var subscription = node.CreateNativeSubscription<Time>(publisher.Name,
            new(queueSize: 2, fullMode: fullMode, useLoanedMessages: true));
        using var cancellation = new CancellationTokenSource(TimeSpan.FromSeconds(10));
        await WaitForSubscriberAsync(publisher);

        for (var i = 0; i < 32; i++)
        {
            // Wait for take and queue insertion without consuming the queued loan.
            var taken = ((IRclWaitObject)subscription).WaitOneAsync(cancellation.Token);
            publisher.Publish(new Time(sec: i));
            await taken;
        }

        subscription.Dispose();
        await context.DisposeAsync();
    }

    [Fact]
    public async Task UnsupportedNativeSubscriptionsRejectLoans()
    {
        await using var context = new RclContext(TestConfig.DefaultContextArguments);
        using var node = context.CreateNode(NameGenerator.GenerateNodeName());
        using var publisher = node.CreatePublisher<Time>(NameGenerator.GenerateTopicName());
        Assert.False(SubscriptionOptions.Default.UseLoanedMessages);

        if (!SubscriptionCanLoanMessages(node, publisher.Name))
        {
            Assert.Throws<NotSupportedException>(() => node.CreateNativeSubscription<Time>(publisher.Name,
                new(useLoanedMessages: true)));
            Assert.Throws<NotSupportedException>(() => node.CreateNativeSubscription(publisher.Name,
                Time.GetTypeSupportHandle(), new(useLoanedMessages: true)));
        }
    }

    private static unsafe bool SubscriptionCanLoanMessages(IRclNode node, string topic)
    {
        using var handle = new SafeSubscriptionHandle(((RclNodeImpl)node).Handle,
            Time.GetTypeSupportHandle(), topic, new(useLoanedMessages: true));
        using var lease = handle.Acquire();
        return RclCommon.rcl_subscription_can_loan_messages(lease.Object);
    }

    private static void WriteTime(RosMessageBuffer buffer, int sec)
    {
        if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
        {
            ref var time = ref buffer.AsRef<Time.Priv>();
            time.Sec = sec;
            time.Nanosec = 123;
        }
        else
        {
            ref var time = ref buffer.AsRef<Time.PrivV2>();
            time.Sec = sec;
            time.Nanosec = 123;
        }
    }

    private static (int Sec, uint Nanosec) ReadTime(RosMessageBuffer buffer)
    {
        if (RosidlRuntime.NativeAbi == RosidlNativeAbi.V1)
        {
            ref var time = ref buffer.AsRef<Time.Priv>();
            return (time.Sec, time.Nanosec);
        }

        ref var timeV2 = ref buffer.AsRef<Time.PrivV2>();
        return (timeV2.Sec, timeV2.Nanosec);
    }

    private static async Task WaitForSubscriberAsync(IRclPublisher publisher)
    {
        for (var retry = 0; publisher.Subscribers == 0 && retry < 500; retry++)
        {
            await Task.Delay(10);
        }

        Assert.True(publisher.Subscribers > 0);
    }
}
