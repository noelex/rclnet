using Rcl.Qos;
using Rcl.Runtime;
using Rosidl.Runtime;

namespace Rcl;

/// <summary>
/// Represents a subscription to a ROS topic.
/// </summary>
public interface IRclSubscription : IRclObject
{
    /// <summary>
    /// Actual QoS selected by the underlying implementation.
    /// </summary>
    QosProfile ActualQos { get; }

    /// <summary>
    /// Determines wheter current <see cref="IRclSubscription"/> is valid.
    /// </summary>
    bool IsValid { get; }

    /// <summary>
    /// Gets the count of publishers publishing to this topic.
    /// </summary>
    int Publishers { get; }

    /// <summary>
    /// Name of the subscribed topic.
    /// </summary>
    string Name { get; }

    /// <summary>
    /// Gets the network flow endpoints of current subscription.
    /// </summary>
    /// <remarks>
    /// Supported by: >= humble
    /// </remarks>
    [SupportedSinceDistribution(RosEnvironment.Humble)]
    NetworkFlowEndpoint[] Endpoints { get; }
}

/// <summary>
/// Represents a subscription to a ROS topic which offers messages of type <typeparamref name="T"/>.
/// </summary>
/// <typeparam name="T">Type of the message.</typeparam>
public interface IRclSubscription<T> : IRclSubscription, IObservable<T>
    where T : IMessage
{
    /// <summary>
    /// Reads all messages from the subscription.
    /// </summary>
    /// <remarks>
    /// Concurrent calls to the same <see cref="IRclSubscription{T}"/> instance are allowed.
    /// But note that each message will be delivered exactly once, regardless of how many ongoing calls
    /// to this method.
    /// </remarks>
    /// <param name="cancellationToken"></param>
    /// <returns>
    /// An <see cref="IAsyncEnumerable{RosMessageBuffer}"/> for receiving the messages asynchronously.
    /// <para>
    /// The asynchronous enumeration will complete when the <see cref="IRclSubscription{T}"/> instance
    /// is disposed.
    /// </para>
    /// </returns>
    IAsyncEnumerable<T> ReadAllAsync(CancellationToken cancellationToken = default);
}

/// <summary>
/// An <see cref="IRclSubscription"/> that allows receiving messages using native message buffers.
/// </summary>
public interface IRclNativeSubscription : IRclSubscription
{
    /// <summary>
    /// Gets whether the middleware and RCL configuration allow this subscription to loan messages.
    /// </summary>
    /// <remarks>
    /// Receiving loaned buffers also requires <see cref="SubscriptionOptions.UseLoanedMessages"/>.
    /// Loan support does not guarantee zero-copy transport.
    /// </remarks>
    bool CanLoanMessages { get; }

    /// <summary>
    /// Reads all messages from the subscription.
    /// </summary>
    /// <remarks>
    /// Concurrent calls to the same <see cref="IRclNativeSubscription"/> instance are allowed.
    /// But note that each message will be delivered exactly once, regardless of how many ongoing calls
    /// to this method.
    /// <para>
    /// Each yielded buffer transfers responsibility for releasing it to the consumer, who must dispose it
    /// exactly once when no longer needed. Ending enumeration or disposing the subscription does not release
    /// buffers already yielded to a consumer.
    /// </para>
    /// <para>
    /// Without <see cref="SubscriptionOptions.UseLoanedMessages"/>, the consumer owns the allocated message memory.
    /// When it is enabled, the middleware retains ownership of the memory and the consumer is responsible for returning the loan.
    /// Treat them as read-only and dispose each exactly once before disposing this subscription or its context.
    /// Copies share the same loan and must not be accessed or disposed after the loan is returned.
    /// Holding many loans can exhaust middleware resources.
    /// </para>
    /// </remarks>
    /// <param name="cancellationToken"></param>
    /// <returns>
    /// An <see cref="IAsyncEnumerable{RosMessageBuffer}"/> for receiving the messages asynchronously.
    /// <para>
    /// The asynchronous enumeration will complete when the <see cref="IRclNativeSubscription"/> instance
    /// is disposed.
    /// </para>
    /// </returns>
    IAsyncEnumerable<RosMessageBuffer> ReadAllAsync(CancellationToken cancellationToken = default);
}
