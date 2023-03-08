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
    /// Reads all messages from the subscription.
    /// </summary>
    /// <remarks>
    /// Concurrent calls to the same <see cref="IRclNativeSubscription"/> instance are allowed.
    /// But note that each message will be delivered exactly once, regardless of how many ongoing calls
    /// to this method.
    /// <para>
    /// Additionally, ownership of the <see cref="RosMessageBuffer"/>s returned by this method are
    /// transferred to the caller. It's the caller's responsibility to dispose the buffer when no more needed.
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