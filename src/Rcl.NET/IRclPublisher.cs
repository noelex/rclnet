using Rcl.Qos;
using Rosidl.Runtime;

namespace Rcl;

/// <summary>
/// Represents a publisher for publishing messages to an ROS topic.
/// </summary>
public interface IRclPublisher : IRclObject
{
    /// <summary>
    /// Actual QoS selected by the underlying implementation.
    /// </summary>
    QosProfile ActualQos { get; }

    /// <summary>
    /// Determines wheter current <see cref="IRclPublisher"/> is valid.
    /// </summary>
    bool IsValid { get; }

    /// <summary>
    /// Gets the count of subscribers subscribing to this topic.
    /// </summary>
    int Subscribers { get; }

    /// <summary>
    /// Name of the publishing topic.
    /// </summary>
    string Name { get; }

    /// <summary>
    /// Publish a message.
    /// </summary>
    /// <remarks>
    /// The type of the message contained in the <see cref="RosMessageBuffer"/> is not checked by the runtime.
    /// It's the caller's responsibility to ensure the <see cref="RosMessageBuffer"/> contains a message
    /// with the same type when creating the publisher. Publishing a different type of message is undefined behavior.
    /// <para>
    /// This method does not take the ownership of the <see cref="RosMessageBuffer"/>.
    /// </para>
    /// <para>
    /// This method may block when <see cref="QosProfile.Reliability"/> is set to <see cref="ReliabilityPolicy.Reliable"/> and
    /// <see cref="QosProfile.Depth"/> is set to a relatively small value.
    /// See <a href="https://github.com/ros2/ros2/issues/255"/> for more information.
    /// </para>
    /// </remarks>
    /// <param name="message">An <see cref="RosMessageBuffer"/> containing the message to be published.</param>
    void Publish(RosMessageBuffer message);

    /// <summary>
    /// Create an <see cref="RosMessageBuffer"/> containing the message with the same type when creating the publisher.
    /// </summary>
    /// <returns>
    /// A newly allocated <see cref="RosMessageBuffer"/>.
    /// <para>
    /// The ownership of the returned buffer is transferred to the caller.
    /// </para>
    /// </returns>
    RosMessageBuffer CreateBuffer();
}

/// <summary>
/// An <see cref="IRclPublisher"/> which publishes strongly typed messages.
/// </summary>
/// <typeparam name="T"></typeparam>
public interface IRclPublisher<T> : IRclPublisher
    where T : IMessage
{
    /// <summary>
    /// Publish a message.
    /// </summary>
    /// <remarks>
    /// This method may block when <see cref="QosProfile.Reliability"/> is set to <see cref="ReliabilityPolicy.Reliable"/> and
    /// <see cref="QosProfile.Depth"/> is set to a relatively small value.
    /// See <a href="https://github.com/ros2/ros2/issues/255"/> for more information.
    /// </remarks>
    /// <param name="message">The message to be published.</param>
    void Publish(T message);
}