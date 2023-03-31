using Rcl.Qos;
using Rcl.Runtime;
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
    /// Gets the globally unique identifier (GID) of the publisher.
    /// </summary>
    GraphId Gid { get; }

    /// <summary>
    /// Gets the network flow endpoints of current publisher.
    /// </summary>
    /// <remarks>
    /// Supported by: >= humble
    /// </remarks>
    [SupportedSinceDistribution(RosEnvironment.Humble)]
    NetworkFlowEndpoint[] Endpoints { get; }

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
    /// If the QoS setting may cause blocking in current RMW implementation, use <see cref="PublishAsync(RosMessageBuffer)"/> instead.
    /// See <a href="https://github.com/ros2/ros2/issues/255"/> for more information.
    /// </para>
    /// </remarks>
    /// <param name="message">An <see cref="RosMessageBuffer"/> containing the message to be published.</param>
    void Publish(RosMessageBuffer message);

    /// <summary>
    /// Publish the message in a background thread, and asynchronously wait for the operation to complete.
    /// </summary>
    /// <param name="message">An <see cref="RosMessageBuffer"/> containing the message to be published.</param>
    /// <returns>A <see cref="ValueTask"/> object represent the asynchronous wait.</returns>
    ///  <remarks>
    /// The type of the message contained in the <see cref="RosMessageBuffer"/> is not checked by the runtime.
    /// It's the caller's responsibility to ensure the <see cref="RosMessageBuffer"/> contains a message
    /// with the same type when creating the publisher. Publishing a different type of message is undefined behavior.
    /// <para>
    /// This method does not take the ownership of the <see cref="RosMessageBuffer"/>.
    /// </para>
    /// <para>
    /// Also, the <see cref="RosMessageBuffer"/> MUST NOT be disposed before the returned <see cref="ValueTask"/> completes.
    /// </para>
    /// <para>
    /// This is a helper method which simply calls <see cref="Publish(RosMessageBuffer)"/> in a background thread.
    /// Calling this method will incur asynchronous scheduling overhead,
    /// which is slightly imperformant compared to the synchronous counterpart.
    /// You should always use <see cref="Publish(RosMessageBuffer)"/>
    /// instead, if the QoS setting does not cause blocking in current RMW implementation.
    /// </para>
    /// <para>
    /// See <a href="https://github.com/ros2/ros2/issues/255"/> for more information.
    /// </para>
    /// </remarks>
    ValueTask PublishAsync(RosMessageBuffer message);

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

    /// <summary>
    /// Manually assert that this publisher is alive (for publishers created with <see cref="QosProfile.Liveliness"/> set to <see cref="LivelinessPolicy.ManualByTopic"/>).
    /// </summary>
    /// <remarks>
    /// If the <see cref="QosProfile.Liveliness"/> is set to <see cref="LivelinessPolicy.ManualByTopic"/>, the creator of
    /// this publisher may manually call this method at some point in time to signal to the rest
    /// of the system that this topic is still alive.
    /// <para>
    /// This method must be called at least as often as the duration specified by <see cref="QosProfile.LivelinessLeaseDuration"/>.
    /// </para>
    /// </remarks>
    void AssertLiveliness();
}

/// <summary>
/// An <see cref="IRclPublisher"/> which publishes strongly typed messages.
/// </summary>
/// <typeparam name="T">Type of the message.</typeparam>
public interface IRclPublisher<T> : IRclPublisher
    where T : IMessage
{
    /// <summary>
    /// Publish a message.
    /// </summary>
    /// <remarks>
    /// If the QoS setting may cause blocking in current RMW implementation, use <see cref="PublishAsync(T)"/> instead.
    /// See <a href="https://github.com/ros2/ros2/issues/255"/> for more information.
    /// </remarks>
    /// <param name="message">The message to be published.</param>
    void Publish(T message);

    /// <summary>
    /// Publish the message in a background thread, and asynchronously wait for the operation to complete.
    /// </summary>
    /// <param name="message">The message to be published.</param>
    /// <returns>A <see cref="ValueTask"/> object represent the asynchronous wait.</returns>
    /// <remarks>
    /// This is a helper method which simply calls <see cref="Publish(T)"/> in a background thread.
    /// Calling this method will incur asynchronous scheduling overhead,
    /// which is slightly imperformant compared to the synchronous counterpart.
    /// You should always use <see cref="Publish(T)"/>
    /// instead, if the QoS setting does not cause blocking in current RMW implementation.
    /// <para>
    /// See <a href="https://github.com/ros2/ros2/issues/255"/> for more information.
    /// </para>
    /// </remarks>
    ValueTask PublishAsync(T message);
}