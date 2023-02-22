using Rcl.Actions;
using Rcl.Graph;
using Rcl.Internal.Services;
using Rcl.Logging;
using Rcl.Qos;
using Rosidl.Runtime;
using System.Text;
using System.Threading.Channels;

namespace Rcl;

/// <summary>
/// Represents an ROS node.
/// </summary>
public interface IRclNode : IRclObject
{
    /// <summary>
    /// Gets the <see cref="IRclContext"/> object which the node belongs.
    /// </summary>
    RclContext Context { get; }

    /// <summary>
    /// Gets the domain ID of the node.
    /// </summary>
    nuint DomaindId { get; }

    /// <summary>
    /// Gets the fully-qualified name of the node.
    /// </summary>
    string FullyQualifiedName { get; }

    /// <summary>
    /// Gets a <see cref="RosGraph"/> for querying and monitoring ROS graph.
    /// </summary>
    RosGraph Graph { get; }

    /// <summary>
    /// Gets the instance ID of the node.
    /// </summary>
    ulong InstanceId { get; }

    /// <summary>
    /// Checks whether current <see cref="IRclNode"/> is valid.
    /// </summary>
    bool IsValid { get; }

    /// <summary>
    /// Logger of the node.
    /// </summary>
    IRclLogger Logger { get; }

    /// <summary>
    /// Gets the name of the node.
    /// </summary>
    string Name { get; }

    /// <summary>
    /// Gets the namespace of the node.
    /// </summary>
    string Namespace { get; }

    /// <summary>
    /// Create an ROS subscription and receive messages using native message buffers.
    /// </summary>
    /// <typeparam name="T">Type of the message to receive.</typeparam>
    /// <param name="topicName">Name of the topic to subscribe.</param>
    /// <param name="qos"><see cref="QosProfile"/> to be used for the subscription. Defaults to <see cref="QosProfile.Default"/>.</param>
    /// <param name="queueSize">Size of the message delivery queue. This parameter is used for seting up the delivery queue on .NET side, and is irrelevant to <see cref="QosProfile.Depth"/>. </param>
    /// <param name="fullMode">Behavior to use when the delivery queue is full.</param>
    /// <returns>A <see cref="IRclNativeSubscription"/> to be used for receiving topic messages.</returns>
    IRclNativeSubscription CreateNativeSubscription<T>(
        string topicName,
        QosProfile? qos = null,
        int queueSize = 1,
        BoundedChannelFullMode fullMode = BoundedChannelFullMode.DropOldest) where T : IMessage;

    /// <summary>
    /// Create an ROS subscription and receive messages using native message buffers.
    /// </summary>
    /// <param name="topicName">Name of the topic to subscribe.</param>
    /// <param name="typeSupport">Type support handle of the message.</param>
    /// <param name="qos"><see cref="QosProfile"/> to be used for the subscription. Defaults to <see cref="QosProfile.Default"/>.</param>
    /// <param name="queueSize">Size of the message delivery queue. This parameter is used for setting up the delivery queue on .NET side, and is irrelevant to <see cref="QosProfile.Depth"/>. </param>
    /// <param name="fullMode">Behavior to use when the delivery queue is full.</param>
    /// <returns>A <see cref="IRclNativeSubscription"/> to be used for receiving topic messages.</returns>
    IRclNativeSubscription CreateNativeSubscription(
        string topicName,
        TypeSupportHandle typeSupport,
        QosProfile? qos = null,
        int queueSize = 1,
        BoundedChannelFullMode fullMode = BoundedChannelFullMode.DropOldest);

    /// <summary>
    /// Create an ROS subscription and receive messages of type <typeparamref name="T"/>.
    /// </summary>
    /// <typeparam name="T">Type of the message to receive.</typeparam>
    /// <param name="topicName">Name of the topic to subscribe.</param>
    /// <param name="qos"><see cref="QosProfile"/> to be used for the subscription. Defaults to <see cref="QosProfile.Default"/>.</param>
    /// <param name="queueSize">Size of the message delivery queue. This parameter is used for setting up the delivery queue on .NET side, and is irrelevant to <see cref="QosProfile.Depth"/>.</param>
    /// <param name="fullMode">Behavior to use when the delivery queue is full.</param>
    /// <param name="textEncoding">Specify the encoding of the string in the message. Defaults to <see cref="Encoding.UTF8"/>.</param>
    /// <returns>A <see cref="IRclNativeSubscription"/> to be used for receiving topic messages.</returns>
    IRclSubscription<T> CreateSubscription<T>(
        string topicName,
        QosProfile? qos = null,
        int queueSize = 1,
        BoundedChannelFullMode fullMode = BoundedChannelFullMode.DropOldest,
        Encoding? textEncoding = null) where T : IMessage;

    /// <summary>
    /// Create an ROS publisher to publish messages of type <typeparamref name="T"/>
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="topicName">Name of the topic.</param>
    /// <param name="qos"><see cref="QosProfile"/> to be used for the publisher. Defaults to <see cref="QosProfile.Default"/>.</param>
    /// <param name="textEncoding">Specify the encoding of the string in the message. Defaults to <see cref="Encoding.UTF8"/>.</param>
    /// <returns>A <see cref="IRclPublisher{T}"/> to be used for publishing topic messages.</returns>
    IRclPublisher<T> CreatePublisher<T>(
        string topicName,
        QosProfile? qos = null,
        Encoding? textEncoding = null) where T : IMessage;

    /// <summary>
    /// Create an ROS service server.
    /// </summary>
    /// <typeparam name="TService">Type of the service.</typeparam>
    /// <typeparam name="TRequest">Type of the service request.</typeparam>
    /// <typeparam name="TResponse">Type of the service response.</typeparam>
    /// <param name="serviceName"></param>
    /// <param name="handler"></param>
    /// <param name="qos"></param>
    /// <param name="textEncoding"></param>
    /// <returns></returns>
    IRclService CreateService<TService, TRequest, TResponse>(
        string serviceName,
        IServiceHandler<TRequest, TResponse> handler,
        QosProfile? qos = null,
        Encoding? textEncoding = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse;

    /// <summary>
    /// Create an ROS service server.
    /// </summary>
    /// <typeparam name="TService">Type of the service.</typeparam>
    /// <typeparam name="TRequest">Type of the service request.</typeparam>
    /// <typeparam name="TResponse">Type of the service response.</typeparam>
    /// <param name="serviceName"></param>
    /// <param name="handler"></param>
    /// <param name="qos"></param>
    /// <param name="textEncoding"></param>
    /// <param name="state"></param>
    /// <returns></returns>
    IRclService CreateService<TService, TRequest, TResponse>(
        string serviceName,
        Func<TRequest, object?, TResponse> handler,
        QosProfile? qos = null,
        Encoding? textEncoding = null,
        object? state = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse;

    /// <summary>
    /// Create an ROS service server which is capable of handling requests concurrently.
    /// </summary>
    /// <typeparam name="TService">Type of the service.</typeparam>
    /// <typeparam name="TRequest">Type of the service request.</typeparam>
    /// <typeparam name="TResponse">Type of the service response.</typeparam>
    /// <param name="serviceName"></param>
    /// <param name="handler"></param>
    /// <param name="qos"></param>
    /// <param name="textEncoding"></param>
    /// <returns></returns>
    IRclService CreateConcurrentService<TService, TRequest, TResponse>(
        string serviceName,
        IConcurrentServiceHandler<TRequest, TResponse> handler,
        QosProfile? qos = null,
        Encoding? textEncoding = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse;

    /// <summary>
    /// Create an ROS service server which is capable of handling requests concurrently.
    /// </summary>
    /// <typeparam name="TService">Type of the service.</typeparam>
    /// <typeparam name="TRequest">Type of the service request.</typeparam>
    /// <typeparam name="TResponse">Type of the service response.</typeparam>
    /// <param name="serviceName"></param>
    /// <param name="handler"></param>
    /// <param name="qos"></param>
    /// <param name="textEncoding"></param>
    /// <param name="state"></param>
    /// <returns></returns>
    IRclService CreateConcurrentService<TService, TRequest, TResponse>(
        string serviceName,
        Func<TRequest, object?, CancellationToken, Task<TResponse>> handler,
        QosProfile? qos = null,
        Encoding? textEncoding = null,
        object? state = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse;

    /// <summary>
    /// Create an ROS service server which sends and receives messages using native message buffer.
    /// </summary>
    /// <typeparam name="TService">Type of the service.</typeparam>
    /// <param name="serviceName"></param>
    /// <param name="handler"></param>
    /// <param name="qos"></param>
    /// <returns></returns>
    IRclService CreateNativeService<TService>(
        string serviceName,
        INativeServiceHandler handler,
        QosProfile? qos = null)
        where TService : IService;

    /// <summary>
    /// Create an ROS service server which sends and receives messages using native message buffer.
    /// </summary>
    /// <typeparam name="TService">Type of the service.</typeparam>
    /// <param name="serviceName"></param>
    /// <param name="handler"></param>
    /// <param name="qos"></param>
    /// <param name="state"></param>
    /// <returns></returns>
    IRclService CreateNativeService<TService>(
        string serviceName,
        Action<RosMessageBuffer, RosMessageBuffer, object?> handler,
        QosProfile? qos = null,
        object? state = null)
        where TService : IService;

    /// <summary>
    /// Create an ROS service server which sends and receives messages using native message buffer,
    /// and is able to handle multiple requests concurrently.
    /// </summary>
    /// <typeparam name="TService">Type of the service.</typeparam>
    /// <param name="serviceName"></param>
    /// <param name="handler"></param>
    /// <param name="qos"></param>
    /// <returns></returns>
    IRclService CreateConcurrentNativeService<TService>(
        string serviceName,
        IConcurrentNativeServiceHandler handler,
        QosProfile? qos = null)
        where TService : IService;

    /// <summary>
    /// Create an ROS service server which sends and receives messages using native message buffer,
    /// and is able to handle multiple requests concurrently.
    /// </summary>
    /// <typeparam name="TService">Type of the service.</typeparam>
    /// <param name="serviceName"></param>
    /// <param name="handler"></param>
    /// <param name="qos"></param>
    /// <param name="state"></param>
    /// <returns></returns>
    IRclService CreateConcurrentNativeService<TService>(
        string serviceName,
        Func<RosMessageBuffer, RosMessageBuffer, object?, CancellationToken, Task> handler,
        QosProfile? qos = null,
        object? state = null)
        where TService : IService;

    /// <summary>
    /// Create an ROS service client.
    /// </summary>
    /// <typeparam name="TService">Type of the service.</typeparam>
    /// <typeparam name="TRequest">Type of the request message.</typeparam>
    /// <typeparam name="TResponse">Type of the response message.</typeparam>
    /// <param name="serviceName">Name of the service to call.</param>
    /// <param name="qos"><see cref="QosProfile"/> to be used for the client. Defaults to <see cref="QosProfile.ServicesDefault"/>.</param>
    /// <param name="textEncoding">Specify the encoding of the string in the message. Defaults to <see cref="Encoding.UTF8"/>.</param>
    /// <returns></returns>
    IRclClient<TRequest, TResponse> CreateClient<TService, TRequest, TResponse>(
        string serviceName,
        QosProfile? qos = null,
        Encoding? textEncoding = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse;

    /// <summary>
    /// Create an ROS action client.
    /// </summary>
    /// <typeparam name="TAction">Type of the action.</typeparam>
    /// <typeparam name="TGoal">Type of the goal message.</typeparam>
    /// <typeparam name="TResult">Type of the result message.</typeparam>
    /// <typeparam name="TFeedback">Type of the feedback message.</typeparam>
    /// <param name="actionName">Name of the action.</param>
    /// <param name="textEncoding">Specify the encoding of the string in the message. Defaults to <see cref="Encoding.UTF8"/>.</param>
    /// <returns></returns>
    IActionClient<TGoal, TResult, TFeedback> CreateActionClient<TAction, TGoal, TResult, TFeedback>(
        string actionName,
        Encoding? textEncoding = null)
        where TAction : IAction<TGoal, TResult, TFeedback>
        where TGoal : IActionGoal
        where TFeedback : IActionFeedback
        where TResult : IActionResult;

    /// <summary>
    /// Create an ROS action server which sends and receives messages using native message buffer.
    /// </summary>
    /// <typeparam name="TAction">Type of the action.</typeparam>
    /// <param name="actionName"></param>
    /// <param name="handler"></param>
    /// <param name="clock"></param>
    /// <param name="resultTimeout">
    /// Timeout of the result cache.
    /// Setting to a negative <see cref="TimeSpan"/> will cause the results to be kept indefinitely until server shutdown.
    /// Setting to <see cref="TimeSpan.Zero"/> will disable caching and the result is removed as soon as the action
    /// client gets the result.
    /// <para>Default is 15 minutes.</para>
    /// </param>
    /// <param name="textEncoding"></param>
    /// <returns></returns>
    IActionServer CreateActionServer<TAction>(
        string actionName,
        INativeActionGoalHandler handler,
        RclClock? clock = null,
        TimeSpan? resultTimeout = null,
        Encoding? textEncoding = null)
        where TAction : IAction;

    /// <summary>
    /// Create an ROS action server.
    /// </summary>
    /// <typeparam name="TAction">Type of the action.</typeparam>
    /// <typeparam name="TGoal">Type of the goal message.</typeparam>
    /// <typeparam name="TResult">Type of the result message.</typeparam>
    /// <typeparam name="TFeedback">Type of the feedback message.</typeparam>
    /// <param name="actionName"></param>
    /// <param name="handler"></param>
    /// <param name="clock"></param>
    /// <param name="textEncoding"></param>
    /// <param name="resultTimeout">
    /// Timeout of the result cache.
    /// Setting to a negative <see cref="TimeSpan"/> will cause the results to be kept indefinitely until server shutdown.
    /// Setting to <see cref="TimeSpan.Zero"/> will disable caching and the result is removed as soon as the action
    /// client gets the result.
    /// <para>Default is 15 minutes.</para>
    /// </param>
    /// <returns></returns>
    IActionServer CreateActionServer<TAction, TGoal, TResult, TFeedback>(
        string actionName,
        IActionGoalHandler<TGoal,TResult, TFeedback> handler,
        RclClock? clock = null,
        TimeSpan? resultTimeout = null,
        Encoding? textEncoding = null)
        where TAction : IAction<TGoal, TResult, TFeedback>
        where TGoal : IActionGoal
        where TFeedback : IActionFeedback
        where TResult : IActionResult;
}