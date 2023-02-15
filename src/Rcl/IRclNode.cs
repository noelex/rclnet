using Rcl.Actions;
using Rcl.Graph;
using Rcl.Internal.Services;
using Rcl.Qos;
using Rosidl.Runtime;
using System.Text;
using System.Threading.Channels;

namespace Rcl;

public interface IRclNode : IRclObject
{
    RclContext Context { get; }

    nuint DomaindId { get; }

    string? FullyQualifiedName { get; }

    RosGraph Graph { get; }

    ulong InstanceId { get; }

    bool IsValid { get; }

    string? LoggerName { get; }

    string? Name { get; }

    string? Namespace { get; }

    /// <summary>
    /// Create an ROS subscription and receive messages using unamanaged message buffers.
    /// </summary>
    /// <typeparam name="T">Type of the message to receive.</typeparam>
    /// <param name="topicName">Name of the topic to subscribe.</param>
    /// <param name="profile"><see cref="QosProfile"/> to be used for the subscription. Defaults to <see cref="QosProfile.Default"/>.</param>
    /// <param name="queueSize">Size of the message delivery queue. This parameter is used setup the delivery queue on .NET side, and is irrelevant to <see cref="QosProfile.Depth"/>. </param>
    /// <param name="fullMode">Behavior to use when the delivery queue is full.</param>
    /// <returns>A <see cref="IRclNativeSubscription"/> to be used for receiving topic messages.</returns>
    IRclNativeSubscription CreateNativeSubscription<T>(
        string topicName,
        QosProfile? qos = null,
        int queueSize = 1,
        BoundedChannelFullMode fullMode = BoundedChannelFullMode.DropOldest) where T : IMessage;

    /// <summary>
    /// Create an ROS subscription and receive messages using unamanaged message buffers.
    /// </summary>
    /// <param name="topicName">Name of the topic to subscribe.</param>
    /// <param name="queueSize">Size of the message delivery queue. This parameter is used setup the delivery queue on .NET side, and is irrelevant to <see cref="QosProfile.Depth"/>. </param>
    /// <param name="fullMode">Behavior to use when the delivery queue is full.</param>
    /// <returns>A <see cref="IRclNativeSubscription"/> to be used for receiving topic messages.</returns>
    IRclNativeSubscription CreateNativeSubscription(
        string topicName,
        TypeSupportHandle typeSupport,
        QosProfile? qos = null,
        int queueSize = 1,
        BoundedChannelFullMode fullMode = BoundedChannelFullMode.DropOldest);

    IRclSubscription<T> CreateSubscription<T>(
        string topicName,
        QosProfile? qos = null,
        int queueSize = 1,
        BoundedChannelFullMode fullMode = BoundedChannelFullMode.DropOldest,
        Encoding? textEncoding = null) where T : IMessage;

    IRclPublisher<T> CreatePublisher<T>(
        string topicName,
        QosProfile? qos = null,
        Encoding? textEncoding = null) where T : IMessage;

    IRclService CreateService<TService, TRequest, TResponse>(
            string serviceName,
            IServiceHandler<TRequest, TResponse> handler,
            QosProfile? qos = null,
            Encoding? textEncoding = null)
            where TService : IService<TRequest, TResponse>
            where TRequest : IServiceRequest
            where TResponse : IServiceResponse;

    IRclService CreateService<TService, TRequest, TResponse>(
        string serviceName,
        Func<TRequest, object?, TResponse> handler,
        QosProfile? qos = null,
        Encoding? textEncoding = null,
        object? state = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse;

    IRclService CreateConcurrentService<TService, TRequest, TResponse>(
        string serviceName,
        IConcurrentServiceHandler<TRequest, TResponse> handler,
        QosProfile? qos = null,
        Encoding? textEncoding = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse;

    IRclService CreateConcurrentService<TService, TRequest, TResponse>(
        string serviceName,
        Func<TRequest, object?, CancellationToken, Task<TResponse>> handler,
        QosProfile? qos = null,
        Encoding? textEncoding = null,
        object? state = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse;

    IRclService CreateNativeService<TService, TRequest, TResponse>(
        string serviceName,
        INativeServiceHandler handler,
        QosProfile? qos = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse;

    IRclService CreateNativeService<TService, TRequest, TResponse>(
        string serviceName,
        Action<RosMessageBuffer, RosMessageBuffer, object?> handler,
        QosProfile? qos = null,
        object? state = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse;

    IRclService CreateConcurrentNativeService<TService, TRequest, TResponse>(
        string serviceName,
        IConcurrentNativeServiceHandler handler,
        QosProfile? qos = null,
        Encoding? textEncoding = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse;

    IRclService CreateConcurrentNativeService<TService, TRequest, TResponse>(
        string serviceName,
        Func<RosMessageBuffer, RosMessageBuffer, object?, CancellationToken, Task> handler,
        QosProfile? qos = null,
        object? state = null,
        Encoding? textEncoding = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse;

    IRclClient<TRequest, TResponse> CreateClient<TService, TRequest, TResponse>(
        string serviceName,
        QosProfile? qos = null,
        Encoding? textEncoding = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse;

    IActionClient<TGoal, TResult, TFeedback> CreateActionClient<TAction, TGoal, TResult, TFeedback>(
        string actionName,
        Encoding? textEncoding = null)
        where TAction : IAction<TGoal, TResult, TFeedback>
        where TGoal : IActionGoal
        where TFeedback : IActionFeedback
        where TResult : IActionResult;
}