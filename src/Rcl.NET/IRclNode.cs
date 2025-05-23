﻿using Rcl.Actions;
using Rcl.Graph;
using Rcl.Logging;
using Rcl.Parameters;
using Rosidl.Runtime;

namespace Rcl;

/// <summary>
/// Represents an ROS node.
/// </summary>
public interface IRclNode : IRclObject
{
    /// <summary>
    /// Gets the <see cref="IRclContext"/> object which the node belongs.
    /// </summary>
    IRclContext Context { get; }

    /// <summary>
    /// The <see cref="RclClock"/> used by services hosted in current <see cref="IRclNode"/>.
    /// </summary>
    IRclClock Clock { get; }

    /// <summary>
    /// Gets an <see cref="IParameterService"/> for accessing parameters in the scope of the node.
    /// </summary>
    IParameterService Parameters { get; }

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
    /// <param name="options"><see cref="SubscriptionOptions"/> to be used for the subscription. Defaults to <see cref="SubscriptionOptions.Default"/>.</param>
    /// <returns>A <see cref="IRclNativeSubscription"/> to be used for receiving topic messages.</returns>
    IRclNativeSubscription CreateNativeSubscription<T>(
        string topicName, SubscriptionOptions? options = null) where T : IMessage;

    /// <summary>
    /// Create an ROS subscription and receive messages using native message buffers.
    /// </summary>
    /// <param name="topicName">Name of the topic to subscribe.</param>
    /// <param name="typeSupport">Type support handle of the message.</param>
    /// <param name="options"><see cref="SubscriptionOptions"/> to be used for the subscription. Defaults to <see cref="SubscriptionOptions.Default"/>.</param>
    /// <returns>A <see cref="IRclNativeSubscription"/> to be used for receiving topic messages.</returns>
    IRclNativeSubscription CreateNativeSubscription(
        string topicName, TypeSupportHandle typeSupport, SubscriptionOptions? options = null);

    /// <summary>
    /// Create an ROS subscription and receive messages of type <typeparamref name="T"/>.
    /// </summary>
    /// <typeparam name="T">Type of the message to receive.</typeparam>
    /// <param name="topicName">Name of the topic to subscribe.</param>
    /// <param name="options"><see cref="SubscriptionOptions"/> to be used for the subscription. Defaults to <see cref="SubscriptionOptions.Default"/>.</param>
    /// <returns>A <see cref="IRclNativeSubscription"/> to be used for receiving topic messages.</returns>
    IRclSubscription<T> CreateSubscription<T>(
        string topicName, SubscriptionOptions? options = null) where T : IMessage;

    /// <summary>
    /// Create an ROS publisher to publish messages of type <typeparamref name="T"/>
    /// </summary>
    /// <typeparam name="T"></typeparam>
    /// <param name="topicName">Name of the topic.</param>
    /// <param name="options"><see cref="PublisherOptions"/> to be used for the publisher. Defaults to <see cref="PublisherOptions.Default"/>.</param>
    /// <returns>A <see cref="IRclPublisher{T}"/> to be used for publishing topic messages.</returns>
    IRclPublisher<T> CreatePublisher<T>(string topicName, PublisherOptions? options = null) where T : IMessage;

    /// <summary>
    /// Create an ROS service server.
    /// </summary>
    /// <typeparam name="TService">Type of the service.</typeparam>
    /// <typeparam name="TRequest">Type of the service request.</typeparam>
    /// <typeparam name="TResponse">Type of the service response.</typeparam>
    /// <param name="serviceName"></param>
    /// <param name="handler"></param>
    /// <param name="options">
    /// <see cref="ServerOptions"/> to be used for the server.
    /// Defaults to <see cref="ServerOptions.Default"/>.
    /// </param>
    /// <returns></returns>
    IRclService CreateService<TService, TRequest, TResponse>(
        string serviceName,
        IServiceHandler<TRequest, TResponse> handler,
        ServerOptions? options = null)
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
    /// <param name="options">
    /// <see cref="ServerOptions"/> to be used for the server.
    /// Defaults to <see cref="ServerOptions.Default"/>.
    /// </param>
    /// <param name="state"></param>
    /// <returns></returns>
    IRclService CreateService<TService, TRequest, TResponse>(
        string serviceName,
        Func<TRequest, object?, TResponse> handler,
        object? state = null,
        ServerOptions? options = null)
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
    /// <param name="options">
    /// <see cref="ServerOptions"/> to be used for the server.
    /// Defaults to <see cref="ServerOptions.Default"/>.
    /// </param>
    /// <returns></returns>
    IRclService CreateConcurrentService<TService, TRequest, TResponse>(
        string serviceName,
        IConcurrentServiceHandler<TRequest, TResponse> handler,
        ServerOptions? options = null)
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
    /// <param name="state"></param>
    /// <param name="options">
    /// <see cref="ServerOptions"/> to be used for the server.
    /// Defaults to <see cref="ServerOptions.Default"/>.
    /// </param>
    /// <returns></returns>
    IRclService CreateConcurrentService<TService, TRequest, TResponse>(
        string serviceName,
        Func<TRequest, object?, CancellationToken, Task<TResponse>> handler,
        object? state = null,
        ServerOptions? options = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse;

    /// <summary>
    /// Create an ROS service server which sends and receives messages using native message buffer.
    /// </summary>
    /// <typeparam name="TService">Type of the service.</typeparam>
    /// <param name="serviceName"></param>
    /// <param name="handler"></param>
    /// <param name="options">
    /// <see cref="ServerOptions"/> to be used for the server.
    /// Defaults to <see cref="ServerOptions.Default"/>.
    /// </param>
    /// <returns></returns>
    IRclService CreateNativeService<TService>(
        string serviceName,
        INativeServiceHandler handler,
        ServerOptions? options = null)
        where TService : IService;

    /// <summary>
    /// Create an ROS service server which sends and receives messages using native message buffer.
    /// </summary>
    /// <typeparam name="TService">Type of the service.</typeparam>
    /// <param name="serviceName"></param>
    /// <param name="handler"></param>
    /// <param name="options">
    /// <see cref="ServerOptions"/> to be used for the server.
    /// Defaults to <see cref="ServerOptions.Default"/>.
    /// </param>
    /// <param name="state"></param>
    /// <returns></returns>
    IRclService CreateNativeService<TService>(
        string serviceName,
        Action<RosMessageBuffer, RosMessageBuffer, object?> handler,
        object? state = null,
        ServerOptions? options = null)
        where TService : IService;

    /// <summary>
    /// Create an ROS service server which sends and receives messages using native message buffer,
    /// and is able to handle multiple requests concurrently.
    /// </summary>
    /// <typeparam name="TService">Type of the service.</typeparam>
    /// <param name="serviceName"></param>
    /// <param name="handler"></param>
    /// <param name="options">
    /// <see cref="ServerOptions"/> to be used for the server.
    /// Defaults to <see cref="ServerOptions.Default"/>.
    /// </param>
    /// <returns></returns>
    IRclService CreateConcurrentNativeService<TService>(
        string serviceName,
        IConcurrentNativeServiceHandler handler,
        ServerOptions? options = null)
        where TService : IService;

    /// <summary>
    /// Create an ROS service server which sends and receives messages using native message buffer,
    /// and is able to handle multiple requests concurrently.
    /// </summary>
    /// <typeparam name="TService">Type of the service.</typeparam>
    /// <param name="serviceName">The name of the service.</param>
    /// <param name="handler">A callback for processing service requests.</param>
    /// <param name="options">
    /// <see cref="ServerOptions"/> to be used for the server.
    /// Defaults to <see cref="ServerOptions.Default"/>.
    /// </param>
    /// <param name="state">A custom state object to be passed to the handler callback.</param>
    /// <returns></returns>
    IRclService CreateConcurrentNativeService<TService>(
        string serviceName,
        Func<RosMessageBuffer, RosMessageBuffer, object?, CancellationToken, Task> handler,
        object? state = null,
        ServerOptions? options = null)
        where TService : IService;

    /// <summary>
    /// Create an ROS service client.
    /// </summary>
    /// <typeparam name="TService">Type of the service.</typeparam>
    /// <typeparam name="TRequest">Type of the request message.</typeparam>
    /// <typeparam name="TResponse">Type of the response message.</typeparam>
    /// <param name="serviceName">Name of the service to call.</param>
    /// <param name="options"><see cref="PublisherOptions"/> to be used for the client. Defaults to <see cref="ClientOptions.Default"/>.</param>
    /// <returns>An <see cref="IRclClient{TRequest, TResponse}"/> object invoking service calls.</returns>
    IRclClient<TRequest, TResponse> CreateClient<TService, TRequest, TResponse>(
        string serviceName,
        ClientOptions? options = null)
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
    /// <param name="options"><see cref="ActionClientOptions"/> to be used for the client. Defaults to <see cref="ActionClientOptions.Default"/>.</param>
    /// <returns></returns>
    IActionClient<TGoal, TResult, TFeedback> CreateActionClient<TAction, TGoal, TResult, TFeedback>(
        string actionName,
        ActionClientOptions? options = null)
        where TAction : IAction<TGoal, TResult, TFeedback>
        where TGoal : IActionGoal
        where TFeedback : IActionFeedback
        where TResult : IActionResult;

    /// <summary>
    /// Create an ROS action server which sends and receives messages using native message buffer.
    /// </summary>
    /// <typeparam name="TAction">Type of the action.</typeparam>
    /// <param name="actionName">The name of the action.</param>
    /// <param name="handler">A <see cref="INativeActionGoalHandler"/> for processing action goals.</param>
    /// <param name="options">
    /// <see cref="ActionServerOptions"/> to be used for the client. Defaults to <see cref="ActionServerOptions.Default"/>.
    /// </param>
    /// <returns></returns>
    IActionServer CreateActionServer<TAction>(
        string actionName,
        INativeActionGoalHandler handler,
        ActionServerOptions? options = null)
        where TAction : IAction;

    /// <summary>
    /// Create an ROS action server.
    /// </summary>
    /// <typeparam name="TAction">Type of the action.</typeparam>
    /// <typeparam name="TGoal">Type of the goal message.</typeparam>
    /// <typeparam name="TResult">Type of the result message.</typeparam>
    /// <typeparam name="TFeedback">Type of the feedback message.</typeparam>
    /// <param name="actionName">The name of the action.</param>
    /// <param name="handler">A <see cref="IActionGoalHandler{TGoal, TResult, TFeedback}"/> for processing action goals.</param>
    /// <param name="options">
    /// <see cref="ActionServerOptions"/> to be used for the client. Defaults to <see cref="ActionServerOptions.Default"/>.
    /// </param>
    /// <returns></returns>
    IActionServer CreateActionServer<TAction, TGoal, TResult, TFeedback>(
        string actionName,
        IActionGoalHandler<TGoal, TResult, TFeedback> handler,
        ActionServerOptions? options = null)
        where TAction : IAction<TGoal, TResult, TFeedback>
        where TGoal : IActionGoal
        where TFeedback : IActionFeedback
        where TResult : IActionResult;
}