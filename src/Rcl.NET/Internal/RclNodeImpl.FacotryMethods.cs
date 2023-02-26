using Rcl.Actions.Client;
using Rcl.Actions;
using Rcl.Internal.Clients;
using Rcl.Internal.Services;
using Rcl.Internal.Subscriptions;
using Rcl.Qos;
using Rosidl.Runtime;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Channels;
using System.Threading.Tasks;
using Rcl.Actions.Server;

namespace Rcl.Internal;

internal partial class RclNodeImpl
{
    public IRclNativeSubscription CreateNativeSubscription<T>(
       string topicName,
       SubscriptionOptions? options = null) where T : IMessage
    {
        return new NativeSubscription<T>(this, topicName, options ?? SubscriptionOptions.Default);
    }

    public IRclSubscription<T> CreateSubscription<T>(
        string topicName,
        SubscriptionOptions? options = null) where T : IMessage
    {
        return new RclSubscription<T>(this,topicName, options ?? SubscriptionOptions.Default);
    }

    public IRclPublisher<T> CreatePublisher<T>(
        string topicName, PublisherOptions? options = null)
        where T : IMessage
    {
        return new RclPublisher<T>(
            this, topicName, options ?? PublisherOptions.Default);
    }

    public IRclService CreateService<TService, TRequest, TResponse>(
        string serviceName,
        IServiceHandler<TRequest, TResponse> handler,
        ServerOptions? options = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse
    {
        return new DefaultService<TService, TRequest, TResponse>(
            this,
            serviceName,
            handler,
            options ?? ServerOptions.Default);
    }

    public IRclService CreateService<TService, TRequest, TResponse>(
        string serviceName,
        Func<TRequest, object?, TResponse> handler,
        object? state = null,
        ServerOptions? options = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse
    {
        return new DefaultService<TService, TRequest, TResponse>(
            this,
            serviceName,
            new DelegateDefaultServiceCallHandler<TRequest, TResponse>(handler, state),
            options ?? ServerOptions.Default);
    }

    public IRclService CreateConcurrentService<TService, TRequest, TResponse>(
        string serviceName,
        IConcurrentServiceHandler<TRequest, TResponse> handler,
        ServerOptions? options = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse
    {
        return new ConcurrentService<TService, TRequest, TResponse>(
            this,
            serviceName,
            handler,
            options ?? ServerOptions.Default);
    }

    public IRclService CreateConcurrentService<TService, TRequest, TResponse>(
        string serviceName,
        Func<TRequest, object?, CancellationToken, Task<TResponse>> handler,
        object? state = null,
        ServerOptions? options = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse
    {
        return new ConcurrentService<TService, TRequest, TResponse>(
            this,
            serviceName,
            new DelegateConcurrentServiceCallHandler<TRequest, TResponse>(handler, state),
            options ?? ServerOptions.Default);
    }

    public IRclService CreateNativeService<TService>(
        string serviceName,
        INativeServiceHandler handler,
        ServerOptions? options = null)
        where TService : IService
    {
        return new IntrospectionService(
            this,
            serviceName,
            TService.GetTypeSupportHandle(),
            handler,
            options ?? ServerOptions.Default);
    }

    public IRclService CreateNativeService<TService>(
        string serviceName,
        Action<RosMessageBuffer, RosMessageBuffer, object?> handler,
        object? state = null,
        ServerOptions? options = null)
        where TService : IService
    {
        return new IntrospectionService(
            this,
            serviceName,
            TService.GetTypeSupportHandle(),
            new DelegateNativeServiceCallHandler(handler, state),
            options ?? ServerOptions.Default);
    }

    public IRclService CreateConcurrentNativeService<TService>(
        string serviceName,
        IConcurrentNativeServiceHandler handler,
        ServerOptions? options = null)
        where TService : IService
    {
        return new ConcurrentIntrospectionService(
            this,
            serviceName,
            handler,
            TService.GetTypeSupportHandle(),
            options ?? ServerOptions.Default);
    }

    public IRclService CreateConcurrentNativeService<TService>(
        string serviceName,
        Func<RosMessageBuffer, RosMessageBuffer, object?, CancellationToken, Task> handler,
        object? state = null,
        ServerOptions? options = null)
        where TService : IService
    {
        return new ConcurrentIntrospectionService(
            this,
            serviceName,
            new DelegateConcurrentNativeServiceCallHandler(handler, state),
            TService.GetTypeSupportHandle(),
            options ?? ServerOptions.Default);
    }

    public IRclClient<TRequest, TResponse> CreateClient<TService, TRequest, TResponse>(
        string serviceName,
        ClientOptions? options = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse
    {
        return new RclClient<TService, TRequest, TResponse>(
            this,
            serviceName,
            options ?? ClientOptions.Default);
    }

    public IActionClient<TGoal, TResult, TFeedback> CreateActionClient<TAction, TGoal, TResult, TFeedback>(
        string actionName,
        ActionClientOptions? options = null)
        where TAction : IAction<TGoal, TResult, TFeedback>
        where TGoal : IActionGoal
        where TFeedback : IActionFeedback
        where TResult : IActionResult
    {
        return new ActionClient<TAction, TGoal, TResult, TFeedback>(
            this, actionName, options ?? ActionClientOptions.Default);
    }

    public IRclNativeSubscription CreateNativeSubscription(
        string topicName,
        TypeSupportHandle typeSupport,
        SubscriptionOptions? options = null)
    {
        return new IntrospectionNativeSubscription(
            this,
            topicName,
            typeSupport,
            options ?? SubscriptionOptions.Default);
    }

    public IActionServer CreateActionServer<TAction>(
        string actionName, INativeActionGoalHandler handler,
        ActionServerOptions? options = null) where TAction : IAction
        => new ActionServer(this,
            actionName,
            TAction.TypeSupportName,
            TAction.GetTypeSupportHandle(), handler,
            options ?? ActionServerOptions.Default);

    public IActionServer CreateActionServer<TAction, TGoal, TResult, TFeedback>(
        string actionName,
        IActionGoalHandler<TGoal, TResult, TFeedback> handler,
        ActionServerOptions? options = null)
        where TAction : IAction<TGoal, TResult, TFeedback>
        where TGoal : IActionGoal
        where TResult : IActionResult
        where TFeedback : IActionFeedback
    {
        options ??= ActionServerOptions.Default;
        return CreateActionServer<TAction>(
            actionName,
            new ActionGoalHandlerWrapper<TGoal, TResult, TFeedback>(handler, options.TextEncoding),
            options);
    }
}
