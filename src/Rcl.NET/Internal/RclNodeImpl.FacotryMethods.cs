﻿using Rcl.Actions.Client;
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
       QosProfile? qos = null,
       int queueSize = 1,
       BoundedChannelFullMode fullMode = BoundedChannelFullMode.DropOldest) where T : IMessage
    {
        return new NativeSubscription<T>(this, topicName,
                qos ?? QosProfile.Default,
                queueSize,
                fullMode);
    }

    public IRclSubscription<T> CreateSubscription<T>(
        string topicName,
        QosProfile? qos = null,
        int queueSize = 1,
        BoundedChannelFullMode fullMode = BoundedChannelFullMode.DropOldest,
        Encoding? textEncoding = null) where T : IMessage
    {
        return new RclSubscription<T>(
                this,
                topicName,
                qos ?? QosProfile.Default,
                queueSize,
                fullMode,
                textEncoding ?? Encoding.UTF8);
    }

    public IRclPublisher<T> CreatePublisher<T>(
        string topicName,
        QosProfile? qos = null,
        Encoding? textEncoding = null)
        where T : IMessage
    {
        return new RclPublisher<T>(
            this, topicName,
            qos ?? QosProfile.Default,
            textEncoding ?? Encoding.UTF8);
    }

    public IRclService CreateService<TService, TRequest, TResponse>(
        string serviceName,
        IServiceHandler<TRequest, TResponse> handler,
        QosProfile? qos = null,
        Encoding? textEncoding = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse
    {
        return new DefaultService<TService, TRequest, TResponse>(
            this,
            serviceName,
            handler,
            qos ?? QosProfile.ServicesDefault,
            textEncoding ?? Encoding.UTF8);
    }

    public IRclService CreateService<TService, TRequest, TResponse>(
        string serviceName,
        Func<TRequest, object?, TResponse> handler,
        QosProfile? qos = null,
        Encoding? textEncoding = null,
        object? state = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse
    {
        return new DefaultService<TService, TRequest, TResponse>(
            this,
            serviceName,
            new DelegateDefaultServiceCallHandler<TRequest, TResponse>(handler, state),
            qos ?? QosProfile.ServicesDefault,
            textEncoding ?? Encoding.UTF8);
    }

    public IRclService CreateConcurrentService<TService, TRequest, TResponse>(
        string serviceName,
        IConcurrentServiceHandler<TRequest, TResponse> handler,
        QosProfile? qos = null,
        Encoding? textEncoding = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse
    {
        return new ConcurrentService<TService, TRequest, TResponse>(
            this,
            serviceName,
            handler,
            qos ?? QosProfile.ServicesDefault,
            textEncoding ?? Encoding.UTF8);
    }

    public IRclService CreateConcurrentService<TService, TRequest, TResponse>(
        string serviceName,
        Func<TRequest, object?, CancellationToken, Task<TResponse>> handler,
        QosProfile? qos = null,
        Encoding? textEncoding = null,
        object? state = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse
    {
        return new ConcurrentService<TService, TRequest, TResponse>(
            this,
            serviceName,
            new DelegateConcurrentServiceCallHandler<TRequest, TResponse>(handler, state),
            qos ?? QosProfile.ServicesDefault,
            textEncoding ?? Encoding.UTF8);
    }

    public IRclService CreateNativeService<TService>(
        string serviceName,
        INativeServiceHandler handler,
        QosProfile? qos = null)
        where TService : IService
    {
        return new IntrospectionService(
            this,
            serviceName,
            TService.GetTypeSupportHandle(),
            handler,
            qos ?? QosProfile.ServicesDefault);
    }

    public IRclService CreateNativeService<TService>(
        string serviceName,
        Action<RosMessageBuffer, RosMessageBuffer, object?> handler,
        QosProfile? qos = null,
        object? state = null)
        where TService : IService
    {
        return new IntrospectionService(
            this,
            serviceName,
            TService.GetTypeSupportHandle(),
            new DelegateNativeServiceCallHandler(handler, state),
            qos ?? QosProfile.ServicesDefault);
    }

    public IRclService CreateConcurrentNativeService<TService>(
        string serviceName,
        IConcurrentNativeServiceHandler handler,
        QosProfile? qos = null)
        where TService : IService
    {
        return new ConcurrentIntrospectionService(
            this,
            serviceName,
            handler,
            TService.GetTypeSupportHandle(),
            qos ?? QosProfile.ServicesDefault);
    }

    public IRclService CreateConcurrentNativeService<TService>(
        string serviceName,
        Func<RosMessageBuffer, RosMessageBuffer, object?, CancellationToken, Task> handler,
        QosProfile? qos = null,
        object? state = null)
        where TService : IService
    {
        return new ConcurrentIntrospectionService(
            this,
            serviceName,
            new DelegateConcurrentNativeServiceCallHandler(handler, state),
            TService.GetTypeSupportHandle(),
            qos ?? QosProfile.ServicesDefault);
    }

    public IRclClient<TRequest, TResponse> CreateClient<TService, TRequest, TResponse>(
        string serviceName,
        QosProfile? qos = null,
        Encoding? textEncoding = null)
        where TService : IService<TRequest, TResponse>
        where TRequest : IServiceRequest
        where TResponse : IServiceResponse
    {
        return new RclClient<TService, TRequest, TResponse>(
            this,
            serviceName,
            qos ?? QosProfile.ServicesDefault,
            textEncoding ?? Encoding.UTF8);
    }

    public IActionClient<TGoal, TResult, TFeedback> CreateActionClient<TAction, TGoal, TResult, TFeedback>(
        string actionName,
        Encoding? textEncoding = null)
        where TAction : IAction<TGoal, TResult, TFeedback>
        where TGoal : IActionGoal
        where TFeedback : IActionFeedback
        where TResult : IActionResult
    {
        return new ActionClient<TAction, TGoal, TResult, TFeedback>(this, actionName, textEncoding ?? Encoding.UTF8);
    }

    public IRclNativeSubscription CreateNativeSubscription(
        string topicName,
        TypeSupportHandle typeSupport,
        QosProfile? qos = null,
        int queueSize = 1,
        BoundedChannelFullMode fullMode = BoundedChannelFullMode.DropOldest)
    {
        return new IntrospectionNativeSubscription(
            this,
            topicName,
            typeSupport,
            qos ?? QosProfile.Default,
            queueSize,
            fullMode);
    }

    public IActionServer CreateActionServer<TAction>(
        string actionName, INativeActionGoalHandler handler,
        TimeSpan? resultTimeout = null, Encoding? textEncoding = null) where TAction : IAction
        => new ActionServer(this,
            actionName,
            TAction.TypeSupportName,
            TAction.GetTypeSupportHandle(), handler,
            textEncoding ?? Encoding.UTF8,
            resultTimeout ?? TimeSpan.FromMinutes(15));

    public IActionServer CreateActionServer<TAction, TGoal, TResult, TFeedback>(
        string actionName,
        IActionGoalHandler<TGoal, TResult, TFeedback> handler,
        TimeSpan? resultTimeout = null,
        Encoding? textEncoding = null)
        where TAction : IAction<TGoal, TResult, TFeedback>
        where TGoal : IActionGoal
        where TResult : IActionResult
        where TFeedback : IActionFeedback
        => CreateActionServer<TAction>(
            actionName,
            new ActionGoalHandlerWrapper<TGoal, TResult, TFeedback>(handler, textEncoding ?? Encoding.UTF8),
            resultTimeout,
            textEncoding);
}