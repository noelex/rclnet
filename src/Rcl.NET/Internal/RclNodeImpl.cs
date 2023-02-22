using Rcl.Actions;
using Rcl.Actions.Client;
using Rcl.Actions.Server;
using Rcl.Graph;
using Rcl.Internal.Clients;
using Rcl.Internal.Services;
using Rcl.Internal.Subscriptions;
using Rcl.Logging;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System.Text;
using System.Threading.Channels;

namespace Rcl.Internal;

class RclNodeImpl : RclObject<SafeNodeHandle>, IRclNode
{
    private readonly RosGraph _graph;

    private readonly CancellationTokenSource _cts = new();

    public unsafe RclNodeImpl(
        RclContext context,
        string name,
         string @namespace = "",
        NodeOptions? options = null)
        : base(new(context.Handle, name, @namespace, options ?? NodeOptions.Default))
    {
        Context = context;
        Options = options ?? NodeOptions.Default;

        _graph = new(this);

        var graphSignal = new RclGuardConditionImpl(context,
            new(rcl_node_get_graph_guard_condition(Handle.Object)));
        _ = GraphBuilder(graphSignal, _cts.Token);

        Name = StringMarshal.CreatePooledString(rcl_node_get_name(Handle.Object))!;
        Namespace = StringMarshal.CreatePooledString(rcl_node_get_namespace(Handle.Object))!;
        FullyQualifiedName = StringMarshal.CreatePooledString(rcl_node_get_fully_qualified_name(Handle.Object))!;

        Logger = context.LoggerFactory.CreateLogger(StringMarshal.CreatePooledString(rcl_node_get_logger_name(Handle.Object))!);
    }

    public NodeOptions Options { get; }

    public RclContext Context { get; }

    public RosGraph Graph => _graph;

    public IRclLogger Logger { get; }

    public string Name { get; }

    public string Namespace { get; }

    public string FullyQualifiedName { get; }

    public unsafe ulong InstanceId
         => rcl_node_get_rcl_instance_id(Handle.Object);

    public unsafe bool IsValid
         => rcl_node_is_valid(Handle.Object);

    public unsafe nuint DomaindId
    {
        get
        {
            size_t s;
            rcl_node_get_domain_id(Handle.Object, &s);
            return s;
        }
    }

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

    private async Task GraphBuilder(RclGuardConditionImpl graphSignal, CancellationToken cancellationToken)
    {
        await Context.Yield();

        using (graphSignal)
        {
            try
            {
                while (!cancellationToken.IsCancellationRequested)
                {
                    try
                    {
                        _graph.Build();
                    }
                    catch (Exception e)
                    {
                        Console.WriteLine("Unable to build ROS graph: " + e.Message);
                        Console.WriteLine(e.StackTrace);
                    }

                    await graphSignal.WaitOneAsync(cancellationToken).ConfigureAwait(false);
                }
            }
            finally
            {
                _graph.Complete();
            }
        }
    }

    public override void Dispose()
    {
        _cts.Cancel();
        _cts.Dispose();
        base.Dispose();
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
        RclClock? clock = null, TimeSpan? resultTimeout = null, Encoding? textEncoding = null) where TAction : IAction
        => new ActionServer(this,
            actionName,
            TAction.TypeSupportName,
            TAction.GetTypeSupportHandle(), handler,
            textEncoding ?? Encoding.UTF8,
            clock ?? RclClock.Ros,
            resultTimeout ?? TimeSpan.FromMinutes(15));

    public IActionServer CreateActionServer<TAction, TGoal, TResult, TFeedback>(
        string actionName,
        IActionGoalHandler<TGoal, TResult, TFeedback> handler,
        RclClock? clock = null,
        TimeSpan? resultTimeout = null,
        Encoding? textEncoding = null)
        where TAction : IAction<TGoal, TResult, TFeedback>
        where TGoal : IActionGoal
        where TResult : IActionResult
        where TFeedback : IActionFeedback
        => CreateActionServer<TAction>(
            actionName,
            new ActionGoalHandlerWrapper<TGoal, TResult, TFeedback>(handler, textEncoding ?? Encoding.UTF8),
            clock,
            resultTimeout,
            textEncoding);
}