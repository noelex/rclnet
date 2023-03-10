using Rcl.Internal.Events;
using Rcl.Interop;
using Rcl.Introspection;
using Rcl.Logging;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System.Runtime.CompilerServices;

namespace Rcl.Internal.Publishers;

internal unsafe class RclNativePublisher : RclContextualObject<SafePublisherHandle>, IRclPublisher
{
    private readonly QosProfile _actualQos;
    private readonly IMessageIntrospection _introspection;
    private readonly RclNodeImpl _node;

    private readonly RclPubisherEvent? _livelinessEvent, _deadlineMissedEvent, _qosEvent;

    public RclNativePublisher(
        RclNodeImpl node,
        string topicName,
        TypeSupportHandle typesupport,
        PublisherOptions options)
        : base(node.Context, new(node.Handle, typesupport, topicName, options))
    {
        bool completelyInitialized = false;
        try
        {
            _node = node;
            ref var actualQos = ref Unsafe.AsRef<rmw_qos_profile_t>(
                rcl_publisher_get_actual_qos(Handle.Object));
            _actualQos = QosProfile.Create(in actualQos);

            _introspection = MessageIntrospection.Create(typesupport);
            Name = StringMarshal.CreatePooledString(rcl_publisher_get_topic_name(Handle.Object))!;
            Options = options;

            Endpoints = GetEndpoints();

            InitializePublisherEvents(options,
                ref _livelinessEvent, ref _deadlineMissedEvent, ref _qosEvent);

            completelyInitialized = true;
        }
        finally
        {
            if (!completelyInitialized) Dispose();
        }
    }

    private unsafe NetworkFlowEndpoint[] GetEndpoints()
    {
        if (!RosEnvironment.IsSupported(RosEnvironment.Humble))
        {
            return Array.Empty<NetworkFlowEndpoint>();
        }

        var allocator = RclAllocator.Default.Object;
        var endpoints = RclHumble.rmw_get_zero_initialized_network_flow_endpoint_array();
        

        try
        {
            RclException.ThrowIfNonSuccess(
                RclHumble.rcl_publisher_get_network_flow_endpoints(Handle.Object, &allocator, &endpoints));
            return InteropHelpers.ConvertNetworkFlowEndpoints(ref endpoints);
        }
        catch(Exception e)
        {
            _node.Context.DefaultLogger.LogDebug("Unable to retrieve network flow endpoints from publisher: " + e.Message);
            return Array.Empty<NetworkFlowEndpoint>();
        }
        finally
        {
            if (endpoints.allocator != null)
            {
                RclHumble.rmw_network_flow_endpoint_array_fini(&endpoints);
            }
        }
    }

    private void InitializePublisherEvents(
        PublisherOptions options,
        ref RclPubisherEvent? livelinessEvent,
        ref RclPubisherEvent? deadlineMissedEvent,
        ref RclPubisherEvent? qosEvent)
    {
        try
        {
            livelinessEvent = new RclPublisherLivelinessLostEvent(
                _node.Context, Handle,
                options.LivelinessLostHandler ?? OnLivelinessEvent);
        }
        catch (RclException ex)
        {
            if (options.LivelinessLostHandler != null)
            {
                throw;
            }
            _node.Context.DefaultLogger.LogDebug("Unable to register LivelinessLostEvent:");
            _node.Context.DefaultLogger.LogDebug(ex.Message);
        }

        try
        {
            deadlineMissedEvent = new RclPublisherOfferedDeadlineMissedEvent(
                _node.Context, Handle,
                options.OfferedDeadlineMissedHandler ?? OnDeadlineEvent);
        }
        catch (RclException ex)
        {
            if (options.OfferedDeadlineMissedHandler != null)
            {
                throw;
            }
            _node.Context.DefaultLogger.LogDebug("Unable to register OfferedDeadlineMissedEvent:");
            _node.Context.DefaultLogger.LogDebug(ex.Message);
        }

        try
        {
            qosEvent = new RclPublisherIncompatibleQosEvent(
                _node.Context, Handle,
                options.OfferedQosIncompatibleHandler ?? OnIncompatibleQosEvent);
        }
        catch (RclException ex)
        {
            if (options.OfferedQosIncompatibleHandler != null)
            {
                throw;
            }
            _node.Context.DefaultLogger.LogDebug("Unable to register OfferedQosIncompatibleEvent:");
            _node.Context.DefaultLogger.LogDebug(ex.Message);
        }
    }

    private void OnLivelinessEvent(LivelinessLostEvent info)
    {
        _node.Logger.LogDebug(
            $"Received LivelinessLostEvent on publisher of topic '{Name}': " +
            $"Total = {info.TotalCount}, Delta = {info.Delta}");
    }

    private void OnDeadlineEvent(OfferedDeadlineMissedEvent info)
    {
        _node.Logger.LogWarning(
            $"Received OfferedDeadlineMissedEvent on publisher of topic '{Name}': " +
            $"Total = {info.TotalCount}, Delta = {info.Delta}");
    }

    private void OnIncompatibleQosEvent(IncompatibleQosEvent info)
    {
        _node.Logger.LogWarning(
            $"Received IncompatibleQosEvent on publisher of topic '{Name}': " +
            $"Total = {info.TotalCount}, Delta = {info.Delta}, PolicyKind = {info.LastPolicyKind}");
    }

    public PublisherOptions Options { get; }

    public QosProfile ActualQos => _actualQos;

    public string Name { get; }

    public int Subscribers
    {
        get
        {
            size_t count;
            RclException.ThrowIfNonSuccess(
                rcl_publisher_get_subscription_count(Handle.Object, &count));
            return (int)count.Value;
        }
    }

    public bool IsValid
         => rcl_publisher_is_valid(Handle.Object);

    public NetworkFlowEndpoint[] Endpoints { get;}

    public void Publish(RosMessageBuffer message)
    {
        RclException.ThrowIfNonSuccess(
            rcl_publish(Handle.Object, message.Data.ToPointer(), null));
    }

    public ValueTask PublishAsync(RosMessageBuffer message) => PublishAsync(message, false);

    protected ValueTask PublishAsync(RosMessageBuffer message, bool disposeBuffer)
    {
        var vts = ObjectPool.Rent<ManualResetValueTaskSource<bool>>();
        var args = ObjectPool.Rent<PublishArgs>().Init(this, message, vts, disposeBuffer);

        // Allow the continuation to run synchronously on the thread pool to reduce
        // scheduling overhead, otherwise we will need 2 scheduling for 1 publish.
        // May cause stack overflow in very rare but possible case.
        vts.RunContinuationsAsynchronously = false;
        vts.OnFinally(static state =>
        {
            var args = (PublishArgs)state!;

            if (args.ShouldDisposeBuffer)
            {
                args.Buffer.Dispose();
            }

            args.TaskSource.Reset();
            ObjectPool.Return(args.TaskSource);

            args.Reset();
            ObjectPool.Return(args);
        }, args);

        ThreadPool.UnsafeQueueUserWorkItem(static args =>
        {
            try
            {
                args.This.Publish(args.Buffer);
                args.TaskSource.SetResult(true);
            }
            catch (Exception e)
            {
                args.TaskSource.SetException(e);
            }
        }, args, true);

        return new(vts, vts.Version);
    }

    public RosMessageBuffer CreateBuffer() => _introspection.CreateBuffer();

    public override void Dispose()
    {
        _deadlineMissedEvent?.Dispose();
        _qosEvent?.Dispose();
        _livelinessEvent?.Dispose();

        base.Dispose();
    }

    private class PublishArgs
    {
        public RosMessageBuffer Buffer { get; private set; }

        public RclNativePublisher This { get; private set; } = null!;

        public ManualResetValueTaskSource<bool> TaskSource { get; private set; } = null!;

        public bool ShouldDisposeBuffer { get; protected set; }

        public void Reset()
        {
            Buffer = RosMessageBuffer.Empty;
            This = null!;
            TaskSource = null!;
            ShouldDisposeBuffer = false;
        }

        public PublishArgs Init(RclNativePublisher self, RosMessageBuffer buffer,
            ManualResetValueTaskSource<bool> taskSource, bool shouldDisposeBuffer)
        {
            Buffer = buffer;
            This = self;
            TaskSource = taskSource;
            ShouldDisposeBuffer = shouldDisposeBuffer;

            return this;
        }
    }
}