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
            using var lease = Handle.Acquire();
            _node = node;
            ref var actualQos = ref Unsafe.AsRef<rmw_qos_profile_t>(
                rcl_publisher_get_actual_qos(lease.Object));
            _actualQos = QosProfile.Create(in actualQos);

            _introspection = MessageIntrospection.Create(typesupport);
            Name = StringMarshal.CreatePooledString(rcl_publisher_get_topic_name(lease.Object))!;
            Options = options;

            Endpoints = GetEndpoints();

            InitializePublisherEvents(options,
                ref _livelinessEvent, ref _deadlineMissedEvent, ref _qosEvent);
            RclWaitObject<SafePublisherEventHandle>.RegisterWaitHandles(Context, _livelinessEvent, _deadlineMissedEvent, _qosEvent);

            rmw_gid_t gid;
            var rmwHandle = rcl_publisher_get_rmw_handle(lease.Object);
            RclException.ThrowIfNonSuccess(rmw_get_gid_for_publisher(rmwHandle, &gid));
            Gid = new(gid.GetGidSpan()[..GraphId.Size]);

            Handle.ThrowIfOperationClosed();
            completelyInitialized = true;
        }
        finally
        {
            if (!completelyInitialized)
            {
                Dispose();
            }
        }
    }

    private unsafe NetworkFlowEndpoint[] GetEndpoints()
    {
        using var lease = Handle.Acquire();

        if (!RosEnvironment.IsSupported(RosEnvironment.Humble))
        {
            return Array.Empty<NetworkFlowEndpoint>();
        }

        var allocator = RclAllocator.Default.Object;
        var endpoints = RclHumble.rmw_get_zero_initialized_network_flow_endpoint_array();

        try
        {
            RclException.ThrowIfNonSuccess(
                RclHumble.rcl_publisher_get_network_flow_endpoints(lease.Object, &allocator, &endpoints));
            return InteropHelpers.ConvertNetworkFlowEndpoints(ref endpoints);
        }
        catch (Exception e)
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
            using var lease = Handle.Acquire();
            size_t count;
            RclException.ThrowIfNonSuccess(
                rcl_publisher_get_subscription_count(lease.Object, &count));
            return (int)count.Value;
        }
    }

    public bool IsValid
    {
        get
        {
            using var lease = Handle.Acquire();
            return rcl_publisher_is_valid(lease.Object);
        }
    }

    public NetworkFlowEndpoint[] Endpoints { get; }

    public GraphId Gid { get; }

    public void Publish(RosMessageBuffer message)
    {
        using var lease = Handle.Acquire();
        RclException.ThrowIfNonSuccess(
            rcl_publish(lease.Object, message.Data.ToPointer(), null));
    }

    public ValueTask PublishAsync(RosMessageBuffer message) => PublishAsync(message, false);

    protected ValueTask PublishAsync(RosMessageBuffer message, bool disposeBuffer)
    {
        var pending = new PendingOperation<bool>(false, static (operation, error) => operation.Fail(error));
        var args = ObjectPool.Rent<PublishArgs>().Init(this, message, pending, disposeBuffer);

        try
        {
            ThreadPool.UnsafeQueueUserWorkItem(static args => args.Run(), args, true);
        }
        catch (Exception error)
        {
            Cleanup.Run(static state => ((PublishArgs)state!).Release(), args);
            pending.Fail(error);
        }
        finally
        {
            pending.FinishSetup();
        }

        return pending.VoidTask;
    }

    public RosMessageBuffer CreateBuffer() => _introspection.CreateBuffer();

    public unsafe void AssertLiveliness()
    {
        using var lease = Handle.Acquire();
        RclException.ThrowIfNonSuccess(
            rcl_publisher_assert_liveliness(lease.Object));
    }

    protected override void DisposeCore()
    {
        Cleanup.Dispose(_deadlineMissedEvent);
        Cleanup.Dispose(_qosEvent);
        Cleanup.Dispose(_livelinessEvent);

        base.DisposeCore();
    }

    private class PublishArgs
    {
        public RosMessageBuffer Buffer { get; private set; }

        public RclNativePublisher This { get; private set; } = null!;

        public PendingOperation<bool> Completion { get; private set; } = null!;

        public bool ShouldDisposeBuffer { get; protected set; }

        public void Run()
        {
            var completion = Completion;
            Exception? failure = null;

            try
            {
                try
                {
                    This.Publish(Buffer);
                }
                finally
                {
                    Release();
                }
            }
            catch (Exception error)
            {
                failure = error;
            }

            // No access to pooled arguments after publication: a synchronous consumer
            // may start another publish before this producer returns.
            if (failure == null)
            {
                completion.Succeed(true);
            }
            else
            {
                completion.Fail(failure);
            }
        }

        public void Release()
        {
            try
            {
                if (ShouldDisposeBuffer)
                {
                    Buffer.Dispose();
                }
            }
            finally
            {
                Reset();
                ObjectPool.Return(this);
            }
        }

        public void Reset()
        {
            Buffer = RosMessageBuffer.Empty;
            This = null!;
            Completion = null!;
            ShouldDisposeBuffer = false;
        }

        public PublishArgs Init(RclNativePublisher self, RosMessageBuffer buffer,
            PendingOperation<bool> completion, bool shouldDisposeBuffer)
        {
            Buffer = buffer;
            This = self;
            Completion = completion;
            ShouldDisposeBuffer = shouldDisposeBuffer;

            return this;
        }
    }
}
