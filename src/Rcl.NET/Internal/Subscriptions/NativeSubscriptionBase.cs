using Rcl.Internal.Events;
using Rcl.Interop;
using Rcl.Logging;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System.Runtime.CompilerServices;
using System.Threading.Channels;

namespace Rcl.Internal.Subscriptions;

internal unsafe abstract class NativeSubscriptionBase :
    RclWaitObject<SafeSubscriptionHandle>,
    IRclNativeSubscription
{
    private readonly RclNodeImpl _node;
    private readonly Channel<RosMessageBuffer> _messageChannel;
    private readonly QosProfile _actualQos;

    private readonly RclSubscriptionEvent? _livelinessEvent, _deadlineMissedEvent, _qosEvent;

    public NativeSubscriptionBase(
        RclNodeImpl node,
        string topicName,
        TypeSupportHandle typeSupport,
        SubscriptionOptions options)
        : base(node.Context, new(node.Handle, typeSupport, topicName, options))
    {
        var completelyInitialized = false;
        try
        {
            using var lease = Handle.Acquire();
            _node = node;
            TypeSupport = typeSupport;

            var opts = new BoundedChannelOptions(options.QueueSize)
            {
                SingleWriter = true,
                SingleReader = false,
                FullMode = options.FullMode,
                AllowSynchronousContinuations = options.AllowSynchronousContinuations
            };

            _messageChannel = Channel.CreateBounded<RosMessageBuffer>(opts, static x => x.Dispose());

            ref var actualQos = ref Unsafe.AsRef<rmw_qos_profile_t>(
                rcl_subscription_get_actual_qos(lease.Object));
            _actualQos = QosProfile.Create(in actualQos);

            Name = StringMarshal.CreatePooledString(rcl_subscription_get_topic_name(lease.Object))!;
            Options = options;
            Endpoints = GetEndpoints();

            if (options.ContentFilter != null && !RclHumble.rcl_subscription_is_cft_enabled(lease.Object))
            {
                throw new NotSupportedException($"Content filter is configured but the feature is " +
                    $"not supported by current RMW implementation '{RosEnvironment.RmwImplementationIdentifier}'.");
            }

            InitializeEvents(options,
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
                RclHumble.rcl_subscription_get_network_flow_endpoints(lease.Object, &allocator, &endpoints));
            return InteropHelpers.ConvertNetworkFlowEndpoints(ref endpoints);
        }
        catch (Exception e)
        {
            _node.Context.DefaultLogger.LogDebug("Unable to retrieve network flow endpoints from subscription: " + e.Message);
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

    private void InitializeEvents(
        SubscriptionOptions options,
        ref RclSubscriptionEvent? livelinessEvent,
        ref RclSubscriptionEvent? deadlineMissedEvent,
        ref RclSubscriptionEvent? qosEvent)
    {
        try
        {
            livelinessEvent = new RclSubscriptionLivelinessChangedEvent(
                Context, Handle,
                options.LivelinessChangedHandler ?? OnLivelinessChanged);
        }
        catch (RclException ex)
        {
            if (options.LivelinessChangedHandler != null)
            {
                throw;
            }
            _node.Context.DefaultLogger.LogDebug("Unable to register LivelinessChangedEvent:");
            _node.Context.DefaultLogger.LogDebug(ex.Message);
        }

        try
        {
            deadlineMissedEvent = new RclSubscriptionRequestedDeadlineMissedEvent(
                Context, Handle,
                options.RequestedDeadlineMissedHandler ?? OnDeadlineMissed);
        }
        catch (RclException ex)
        {
            if (options.RequestedDeadlineMissedHandler != null)
            {
                throw;
            }
            _node.Context.DefaultLogger.LogDebug("Unable to register RequestedDeadlineMissedEvent:");
            _node.Context.DefaultLogger.LogDebug(ex.Message);
        }

        try
        {
            qosEvent = new RclSubscriptionRequestedIncompatibleQosEvent(
                Context, Handle,
                options.RequestedQosIncompatibleHandler ?? OnIncompatibleQos);
        }
        catch (RclException ex)
        {
            if (options.RequestedQosIncompatibleHandler != null)
            {
                throw;
            }
            _node.Context.DefaultLogger.LogDebug("Unable to register RequestedQosIncompatibleEvent:");
            _node.Context.DefaultLogger.LogDebug(ex.Message);
        }
    }

    private void OnLivelinessChanged(LivelinessChangedEvent info)
    {
        _node.Logger.LogDebug(
            $"Received LivelinessChangedEvent on subscription of topic '{Name}': " +
            $"Alive = {info.AliveCount}, AliveDelta = {info.AliveCountDelta}, " +
            $"NotAlive = {info.NotAliveCount}, NotAliveDelta = {info.NotAliveCountDelta}");
    }

    private void OnDeadlineMissed(RequestedDeadlineMissedEvent info)
    {
        _node.Logger.LogWarning(
            $"Received RequestedDeadlineMissedEvent on subscription of topic '{Name}': " +
            $"Total = {info.TotalCount}, Delta = {info.Delta}");
    }

    private void OnIncompatibleQos(IncompatibleQosEvent info)
    {
        _node.Logger.LogWarning(
           $"Received IncompatibleQosEvent on subscription of topic '{Name}': " +
           $"Total = {info.TotalCount}, Delta = {info.Delta}, PolicyKind = {info.LastPolicyKind}");
    }

    public SubscriptionOptions Options { get; }

    protected TypeSupportHandle TypeSupport { get; }

    public QosProfile ActualQos => _actualQos;

    public string Name { get; }

    public int Publishers
    {
        get
        {
            using var lease = Handle.Acquire();
            size_t count;
            RclException.ThrowIfNonSuccess(
                rcl_subscription_get_publisher_count(lease.Object, &count));
            return (int)count.Value;
        }
    }

    public bool IsValid
    {
        get
        {
            using var lease = Handle.Acquire();
            return rcl_subscription_is_valid(lease.Object);
        }
    }

    public NetworkFlowEndpoint[] Endpoints { get; }

    protected override void OnWaitCompleted()
    {
        var msg = TakeMessage();
        if (!msg.IsEmpty)
        {
            // Just in case FullMode is set to Wait, simply drop the incoming message.
            if (!_messageChannel.Writer.TryWrite(msg))
            {
                msg.Dispose();
            }
        }
    }

    protected abstract RosMessageBuffer TakeMessage();

    public IAsyncEnumerable<RosMessageBuffer> ReadAllAsync(CancellationToken cancellationToken)
    {
        return _messageChannel.Reader.ReadAllAsync(cancellationToken);
    }

    protected override void DisposeCore()
    {
        _livelinessEvent?.Dispose();
        _deadlineMissedEvent?.Dispose();
        _qosEvent?.Dispose();

        if (_messageChannel?.Writer.TryComplete() == true)
        {
            while (_messageChannel.Reader.TryRead(out var buffer))
            {
                buffer.Dispose();
            }
        }

        base.DisposeCore();
    }
}