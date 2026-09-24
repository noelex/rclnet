using Rcl.Internal.Events;
using Rcl.Interop;
using Rcl.Logging;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System.Collections.Concurrent;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Channels;
using System.Xml.Linq;

namespace Rcl.Internal.Subscriptions;

internal unsafe class RclSubscription<T> :
    RclWaitObject<SafeSubscriptionHandle>,
    IRclSubscription<T> where T : IMessage
{
    private readonly RclNodeImpl _node;
    private readonly RosMessageBuffer _messageBuffer;
    private readonly Channel<T> _messageChannel;
    private readonly QosProfile _actualQos;
    private readonly Encoding _textEncoding;

    private readonly RclSubscriptionEvent? _livelinessEvent, _deadlineMissedEvent, _qosEvent;

    private readonly ConcurrentDictionary<int, IObserver<T>> _observers = new();
    private int _subscriberId;

    public RclSubscription(
        RclNodeImpl node,
        string topicName,
        SubscriptionOptions options)
        : base(node.Context, new(node.Handle, T.GetTypeSupportHandle(), topicName, options))
    {
        var completelyInitialized = false;

        try
        {
            using var lease = Handle.Acquire();
            _node = node;
            _messageBuffer = RosMessageBuffer.Create<T>();
            var opts = new BoundedChannelOptions(options.QueueSize)
            {
                SingleWriter = true,
                SingleReader = false,
                FullMode = options.FullMode,
                AllowSynchronousContinuations = options.AllowSynchronousContinuations
            };

            _messageChannel = Channel.CreateBounded<T>(opts);

            ref var actualQos = ref Unsafe.AsRef<rmw_qos_profile_t>(
                rcl_subscription_get_actual_qos(lease.Object));
            _actualQos = QosProfile.Create(in actualQos);

            _textEncoding = options.TextEncoding;
            Name = StringMarshal.CreatePooledString(rcl_subscription_get_topic_name(lease.Object))!;
            Endpoints = GetEndpoints();

            if (options.ContentFilter != null && !RclHumble.rcl_subscription_is_cft_enabled(lease.Object))
            {
                throw new NotSupportedException($"Content filter is configured but the feature is " +
                    $"not supported by current RMW implementation '{RosEnvironment.RmwImplementationIdentifier}'.");
            }

            InitializeEvents(options,
                ref _livelinessEvent, ref _deadlineMissedEvent, ref _qosEvent);
            RegisterWaitHandle();
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
        using var lease = Handle.Acquire();
        // TODO: Parse this as RclFoxy.rmw_message_info_t
        // if need to access header fields on foxy.
        // Defined as RclHumble.rmw_message_info_t only because it has bigger size
        // to be compatible with both foxy and humble.
        //
        // Size of the GID was changed from 24 bytes to 16 bytes since iron.
        // We can still use RclHumble.rmw_message_info_t because it's bigger in size,
        // unless we need to access rmw_message_info_t.from_intra_process.
        RclHumble.rmw_message_info_t header;

        try
        {
            if (rcl_ret_t.RCL_RET_OK == rcl_take(lease.Object, _messageBuffer.Data.ToPointer(), &header, null))
            {
                var msg = (T)T.CreateFrom(_messageBuffer.Data, _textEncoding);
                _messageChannel.Writer.TryWrite(msg);

                foreach (var (_, obs) in _observers)
                {
                    obs.OnNext(msg);
                }
            }
        }
        finally
        {
            T.UnsafeFinalize(_messageBuffer.Data);
        }
    }

    public IAsyncEnumerable<T> ReadAllAsync(CancellationToken cancellationToken)
    {
        return _messageChannel.Reader.ReadAllAsync(cancellationToken);
    }

    protected override void DisposeCore()
    {
        Cleanup.Dispose(_livelinessEvent);
        Cleanup.Dispose(_deadlineMissedEvent);
        Cleanup.Dispose(_qosEvent);

        base.DisposeCore();
    }

    protected override void OnDetached()
    {
        // Native take and synchronous observers have exited before this buffer is destroyed.
        _messageChannel?.Writer.TryComplete();

        if (!_messageBuffer.IsEmpty)
        {
            _messageBuffer.Dispose();
        }

        foreach (var (_, observer) in _observers)
            Cleanup.Run(static state => ((IObserver<T>)state!).OnCompleted(), observer);

        _observers.Clear();
    }

    public IDisposable Subscribe(IObserver<T> observer)
    {
        lock (Context.RegistrationGate)
        {
            Handle.ThrowIfOperationClosed();
            var id = Interlocked.Increment(ref _subscriberId);
            _observers[id] = observer;
            return new Subscription(this, id);
        }
    }

    private void Unsubscribe(int id)
    {
        _observers.Remove(id, out _);
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

    private class Subscription : IDisposable
    {
        private readonly int _id;
        private readonly RclSubscription<T> _owner;

        public Subscription(RclSubscription<T> owner, int id)
        {
            _owner = owner;
            _id = id;
        }

        public void Dispose()
        {
            _owner.Unsubscribe(_id);
        }
    }
}
