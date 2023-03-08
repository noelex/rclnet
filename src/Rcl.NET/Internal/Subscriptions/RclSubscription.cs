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
    private readonly RosMessageBuffer _messageBuffer = RosMessageBuffer.Create<T>();
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
        _node = node;
        var opts = new BoundedChannelOptions(options.QueueSize)
        {
            SingleWriter = true,
            SingleReader = false,
            FullMode = options.FullMode,
            AllowSynchronousContinuations = options.AllowSynchronousContinuations
        };

        _messageChannel = Channel.CreateBounded<T>(opts);

        ref var actualQos = ref Unsafe.AsRef<rmw_qos_profile_t>(
            rcl_subscription_get_actual_qos(Handle.Object));
        _actualQos = QosProfile.Create(in actualQos);

        _textEncoding = options.TextEncoding;
        Name = StringMarshal.CreatePooledString(rcl_subscription_get_topic_name(Handle.Object))!;

        var completelyInitialized = false;
        try
        {
            if (options.ContentFilter != null && !RclHumble.rcl_subscription_is_cft_enabled(Handle.Object))
            {
                throw new NotSupportedException($"Content filter is configured but the feature is " +
                    $"not supported by current RMW implementation '{RosEnvironment.RmwImplementationIdentifier}'.");
            }

            try
            {
                _livelinessEvent = new RclSubscriptionLivelinessChangedEvent(
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
                _deadlineMissedEvent = new RclSubscriptionRequestedDeadlineMissedEvent(
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
                _qosEvent = new RclSubscriptionRequestedIncompatibleQosEvent(
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

            completelyInitialized = true;
        }
        finally
        {
            if (!completelyInitialized) Dispose();
        }

        RegisterWaitHandle();
    }

    public QosProfile ActualQos => _actualQos;

    public string Name { get; }

    public int Publishers
    {
        get
        {
            size_t count;
            RclException.ThrowIfNonSuccess(
                rcl_subscription_get_publisher_count(Handle.Object, &count));
            return (int)count.Value;
        }
    }

    public bool IsValid
         => rcl_subscription_is_valid(Handle.Object);

    protected override void OnWaitCompleted()
    {
        // TODO: Parse this as RclFoxy.rmw_message_info_t
        // if need to access header fields on foxy.
        // Defined as RclHumble.rmw_message_info_t only because it has bigger size
        // to be compatible with both foxy and humble.
        RclHumble.rmw_message_info_t header;

        try
        {
            if (rcl_ret_t.RCL_RET_OK == rcl_take(Handle.Object, _messageBuffer.Data.ToPointer(), &header, null))
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

    public override void Dispose()
    {
        _node.Context.DefaultLogger.LogDebug($"Disposing RclSubscription '{Name}' ...");
        _livelinessEvent?.Dispose();
        _deadlineMissedEvent?.Dispose();
        _qosEvent?.Dispose();

        if (_messageChannel.Writer.TryComplete())
        {
            _messageBuffer.Dispose();
            foreach (var (_, obs) in _observers)
            {
                obs.OnCompleted();
            }
            _observers.Clear();
        }

        base.Dispose();
        _node.Context.DefaultLogger.LogDebug($"Disposed RclSubscription '{Name}'.");
    }

    public IDisposable Subscribe(IObserver<T> observer)
    {
        var id = Interlocked.Increment(ref _subscriberId);
        _observers[id] = observer;
        return new Subscription(this, id);
    }

    private void Unsubscribe(int id)
    {
        _observers.Remove(id, out _);
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