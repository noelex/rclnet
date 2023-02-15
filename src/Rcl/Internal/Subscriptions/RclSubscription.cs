using Rcl.Interop;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System.Collections.Concurrent;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Channels;

namespace Rcl.Internal.Subscriptions;

internal unsafe class RclSubscription<T> :
    RclWaitObject<SafeSubscriptionHandle>,
    IRclSubscription<T> where T : IMessage
{
    private readonly RosMessageBuffer _messageBuffer = RosMessageBuffer.Create<T>();
    private readonly Channel<T> _messageChannel;
    private readonly QosProfile _actualQos;
    private readonly Encoding _textEncoding;

    private readonly ConcurrentDictionary<int, IObserver<T>> _observers = new();
    private int _subscriberId;

    public RclSubscription(
        RclNodeImpl node,
        string topicName,
        QosProfile qos,
        int queueSize,
        BoundedChannelFullMode fullMode,
        Encoding textEncoding)
        : base(node.Context, new(node.Handle, T.GetTypeSupportHandle(), topicName, qos))
    {
        var opts = new BoundedChannelOptions(queueSize)
        {
            SingleWriter = true,
            SingleReader = false,
            FullMode = fullMode,
            AllowSynchronousContinuations = true
        };

        _messageChannel = Channel.CreateBounded<T>(opts);

        ref var actualQos = ref Unsafe.AsRef<rmw_qos_profile_t>(
            rcl_subscription_get_actual_qos(Handle.Object));
        _actualQos = QosProfile.Create(in actualQos);

        _textEncoding = textEncoding;
    }

    public QosProfile ActualQos => _actualQos;

    public unsafe string? Name
        => StringMarshal.CreatePooledString(rcl_subscription_get_topic_name(Handle.Object));

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
        rmw_message_info_t header;

        if (rcl_ret.RCL_RET_OK ==
            (rcl_ret)rcl_take(Handle.Object, _messageBuffer.Data.ToPointer(), &header, null).Value.Value)
        {
            var msg = (T)T.CreateFrom(_messageBuffer.Data, _textEncoding);
            _messageChannel.Writer.TryWrite(msg);
            foreach (var (_, obs) in _observers)
            {
                obs.OnNext(msg);
            }
        }
    }

    public IAsyncEnumerable<T> ReadAllAsync(CancellationToken cancellationToken)
    {
        return _messageChannel.Reader.ReadAllAsync(cancellationToken);
    }

    public override void Dispose()
    {
        if (!IsDisposed)
        {
            _messageBuffer.Dispose();
            _messageChannel.Writer.TryComplete();
            foreach (var (_, obs) in _observers)
            {
                obs.OnCompleted();
            }
            _observers.Clear();
        }

        base.Dispose();
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