using Rcl.Qos;
using System.Text;
using System.Threading.Channels;

namespace Rcl;

public record SubscriptionOptions
{
    public SubscriptionOptions(
        QosProfile? qos = null,
        Encoding? textEncoding = null,
        int queueSize = 1,
        BoundedChannelFullMode fullMode = BoundedChannelFullMode.DropOldest,
        Action<LivelinessChangedEvent>? livelinessChangedHandler = null,
        Action<RequestedDeadlineMissedEvent>? requestedDeadlineMissedHandler = null,
        Action<IncompatibleQosEvent>? requestedQosIncompatibleHandler = null)
    {
        Qos = qos ?? QosProfile.Default;
        TextEncoding = textEncoding ?? Encoding.UTF8;
        QueueSize = queueSize;
        FullMode = fullMode;
        LivelinessChangedHandler = livelinessChangedHandler;
        RequestedDeadlineMissedHandler = requestedDeadlineMissedHandler;
        RequestedQosIncompatibleHandler = requestedQosIncompatibleHandler;
    }

    public static SubscriptionOptions Default { get; } = new();

    public QosProfile Qos { get; }

    public Encoding TextEncoding { get; }

    public int QueueSize { get; }

    public BoundedChannelFullMode FullMode { get; }

    public Action<LivelinessChangedEvent>? LivelinessChangedHandler { get; }
    public Action<RequestedDeadlineMissedEvent>? RequestedDeadlineMissedHandler { get; }
    public Action<IncompatibleQosEvent>? RequestedQosIncompatibleHandler { get; }
}
