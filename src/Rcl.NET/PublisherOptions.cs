using Rcl.Qos;
using System.Text;

namespace Rcl;

public record PublisherOptions
{
    public PublisherOptions(
        QosProfile? qos = null,
        Encoding? textEncoding = null,
        Action<LivelinessLostEvent>? livelinessLostHandler = null,
        Action<OfferedDeadlineMissedEvent>? offeredDeadlineMissedHandler = null,
        Action<IncompatibleQosEvent>? offeredQosIncompatibleHandler = null)
    {
        Qos = qos ?? QosProfile.Default;
        TextEncoding = textEncoding ?? Encoding.UTF8;
        LivelinessLostHandler = livelinessLostHandler;
        OfferedDeadlineMissedHandler = offeredDeadlineMissedHandler;
        OfferedQosIncompatibleHandler = offeredQosIncompatibleHandler;
    }

    public static PublisherOptions Default { get; } = new();

    public QosProfile Qos { get; }

    public Encoding TextEncoding { get; }

    public Action<LivelinessLostEvent>? LivelinessLostHandler { get; }

    public Action<OfferedDeadlineMissedEvent>? OfferedDeadlineMissedHandler { get; }

    public Action<IncompatibleQosEvent>? OfferedQosIncompatibleHandler { get; }
}
