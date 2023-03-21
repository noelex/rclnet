using Rcl.SafeHandles;

namespace Rcl.Internal.Events;

internal class RclPublisherLivelinessLostEvent : RclPubisherEvent
{
    private readonly Action<LivelinessLostEvent> _handler;

    public RclPublisherLivelinessLostEvent(
        RclContext context,
        SafePublisherHandle publisher,
        Action<LivelinessLostEvent> handler)
        : base(context, publisher, rcl_publisher_event_type_t.RCL_PUBLISHER_LIVELINESS_LOST)
    {
        _handler = handler;
        RegisterWaitHandle();
    }

    protected override void OnLivelinessLost(LivelinessLostEvent info)
    {
        _handler(info);
    }
}

internal class RclPublisherOfferedDeadlineMissedEvent : RclPubisherEvent
{
    private readonly Action<OfferedDeadlineMissedEvent> _handler;

    public RclPublisherOfferedDeadlineMissedEvent(
        RclContext context,
        SafePublisherHandle publisher,
        Action<OfferedDeadlineMissedEvent> handler)
        : base(context, publisher, rcl_publisher_event_type_t.RCL_PUBLISHER_OFFERED_DEADLINE_MISSED)
    {
        _handler = handler;
        RegisterWaitHandle();
    }

    protected override void OnOfferedDeadlineMissed(OfferedDeadlineMissedEvent info)
    {
        _handler(info);
    }
}

internal class RclPublisherIncompatibleQosEvent : RclPubisherEvent
{
    private readonly Action<IncompatibleQosEvent> _handler;

    public RclPublisherIncompatibleQosEvent(
        RclContext context,
        SafePublisherHandle publisher,
        Action<IncompatibleQosEvent> handler)
        : base(context, publisher, rcl_publisher_event_type_t.RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS)
    {
        _handler = handler;
        RegisterWaitHandle();
    }

    protected override void OnIncomatibleQos(IncompatibleQosEvent info)
    {
        _handler(info);
    }
}