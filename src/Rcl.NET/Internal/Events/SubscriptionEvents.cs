using Rcl.SafeHandles;

namespace Rcl.Internal.Events;

internal class RclSubscriptionLivelinessChangedEvent : RclSubscriptionEvent
{
    private readonly Action<LivelinessChangedEvent> _handler;

    public RclSubscriptionLivelinessChangedEvent(
        RclContext context,
        SafeSubscriptionHandle subscription,
        Action<LivelinessChangedEvent> handler)
        : base(context, subscription, rcl_subscription_event_type_t.RCL_SUBSCRIPTION_LIVELINESS_CHANGED)
    {
        _handler = handler;
        RegisterWaitHandle();
    }

    protected override void OnLivelinessChanged(LivelinessChangedEvent info)
    {
        _handler(info);
    }
}

internal class RclSubscriptionRequestedDeadlineMissedEvent : RclSubscriptionEvent
{
    private readonly Action<RequestedDeadlineMissedEvent> _handler;

    public RclSubscriptionRequestedDeadlineMissedEvent(
        RclContext context,
        SafeSubscriptionHandle subscription,
        Action<RequestedDeadlineMissedEvent> handler)
        : base(context, subscription, rcl_subscription_event_type_t.RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED)
    {
        _handler = handler;
        RegisterWaitHandle();
    }

    protected override void OnRequestedDeadlineMissed(RequestedDeadlineMissedEvent info)
    {
        _handler(info);
    }
}

internal class RclSubscriptionRequestedIncompatibleQosEvent : RclSubscriptionEvent
{
    private readonly Action<IncompatibleQosEvent> _handler;

    public RclSubscriptionRequestedIncompatibleQosEvent(
        RclContext context,
        SafeSubscriptionHandle subscription,
        Action<IncompatibleQosEvent> handler)
        : base(context, subscription, rcl_subscription_event_type_t.RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS)
    {
        _handler = handler;
        RegisterWaitHandle();
    }

    protected override void OnIncomatibleQos(IncompatibleQosEvent info)
    {
        _handler(info);
    }
}