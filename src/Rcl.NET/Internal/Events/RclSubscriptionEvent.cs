using Rcl.Qos;
using Rcl.SafeHandles;

namespace Rcl.Internal.Events;

internal abstract class RclSubscriptionEvent : RclWaitObject<SafeSubscriptionEventHandle>
{
    private readonly rcl_subscription_event_type_t _type;

    public RclSubscriptionEvent(
        RclContext context,
        SafeSubscriptionHandle subscription,
        rcl_subscription_event_type_t type)
        : base(context, new(subscription, type))
    {
        _type = type;
        RegisterWaitHandle();
    }

    protected override unsafe void OnWaitCompleted()
    {
        switch (_type)
        {
            case rcl_subscription_event_type_t.RCL_SUBSCRIPTION_LIVELINESS_CHANGED:
                rmw_liveliness_changed_status_t lll;
                TakeEvent(&lll);
                OnLivelinessChanged(new(
                    lll.alive_count,
                    lll.not_alive_count,
                    lll.alive_count_change,
                    lll.not_alive_count_change));
                break;
            case rcl_subscription_event_type_t.RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED:
                rmw_requested_deadline_missed_status_t odm;
                TakeEvent(&odm);
                OnRequestedDeadlineMissed(new(odm.total_count, odm.total_count_change));
                break;
            case rcl_subscription_event_type_t.RCL_SUBSCRIPTION_REQUESTED_INCOMPATIBLE_QOS:
                rmw_qos_incompatible_event_status_t qos;
                TakeEvent(&qos);
                OnIncomatibleQos(new(qos.total_count, qos.total_count_change, (QosPolicyKind)qos.last_policy_kind));
                break;
            default:
                throw new NotSupportedException($"RCL subscription event type '{_type}' is not supported.");
        }
    }

    protected virtual void OnLivelinessChanged(LivelinessChangedEvent info)
    {

    }

    protected virtual void OnRequestedDeadlineMissed(RequestedDeadlineMissedEvent info)
    {

    }

    protected virtual void OnIncomatibleQos(IncompatibleQosEvent info)
    {

    }

    private unsafe void TakeEvent(void* data)
    {
        RclException.ThrowIfNonSuccess(rcl_take_event(Handle.Object, data));
    }
}