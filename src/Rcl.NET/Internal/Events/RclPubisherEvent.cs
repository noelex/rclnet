using Rcl.Qos;
using Rcl.SafeHandles;

namespace Rcl.Internal.Events;

internal abstract class RclPubisherEvent : RclWaitObject<SafePublisherEventHandle>
{
    private readonly rcl_publisher_event_type_t _type;

    public RclPubisherEvent(
        RclContext context,
        SafePublisherHandle publisher,
        rcl_publisher_event_type_t type)
        : base(context, new(publisher, type))
    {
        _type = type;
    }

    protected override unsafe void OnWaitCompleted()
    {
        switch (_type)
        {
            case rcl_publisher_event_type_t.RCL_PUBLISHER_LIVELINESS_LOST:
                rmw_liveliness_lost_status_t lll;
                TakeEvent(&lll);
                OnLivelinessLost(new(lll.total_count, lll.total_count_change));
                break;
            case rcl_publisher_event_type_t.RCL_PUBLISHER_OFFERED_DEADLINE_MISSED:
                rmw_offered_deadline_missed_status_t odm;
                TakeEvent(&odm);
                OnOfferedDeadlineMissed(new(odm.total_count, odm.total_count_change));
                break;
            case rcl_publisher_event_type_t.RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS:
                rmw_qos_incompatible_event_status_t qos;
                TakeEvent(&qos);
                OnIncomatibleQos(new(qos.total_count, qos.total_count_change, (QosPolicyKind)qos.last_policy_kind));
                break;
            default:
                throw new NotSupportedException($"RCL publisher event type '{_type}' is not supported.");
        }
    }

    protected virtual void OnLivelinessLost(LivelinessLostEvent info)
    {

    }

    protected virtual void OnOfferedDeadlineMissed(OfferedDeadlineMissedEvent info)
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
