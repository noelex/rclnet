namespace Rcl.SafeHandles;

internal unsafe class SafeSubscriptionEventHandle : RclObjectHandle<rcl_event_t>
{
    public SafeSubscriptionEventHandle(SafeSubscriptionHandle subscription, rcl_subscription_event_type_t eventType)
    {
        *Object = rcl_get_zero_initialized_event();
        RclException.ThrowIfNonSuccess(
            rcl_subscription_event_init(Object, subscription.Object, eventType));
    }

    protected override unsafe void ReleaseHandleCore(rcl_event_t* ptr)
    {
        rcl_event_fini(ptr);
    }
}
