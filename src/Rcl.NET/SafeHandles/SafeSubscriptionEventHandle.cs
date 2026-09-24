namespace Rcl.SafeHandles;

internal unsafe class SafeSubscriptionEventHandle : RclObjectHandle<rcl_event_t>
{
    public SafeSubscriptionEventHandle(SafeSubscriptionHandle subscription, rcl_subscription_event_type_t eventType)
    {
        *Object = rcl_get_zero_initialized_event();
        try
        {
            RclException.ThrowIfNonSuccess(
                rcl_subscription_event_init(Object, subscription.Object, eventType));
            MarkInitialized();
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    protected override unsafe bool ReleaseHandleCore(rcl_event_t* ptr)
    {
        return CheckReleaseResult(rcl_event_fini(ptr), nameof(rcl_event_fini));
    }
}
