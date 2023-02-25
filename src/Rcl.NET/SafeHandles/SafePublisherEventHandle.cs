namespace Rcl.SafeHandles;

internal unsafe class SafePublisherEventHandle : RclObjectHandle<rcl_event_t>
{
    public SafePublisherEventHandle(SafePublisherHandle publisher, rcl_publisher_event_type_t eventType)
    {
        *Object = rcl_get_zero_initialized_event();
        RclException.ThrowIfNonSuccess(
            rcl_publisher_event_init(Object, publisher.Object, eventType));
    }

    protected override unsafe void ReleaseHandleCore(rcl_event_t* ptr)
    {
        rcl_event_fini(ptr);
    }
}
