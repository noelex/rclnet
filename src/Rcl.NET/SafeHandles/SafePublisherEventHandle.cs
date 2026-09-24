namespace Rcl.SafeHandles;

internal unsafe class SafePublisherEventHandle : RclObjectHandle<rcl_event_t>
{
    public SafePublisherEventHandle(SafePublisherHandle publisher, rcl_publisher_event_type_t eventType)
    {
        *Object = rcl_get_zero_initialized_event();
        try
        {
            RclException.ThrowIfNonSuccess(
                rcl_publisher_event_init(Object, publisher.Object, eventType));
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
