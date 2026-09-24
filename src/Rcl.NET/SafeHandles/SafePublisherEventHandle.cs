namespace Rcl.SafeHandles;

internal unsafe class SafePublisherEventHandle : RclObjectHandle<rcl_event_t>
{
    public SafePublisherEventHandle(SafePublisherHandle publisher, rcl_publisher_event_type_t eventType)
    {
        try
        {
            lock (publisher.Context.LifecycleGate)
            {
                SetDependencies(publisher);
                *Object = rcl_get_zero_initialized_event();
                RclException.ThrowIfNonSuccess(
                    rcl_publisher_event_init(Object, publisher.DangerousObject, eventType));
                MarkInitialized();
            }
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
