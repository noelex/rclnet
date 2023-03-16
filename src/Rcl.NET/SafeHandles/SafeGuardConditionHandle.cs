namespace Rcl.SafeHandles;

unsafe class SafeGuardConditionHandle : RclObjectHandle<rcl_guard_condition_t>
{
    public SafeGuardConditionHandle(SafeContextHandle context)
    {
        *Object = rcl_get_zero_initialized_guard_condition();
        try
        {
            RclException.ThrowIfNonSuccess(
                rcl_guard_condition_init(Object, context.Object, new() { allocator = RclAllocator.Default.Object }));
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    public SafeGuardConditionHandle(rcl_guard_condition_t* handle)
        : base(new(handle))
    {

    }

    protected override void ReleaseHandleCore(rcl_guard_condition_t* ptr)
    {
        rcl_guard_condition_fini(ptr);
    }
}
