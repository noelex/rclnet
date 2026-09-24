namespace Rcl.SafeHandles;

internal unsafe sealed class SafeWaitSetHandle : RclObjectHandle<rcl_wait_set_t>
{
    internal SafeWaitSetHandle(SafeContextHandle context)
    {
        try
        {
            lock (context.LifecycleGate)
            {
                SetDependencies(context);
                *Object = rcl_get_zero_initialized_wait_set();
                RclException.ThrowIfNonSuccess(rcl_wait_set_init(
                    Object, 0, 0, 0, 0, 0, 0, context.DangerousObject, RclAllocator.Default.Object));
                MarkInitialized();
            }
        }
        catch { Dispose(); throw; }
    }

    protected override bool ReleaseHandleCore(rcl_wait_set_t* ptr)
        => CheckReleaseResult(rcl_wait_set_fini(ptr), nameof(rcl_wait_set_fini));
}
