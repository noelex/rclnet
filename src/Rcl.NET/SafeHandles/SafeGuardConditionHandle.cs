namespace Rcl.SafeHandles;

unsafe class SafeGuardConditionHandle : RclObjectHandle<rcl_guard_condition_t>
{
    public SafeGuardConditionHandle(SafeContextHandle context)
    {
        try
        {
            lock (context.LifecycleGate)
            {
                SetDependencies(context);
                *DangerousObject = rcl_get_zero_initialized_guard_condition();
                RclException.ThrowIfNonSuccess(
                    rcl_guard_condition_init(DangerousObject, context.DangerousObject, new()
                    {
                        allocator = RclAllocator.Default.Object
                    }));
                MarkInitialized();
            }
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    private SafeGuardConditionHandle(rcl_guard_condition_t* handle, SafeNodeHandle owner)
        : base(new(handle), owner)
    {
    }

    internal static SafeGuardConditionHandle BorrowGraphGuard(SafeNodeHandle node)
    {
        using var lease = node.Acquire();
        return new(rcl_node_get_graph_guard_condition(lease.Object), node);
    }

    protected override bool ReleaseHandleCore(rcl_guard_condition_t* ptr)
    {
        return CheckReleaseResult(rcl_guard_condition_fini(ptr), nameof(rcl_guard_condition_fini));
    }
}
