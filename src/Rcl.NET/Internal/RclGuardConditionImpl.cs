using Rcl.SafeHandles;

namespace Rcl.Internal;

internal class RclGuardConditionImpl : RclWaitObject<SafeGuardConditionHandle>, IRclGuardCondition
{
    public RclGuardConditionImpl(RclContext context)
        : base(context, new(context.Handle))
    {
        RegisterWaitHandle();
    }

    public RclGuardConditionImpl(RclContext context, SafeGuardConditionHandle handle)
        : base(context, handle)
    {
        RegisterWaitHandle();
    }

    /// <summary>
    /// Trigger the <see cref="RclGuardConditionImpl"/> once, allowing pending awaiters to continue.
    /// </summary>
    public unsafe void Trigger()
    {
        rcl_trigger_guard_condition(Handle.Object);
    }
}