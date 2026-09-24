using System.Runtime.InteropServices;

namespace Rcl.Interop;

internal static unsafe partial class RclCommon
{
    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl)]
    public static extern nint rcl_guard_condition_get_rmw_handle(rcl_guard_condition_t* guardCondition);
}
