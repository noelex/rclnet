using System.Runtime.InteropServices;

namespace Rcl.Interop;

internal static unsafe partial class RclCommon
{
    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl)]
    public static extern rcl_ret_t rcl_timer_is_ready(rcl_timer_t* timer, bool* isReady);
}
