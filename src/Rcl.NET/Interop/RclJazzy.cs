using System.Runtime.InteropServices;

namespace Rcl.Interop;

unsafe static class RclJazzy
{
    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl)]
    public static extern rcl_ret_t rcl_timer_init2(
        RclCommon.rcl_timer_t* timer,
        RclCommon.rcl_clock_t* clock,
        RclCommon.rcl_context_t* context,
        long period,
        delegate* unmanaged[Cdecl]<RclCommon.rcl_timer_t*, long, void> callback,
        RclCommon.rcl_allocator_t allocator,
        bool autostart);
}