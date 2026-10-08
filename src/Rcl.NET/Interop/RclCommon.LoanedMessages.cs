using Rosidl.Runtime.Interop;
using System.Runtime.InteropServices;

namespace Rcl.Interop;

internal static unsafe partial class RclCommon
{
    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl)]
    public static extern bool rcl_publisher_can_loan_messages(rcl_publisher_t* publisher);

    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl)]
    public static extern rcl_ret_t rcl_borrow_loaned_message(
        rcl_publisher_t* publisher, MessageTypeSupport* typeSupport, void** message);

    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl)]
    public static extern rcl_ret_t rcl_return_loaned_message_from_publisher(
        rcl_publisher_t* publisher, void* message);

    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl)]
    public static extern rcl_ret_t rcl_publish_loaned_message(
        rcl_publisher_t* publisher, void* message, rmw_publisher_allocation_t* allocation);

    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl)]
    public static extern bool rcl_subscription_can_loan_messages(rcl_subscription_t* subscription);

    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl)]
    public static extern rcl_ret_t rcl_take_loaned_message(
        rcl_subscription_t* subscription, void** message, void* messageInfo,
        rmw_subscription_allocation_t* allocation);

    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl)]
    public static extern rcl_ret_t rcl_return_loaned_message_from_subscription(
        rcl_subscription_t* subscription, void* message);
}
