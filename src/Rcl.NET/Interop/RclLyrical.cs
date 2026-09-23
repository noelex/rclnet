using System.Runtime.InteropServices;
using static Rcl.Interop.RclHumble;

namespace Rcl.Interop;

internal unsafe static class RclLyrical
{
    [StructLayout(LayoutKind.Sequential)]
    public struct rmw_subscription_options_t
    {
        public void* rmw_specific_subscription_payload;

        public bool ignore_local_publications;

        public rmw_unique_network_flow_endpoints_requirement_t require_unique_network_flow_endpoints;

        public rmw_subscription_content_filter_options_t* content_filter_options;

        public byte* acceptable_buffer_backends;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct rcl_subscription_options_t
    {
        public rmw_qos_profile_t qos;

        public rcl_allocator_t allocator;

        public rmw_subscription_options_t rmw_subscription_options;

        public bool disable_loaned_message;
    }

    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
    public static extern rcl_subscription_options_t rcl_subscription_get_default_options();

    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
    public static extern rcl_ret_t rcl_subscription_options_set_content_filter_options(
        byte* filter_expression,
        size_t expression_parameters_argc,
        byte** expression_parameter_argv,
        rcl_subscription_options_t* options);
}
