using System.Runtime.InteropServices;

namespace Rcl.Interop;

internal static class RclHumble
{
    /// <summary>
    /// Structure which encapsulates the options for creating a rcl_node_t.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public partial struct rcl_node_options_t
    {
        // Removed in humble.
        // public size_t domain_id;

        /// <summary>
        /// Custom allocator used for internal allocations.
        /// </summary>
        public rcl_allocator_t allocator;

        /// <summary>
        /// If false then only use arguments in this struct, otherwise use global arguments also.
        /// </summary>
        public bool use_global_arguments;

        /// <summary>
        /// Command line arguments that apply only to this node.
        /// </summary>
        public rcl_arguments_t arguments;

        /// <summary>
        /// Flag to enable rosout for this node
        /// </summary>
        public bool enable_rosout;
    }

    /// <summary>
    /// Return the default node options in a rcl_node_options_t.
    /// </summary>
    /// <remarks>
    /// The default values are:- domain_id = RCL_NODE_OPTIONS_DEFAULT_DOMAIN_ID
    /// - allocator = rcl_get_default_allocator()
    /// - use_global_arguments = true
    /// - enable_rosout = true
    /// - arguments = rcl_get_zero_initialized_arguments()
    /// </remarks>
    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl)]
    public static extern rcl_node_options_t rcl_node_get_default_options();
}
