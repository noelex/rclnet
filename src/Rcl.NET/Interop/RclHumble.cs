using System.Runtime.InteropServices;

namespace Rcl.Interop;

internal unsafe static class RclHumble
{
    public enum rmw_unique_network_flow_endpoints_requirement_t
    {
        /// Unique network flow endpoints not required
        RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_NOT_REQUIRED = 0,

        /// Unique network flow endpoins strictly required.
        /// Error if not provided by RMW implementation.
        RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED,

        /// Unique network flow endpoints optionally required.
        /// No error if not provided RMW implementation.
        RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_OPTIONALLY_REQUIRED,

        /// Unique network flow endpoints requirement decided by system.
        RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_SYSTEM_DEFAULT
    }


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

        /// Middleware quality of service settings for /rosout.
        public rmw_qos_profile_t rosout_qos;
    }

    /// <summary>
    /// Information describing an rmw message
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public partial struct rmw_message_info_t
    {
        public rcl_time_point_value_t source_timestamp;

        public rcl_time_point_value_t received_timestamp;

        public ulong publication_sequence_number;

        public ulong reception_sequence_number;

        public rmw_gid_t publisher_gid;

        /// <summary>
        /// Whether this message is from intra_process communication or not
        /// </summary>
        public bool from_intra_process;
    }

    /// <summary>
    /// Options that can be used to configure the creation of a subscription in rmw.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public partial struct rmw_subscription_options_t
    {
        /// <summary>
        /// Used to pass rmw implementation specific resources during subscription creation.
        /// </summary>
        /// <remarks>
        /// All the same details and restrictions of this field in
        /// rmw_publisher_options_t apply to this struct as well.
        /// </remarks>
        /// <seealso cref="rmw_publisher_options_t.rmw_specific_publisher_payload"/>
        public void* rmw_specific_subscription_payload;

        /// <summary>
        /// If true then the middleware should not deliver data from local publishers.
        /// </summary>
        /// <remarks>
        /// This setting is most often used when data should only be received from
        /// remote nodes, especially to avoid "double delivery" when both intra- and
        /// inter- process communication is taking place.@TODO (wjwwood): nail this down when participant mapping is sorted out.
        /// See: https://github.com/ros2/design/pull/250The definition of local is somewhat vague at the moment.
        /// Right now it means local to the node, and that definition works best, but
        /// may become more complicated when/if participants map to a context instead.
        /// </remarks>
        public bool ignore_local_publications;

        public rmw_unique_network_flow_endpoints_requirement_t require_unique_network_flow_endpoints;

        public void* content_filter_options;
    }

    /// <summary>
    /// Options available for a rcl subscription.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public partial struct rcl_subscription_options_t
    {
        /// <summary>
        /// Middleware quality of service settings for the subscription.
        /// </summary>
        public rmw_qos_profile_t qos;

        /// <summary>
        /// Custom allocator for the subscription, used for incidental allocations.
        /// For default behavior (malloc/free), see: rcl_get_default_allocator()
        /// </summary>
        public rcl_allocator_t allocator;

        /// <summary>
        /// rmw specific subscription options, e.g. the rmw implementation specific payload.
        /// </summary>
        public rmw_subscription_options_t rmw_subscription_options;
    }

    /// <summary>
    /// Options that can be used to configure the creation of a publisher in rmw.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public partial struct rmw_publisher_options_t
    {
        /// <summary>
        /// Used to pass rmw implementation specific resources during publisher creation.
        /// </summary>
        /// <remarks>
        /// This field is type erased (rather than forward declared) because it will
        /// usually be a non-owned reference to an language specific object, e.g.
        /// C++ it may be a polymorphic class that only the rmw implementation can use.The resource pointed to here needs to outlive this options structure, and
        /// any rmw_publisher objects that are created using it, as they copy this
        /// structure and may use this payload throughout their lifetime.
        /// </remarks>
        public void* rmw_specific_publisher_payload;

        public rmw_unique_network_flow_endpoints_requirement_t require_unique_network_flow_endpoints;
    }


    /// <summary>
    /// Options available for a rcl publisher.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public partial struct rcl_publisher_options_t
    {
        /// <summary>
        /// Middleware quality of service settings for the publisher.
        /// </summary>
        public rmw_qos_profile_t qos;

        /// <summary>
        /// Custom allocator for the publisher, used for incidental allocations.
        /// For default behavior (malloc/free), use: rcl_get_default_allocator()
        /// </summary>
        public rcl_allocator_t allocator;

        /// <summary>
        /// rmw specific publisher options, e.g. the rmw implementation specific payload.
        /// </summary>
        public rmw_publisher_options_t rmw_publisher_options;
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

    /// <summary>
    /// Return the default subscription options in a rcl_subscription_options_t.
    /// </summary>
    /// <remarks>
    /// The defaults are:- qos = rmw_qos_profile_default
    /// - allocator = rcl_get_default_allocator()
    /// - rmw_subscription_options = rmw_get_default_subscription_options();
    /// </remarks>
    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl)]
    public static extern rcl_subscription_options_t rcl_subscription_get_default_options();


    /// <summary>
    /// Return the default publisher options in a rcl_publisher_options_t.
    /// </summary>
    /// <remarks>
    /// The defaults are:- qos = rmw_qos_profile_default
    /// - allocator = rcl_get_default_allocator()
    /// - rmw_publisher_options = rmw_get_default_publisher_options()
    /// </remarks>
    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl)]
    public static extern rcl_publisher_options_t rcl_publisher_get_default_options();
}
