using System.Runtime.InteropServices;
using static Rcl.Interop.RclHumble;
using static System.Runtime.InteropServices.JavaScript.JSType;

namespace Rcl.Interop;

internal unsafe static class RclHumble
{
    /// <summary>
    /// Transport protocol types
    /// </summary>
    public enum rmw_transport_protocol_t
    {
        RMW_TRANSPORT_PROTOCOL_UNKNOWN = 0,
        RMW_TRANSPORT_PROTOCOL_UDP,
        RMW_TRANSPORT_PROTOCOL_TCP,
        RMW_TRANSPORT_PROTOCOL_COUNT
    }

    /// <summary>
    /// Internet protocol types
    /// </summary>
    public enum rmw_internet_protocol_t
    {
        RMW_INTERNET_PROTOCOL_UNKNOWN = 0,
        RMW_INTERNET_PROTOCOL_IPV4,
        RMW_INTERNET_PROTOCOL_IPV6,
        RMW_INTERNET_PROTOCOL_COUNT
    }

    /// Structure that describes network flow endpoint of a publisher or subscription
    public struct rmw_network_flow_endpoint_t
    {
        /// <summary>
        /// Transport protocol
        /// </summary>
        public rmw_transport_protocol_t transport_protocol;

        /// <summary>
        /// Internet protocol
        /// </summary>
        public rmw_internet_protocol_t internet_protocol;

        /// <summary>
        /// Port
        /// </summary>
        public ushort transport_port;

        /// <summary>
        /// Flow label
        /// TODO(anamud): Consider specializing since flow_label is set only at publisher
        /// ... side.
        /// </summary>
        public uint flow_label;

        /// <summary>
        /// DSCP (Diff. Services Code Point)
        /// TODO(anamud): Consider specializing since DSCP is set only at publisher
        /// ... side.
        /// </summary>
        public byte dscp;

        /// <summary>
        /// Internet address, 48 bytes.
        /// </summary>
        public fixed byte internet_address[48];
    }

    /// <summary>
    /// Structure to hold an arrary of network_flow_endpoint_t
    /// </summary>
    public struct rmw_network_flow_endpoint_array_t
    {
        /// <summary>
        /// Size of the array
        /// </summary>
        public size_t size;

        /// <summary>
        /// Array of rmw_network_flow_endpoint_t
        /// </summary>
        public rmw_network_flow_endpoint_t* network_flow_endpoint;

        /// <summary>
        /// Allocator
        /// </summary>
        public rcutils_allocator_t* allocator;
    }

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
    /// 
    /// </summary>
    public struct rmw_subscription_content_filter_options_t
    {
        /// <summary>
        /// Specify the criteria to select the data samples of interest.
        ///
        /// It is similar to the WHERE part of an SQL clause.
        /// </summary>
        public byte* filter_expression;

        /// <summary>
        /// Give values to the tokens placeholder ‘parameters’ (i.e., "%n" tokens begin from 0) in the
        /// filter_expression.The number of supplied parameters must fit with the requested values.
        ///
        /// It can be NULL if there is no "%n" tokens placeholder in filter_expression.
        /// The maximum index number must be smaller than 100.
        /// </summary>
        public rcutils_string_array_t expression_parameters;
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

        /// <summary>
        /// Used to create a content filter options during subscription creation.
        /// </summary>
        public rmw_subscription_content_filter_options_t* content_filter_options;
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


    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
    public static extern rcl_ret_t rcl_subscription_options_set_content_filter_options(
          byte* filter_expression,
          size_t expression_parameters_argc,
          byte** expression_parameter_argv,
          rcl_subscription_options_t* options);

    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
    public static extern bool rcl_subscription_is_cft_enabled(rcl_subscription_t* subscription);

    [DllImport("rmw", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
    public static extern rmw_network_flow_endpoint_array_t rmw_get_zero_initialized_network_flow_endpoint_array();

    [DllImport("rmw", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
    public static extern rcl_ret_t rmw_network_flow_endpoint_array_fini(rmw_network_flow_endpoint_array_t* network_flow_endpoint_array);

    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
    public static extern rcl_ret_t rcl_publisher_get_network_flow_endpoints(
      rcl_publisher_t* publisher,
      rcl_allocator_t * allocator,
      rmw_network_flow_endpoint_array_t* network_flow_endpoint_array);

    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
    public static extern rcl_ret_t rcl_subscription_get_network_flow_endpoints(
      rcl_subscription_t* subscription,
      rcl_allocator_t * allocator,
      rmw_network_flow_endpoint_array_t* network_flow_endpoint_array);
}
