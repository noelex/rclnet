using Rosidl.Runtime.Interop;
using System.Runtime.InteropServices;
using static Rcl.Interop.RclHumble;

namespace Rcl.Interop;

unsafe static class RclIron
{
    /// <summary>
    /// ROS graph ID of the topic
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public partial struct rmw_gid_t
    {
        /// <summary>
        /// Name of the rmw implementation
        /// </summary>
        public byte* implementation_identifier;

        /// <summary>
        /// Bype data Gid value
        /// </summary>
        public Guid data;
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
    /// A data structure that encapsulates the node name, node namespace,
    /// topic_type, gid, and qos_profile of publishers and subscriptions
    /// for a topic.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public partial struct rmw_topic_endpoint_info_t
    {
        /// <summary>
        /// Name of the node
        /// </summary>
        public byte* node_name;

        /// <summary>
        /// Namespace of the node
        /// </summary>
        public byte* node_namespace;

        /// <summary>
        /// The associated topic type
        /// </summary>
        public byte* topic_type;

        /// <summary>
        /// The endpoint type
        /// </summary>
        public rmw_endpoint_type_t endpoint_type;

        /// <summary>
        /// The GID of the endpoint
        /// </summary>
        public Guid endpoint_gid;

        /// <summary>
        /// QoS profile of the endpoint
        /// </summary>
        public rmw_qos_profile_t qos_profile;
    }

    /// <summary>
    /// Array of topic endpoint information
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public partial struct rmw_topic_endpoint_info_array_t
    {
        /// <summary>
        /// Size of the array.
        /// </summary>
        public size_t size;

        /// <summary>
        /// Contiguous storage for topic endpoint information elements.
        /// </summary>
        public rmw_topic_endpoint_info_t* info_array;
    }

    public enum rcl_service_introspection_state_t
    {
        /// Introspection disabled
        RCL_SERVICE_INTROSPECTION_OFF,
        /// Introspect metadata only
        RCL_SERVICE_INTROSPECTION_METADATA,
        /// Introspection metadata and contents
        RCL_SERVICE_INTROSPECTION_CONTENTS,
    }

    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
    public static extern rcl_ret_t rcl_service_configure_service_introspection(
      rcl_service_t* service,
      rcl_node_t* node,
      rcl_clock_t* clock,
      ServiceTypeSupport* type_support,
      rcl_publisher_options_t publisher_options,
      rcl_service_introspection_state_t introspection_state);

    [DllImport("rcl", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
    public static extern rcl_ret_t rcl_client_configure_service_introspection(
      rcl_client_t* client,
      rcl_node_t* node,
      rcl_clock_t* clock,
      ServiceTypeSupport* type_support,
      rcl_publisher_options_t publisher_options,
      rcl_service_introspection_state_t introspection_state);

    [DllImport("rmw_implementation", CallingConvention = CallingConvention.Cdecl)]
    public static extern rcl_ret_t rmw_get_gid_for_client(nint rmw_client_handle, void* gid);

}
