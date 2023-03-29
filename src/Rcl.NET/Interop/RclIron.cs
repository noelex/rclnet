using System.Runtime.InteropServices;

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
}
