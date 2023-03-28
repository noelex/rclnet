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
}
