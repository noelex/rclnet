using Rcl.Interop;
using Rcl.Qos;
using Rosidl.Runtime;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.SafeHandles;

unsafe class SafePublisherHandle : RclObjectHandle<rcl_publisher_t>
{
    private readonly SafeNodeHandle _node;

    public SafePublisherHandle(
        SafeNodeHandle node, TypeSupportHandle typeSupportHandle, string topicName, PublisherOptions options)
    {
        _node = node;
        *Object = rcl_get_zero_initialized_publisher();

        // This is backward compatible as long as we don't access
        // rmw_publisher_options.require_unique_network_flow_endpoints
        var opts = RclHumble.rcl_publisher_get_default_options();
        opts.qos = options.Qos.ToRmwQosProfile();

        if (RosEnvironment.IsSupported(RosEnvironment.Humble))
        {
            opts.rmw_publisher_options.require_unique_network_flow_endpoints =
                (RclHumble.rmw_unique_network_flow_endpoints_requirement_t)options.UniqueNetworkFlowEndpoints;
        }

        var nameSize = InteropHelpers.GetUtf8BufferSize(topicName);
        Span<byte> nameBuffer = stackalloc byte[nameSize];
        InteropHelpers.FillUtf8Buffer(topicName, nameBuffer);

        fixed (byte* pname = nameBuffer)
        {
            RclException.ThrowIfNonSuccess(
                rcl_publisher_init(
                    Object,
                    node.Object,
                    typeSupportHandle.GetMessageTypeSupport(),
                    pname,
                    &opts));
        }
    }

    protected override void ReleaseHandleCore(rcl_publisher_t* ptr)
    {
        rcl_publisher_fini(ptr, _node.Object);
    }
}
