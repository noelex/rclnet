using Rcl.Interop;
using Rcl.Qos;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace Rcl.SafeHandles;

unsafe class SafeSubscriptionHandle : RclObjectHandle<rcl_subscription_t>
{
    private readonly SafeNodeHandle _node;

    public SafeSubscriptionHandle(
        SafeNodeHandle node, TypeSupportHandle typeSupportHandle, string topicName, SubscriptionOptions options)
    {
        _node = node;
        * Object = rcl_get_zero_initialized_subscription();

        // This is backward compatible as long as we don't access
        // rmw_subscription_options.require_unique_network_flow_endpoints and
        // rmw_subscription_options.content_filter_options
        var opts = RclHumble.rcl_subscription_get_default_options();
        opts.qos = options.Qos.ToRmwQosProfile();
        opts.rmw_subscription_options.ignore_local_publications = options.IgnoreLocalPublications;

        var nameSize = InteropHelpers.GetUtf8BufferSize(topicName);
        Span<byte> nameBuffer = stackalloc byte[nameSize];
        InteropHelpers.FillUtf8Buffer(topicName, nameBuffer);

        fixed (byte* pname = nameBuffer)
        {
            RclException.ThrowIfNonSuccess(
                rcl_subscription_init(
                    Object,
                    node.Object,
                    typeSupportHandle.GetMessageTypeSupport(),
                    pname,
                    &opts));
        }
    }

    protected override void ReleaseHandleCore(rcl_subscription_t* ptr)
    {
        rcl_subscription_fini(ptr, _node.Object);
    }
}