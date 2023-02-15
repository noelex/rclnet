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
        SafeNodeHandle node, TypeSupportHandle typeSupportHandle, string topicName, QosProfile qos)
    {
        _node = node;
        *Object = rcl_get_zero_initialized_publisher();

        var opts = rcl_publisher_get_default_options();
        opts.qos = qos.ToRmwQosProfile();

        var nameSize = InteropHelpers.GetUtf8BufferSize(topicName);
        Span<byte> nameBuffer = stackalloc byte[nameSize];
        InteropHelpers.FillUtf8Buffer(topicName, nameBuffer);

        fixed (byte* pname = nameBuffer)
        {
            RclException.ThrowIfNonSuccess(
                rcl_publisher_init(
                    Object,
                    node.Object,
                    (rosidl_message_type_support_t*)typeSupportHandle.GetMessageTypeSupport(),
                    pname,
                    &opts));
        }
    }

    protected override void ReleaseHandleCore(rcl_publisher_t* ptr)
    {
        rcl_publisher_fini(ptr, _node.Object);
    }
}
