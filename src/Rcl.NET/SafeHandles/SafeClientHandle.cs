using Rcl.Interop;
using Rcl.Qos;
using Rosidl.Runtime;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.SafeHandles;

internal unsafe class SafeClientHandle : RclObjectHandle<rcl_client_t>
{
    private readonly SafeNodeHandle _node;

    public SafeClientHandle(
        SafeNodeHandle node, TypeSupportHandle typeSupportHandle, string serviceName, QosProfile qos)
    {
        _node = node;
        *Object = rcl_get_zero_initialized_client();

        var opts = rcl_client_get_default_options();
        opts.qos = qos.ToRmwQosProfile();

        var nameSize = InteropHelpers.GetUtf8BufferSize(serviceName);
        Span<byte> nameBuffer = stackalloc byte[nameSize];
        InteropHelpers.FillUtf8Buffer(serviceName, nameBuffer);

        fixed (byte* pname = nameBuffer)
        {
            RclException.ThrowIfNonSuccess(
                rcl_client_init(
                    Object,
                    node.Object,
                    (rosidl_service_type_support_t*)typeSupportHandle.GetServiceTypeSupport(),
                    pname,
                    &opts));
        }
    }

    protected override void ReleaseHandleCore(rcl_client_t* ptr)
    {
        rcl_client_fini(ptr, _node.Object);
    }
}