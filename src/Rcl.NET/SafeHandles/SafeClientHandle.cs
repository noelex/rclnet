using Rcl.Interop;
using Rcl.Qos;
using Rosidl.Runtime;

namespace Rcl.SafeHandles;

internal unsafe class SafeClientHandle : RclObjectHandle<rcl_client_t>
{
    private readonly SafeNodeHandle _node;

    public SafeClientHandle(
        SafeNodeHandle node, TypeSupportHandle typeSupportHandle, string serviceName, QosProfile qos)
    {
        _node = node;
        *Object = rcl_get_zero_initialized_client();

        try
        {
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
                        typeSupportHandle.GetServiceTypeSupport(),
                        pname,
                        &opts));
            }
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    protected override void ReleaseHandleCore(rcl_client_t* ptr)
    {
        rcl_client_fini(ptr, _node.Object);
    }
}