using Rcl.Interop;
using Rcl.Qos;
using Rosidl.Runtime;

namespace Rcl.SafeHandles;

internal unsafe class SafeServiceHandle : RclObjectHandle<rcl_service_t>
{
    private readonly SafeNodeHandle _node;

    public SafeServiceHandle(
        SafeNodeHandle node, TypeSupportHandle typeSupportHandle, string serviceName, QosProfile qos)
    {
        _node = node;
        *Object = rcl_get_zero_initialized_service();
        try
        {
            var opts = rcl_service_get_default_options();
            opts.qos = qos.ToRmwQosProfile();

            var nameSize = InteropHelpers.GetUtf8BufferSize(serviceName);
            Span<byte> nameBuffer = stackalloc byte[nameSize];
            InteropHelpers.FillUtf8Buffer(serviceName, nameBuffer);

            fixed (byte* pname = nameBuffer)
            {
                RclException.ThrowIfNonSuccess(
                    rcl_service_init(
                        Object,
                        node.Object,
                        typeSupportHandle.GetServiceTypeSupport(),
                        pname,
                        &opts));
            }
            MarkInitialized();
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    protected override bool ReleaseHandleCore(rcl_service_t* ptr)
    {
        return CheckReleaseResult(rcl_service_fini(ptr, _node.Object), nameof(rcl_service_fini));
    }
}