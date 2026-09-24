using Rcl.Interop;
using Rcl.Qos;
using Rosidl.Runtime;

namespace Rcl.SafeHandles;

internal unsafe class SafeClientHandle : RclObjectHandle<rcl_client_t>
{
    private readonly SafeNodeHandle _node;

    internal object NativeGate { get; } = new();

    public SafeClientHandle(
        SafeNodeHandle node, SafeClockHandle clock, TypeSupportHandle typeSupportHandle, string serviceName, QosProfile qos)
    {
        _node = node;

        try
        {
            lock (node.Context.LifecycleGate)
            {
                SetDependencies(node, clock);
                *DangerousObject = rcl_get_zero_initialized_client();
                var opts = rcl_client_get_default_options();
                opts.qos = qos.ToRmwQosProfile();

                var nameSize = InteropHelpers.GetUtf8BufferSize(serviceName);
                Span<byte> nameBuffer = stackalloc byte[nameSize];
                InteropHelpers.FillUtf8Buffer(serviceName, nameBuffer);

                fixed (byte* pname = nameBuffer)
                {
                    RclException.ThrowIfNonSuccess(
                        rcl_client_init(
                            DangerousObject,
                            node.DangerousObject,
                            typeSupportHandle.GetServiceTypeSupport(),
                            pname,
                            &opts));
                }

                MarkInitialized();
            }
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    protected override bool ReleaseHandleCore(rcl_client_t* ptr)
    {
        return CheckReleaseResult(rcl_client_fini(ptr, _node.DangerousObject), nameof(rcl_client_fini));
    }
}
