using Rcl.Interop;
using Rosidl.Runtime;

namespace Rcl.SafeHandles;

unsafe class SafePublisherHandle : RclObjectHandle<rcl_publisher_t>
{
    private readonly SafeNodeHandle _node;

    public SafePublisherHandle(
        SafeNodeHandle node, TypeSupportHandle typeSupportHandle, string topicName, PublisherOptions options)
    {
        _node = node;
        *Object = rcl_get_zero_initialized_publisher();

        try
        {
            var nameSize = InteropHelpers.GetUtf8BufferSize(topicName);
            Span<byte> nameBuffer = stackalloc byte[nameSize];
            InteropHelpers.FillUtf8Buffer(topicName, nameBuffer);

            fixed (byte* pname = nameBuffer)
            {
                if (RosEnvironment.IsFoxy)
                {
                    InitFoxy(pname, typeSupportHandle, options);
                }
                else if (RosEnvironment.IsHumble)
                {
                    InitHumble(pname, typeSupportHandle, options);
                }
                else
                {
                    InitIronOrLater(pname, typeSupportHandle, options);
                }
            }
            MarkInitialized();
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    private void InitFoxy(byte* name, TypeSupportHandle typeSupport, PublisherOptions options)
    {
        var nativeOptions = RclFoxy.rcl_publisher_get_default_options();
        nativeOptions.qos = options.Qos.ToRmwQosProfile();

        RclException.ThrowIfNonSuccess(
            rcl_publisher_init(
                Object,
                _node.Object,
                typeSupport.GetMessageTypeSupport(),
                name,
                &nativeOptions));
    }

    private void InitHumble(byte* name, TypeSupportHandle typeSupport, PublisherOptions options)
    {
        var nativeOptions = RclHumble.rcl_publisher_get_default_options();
        nativeOptions.qos = options.Qos.ToRmwQosProfile();
        nativeOptions.rmw_publisher_options.require_unique_network_flow_endpoints =
            (RclHumble.rmw_unique_network_flow_endpoints_requirement_t)options.UniqueNetworkFlowEndpoints;

        RclException.ThrowIfNonSuccess(
            rcl_publisher_init(
                Object,
                _node.Object,
                typeSupport.GetMessageTypeSupport(),
                name,
                &nativeOptions));
    }

    private void InitIronOrLater(byte* name, TypeSupportHandle typeSupport, PublisherOptions options)
    {
        var nativeOptions = RclIron.rcl_publisher_get_default_options();
        nativeOptions.qos = options.Qos.ToRmwQosProfile();
        nativeOptions.rmw_publisher_options.require_unique_network_flow_endpoints =
            (RclHumble.rmw_unique_network_flow_endpoints_requirement_t)options.UniqueNetworkFlowEndpoints;

        RclException.ThrowIfNonSuccess(
            rcl_publisher_init(
                Object,
                _node.Object,
                typeSupport.GetMessageTypeSupport(),
                name,
                &nativeOptions));
    }

    protected override bool ReleaseHandleCore(rcl_publisher_t* ptr)
    {
        return CheckReleaseResult(rcl_publisher_fini(ptr, _node.Object), nameof(rcl_publisher_fini));
    }
}
