using Rcl.Interop;
using Rcl.Introspection;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;

namespace Rcl.Internal.Services;

internal abstract class IntrospectionServiceBase : RclWaitObject<SafeServiceHandle>, IRclService
{
    private readonly RclNodeImpl _node;
    private readonly ServiceIntrospection _typesupport;

    public unsafe IntrospectionServiceBase(
        RclNodeImpl node,
        string serviceName,
        TypeSupportHandle typesupport,
        ServerOptions options)
        : base(node.Context, new(node.Handle, typesupport, serviceName, options.Qos))
    {
        _node = node;
        _typesupport = new ServiceIntrospection(typesupport);

        Name = StringMarshal.CreatePooledString(rcl_service_get_service_name(Handle.Object))!;
        RegisterWaitHandle();
    }

    protected virtual RosMessageBuffer CreateRequestBuffer()
        => _typesupport.Request.CreateBuffer();

    protected virtual RosMessageBuffer CreateResponseBuffer()
        => _typesupport.Response.CreateBuffer();

    public string Name { get; }

    public unsafe bool IsValid
         => rcl_service_is_valid(Handle.Object);

    protected override unsafe void OnWaitCompleted()
    {
        rmw_service_info_t header;

        var requestBuffer = CreateRequestBuffer();

        if (rcl_ret_t.RCL_RET_OK == rcl_take_request_with_info(
                Handle.Object, &header, requestBuffer.Data.ToPointer()))
        {
            var responseBuffer = CreateResponseBuffer();
            DispatchRequest(requestBuffer, responseBuffer, header.request_id);
        }
        else
        {
            OnTakeRequestFailed(requestBuffer);
        }
    }

    protected virtual void OnTakeRequestFailed(RosMessageBuffer requestBuffer)
    {
        requestBuffer.Dispose();
    }

    protected abstract unsafe void DispatchRequest(RosMessageBuffer request, RosMessageBuffer response, rmw_request_id_t id);

    public unsafe void ConfigureIntrospection(ServiceIntrospectionState state, QosProfile? qos = null)
    {
        RosEnvironment.Require(RosEnvironment.Iron, feature: "Service Introspection");

        var opts = RclHumble.rcl_publisher_get_default_options();
        opts.qos = (qos ?? QosProfile.SystemDefault).ToRmwQosProfile();

        var ret = RclIron.rcl_service_configure_service_introspection(
            Handle.Object,
            _node.Handle.Object,
            _node.Clock.Impl.Handle.Object,
            _typesupport.TypeSupportHandle,
            opts,
            (RclIron.rcl_service_introspection_state_t)state);

        RclException.ThrowIfNonSuccess(ret);
    }
}