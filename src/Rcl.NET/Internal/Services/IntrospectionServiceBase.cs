using Rcl.Interop;
using Rcl.Introspection;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;

namespace Rcl.Internal.Services;

internal abstract class IntrospectionServiceBase : RclWaitObject<SafeServiceHandle>, IRclService
{
    private readonly ServiceIntrospection _typesupport;

    public unsafe IntrospectionServiceBase(
        RclNodeImpl node,
        string serviceName,
        TypeSupportHandle typesupport,
        ServerOptions options)
        : base(node.Context, new(node.Handle, typesupport, serviceName, options.Qos))
    {
        _typesupport = new ServiceIntrospection(typesupport);

        Name = StringMarshal.CreatePooledString(rcl_service_get_service_name(Handle.Object))!;
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
}