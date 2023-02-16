using Rcl.Interop;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime.Interop;
using Rosidl.Runtime;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rcl.Introspection;

namespace Rcl.Internal.Services;

internal class IntrospectionService : RclWaitObject<SafeServiceHandle>, IRclService
{
    private readonly INativeServiceHandler _handler;
    private readonly ServiceIntrospection _typesupport;

    private readonly RosMessageBuffer _requestBuffer, _responseBuffer;

    public unsafe IntrospectionService(
        RclNodeImpl node,
        string serviceName,
        TypeSupportHandle typesupport,
        INativeServiceHandler handler,
        QosProfile qos)
        : base(node.Context, new(node.Handle, typesupport, serviceName, qos))
    {
        _handler = handler;
        _typesupport = new ServiceIntrospection(typesupport);

        _requestBuffer = _typesupport.Request.CreateBuffer();
        _responseBuffer = _typesupport.Response.CreateBuffer();
    }

    public unsafe string? Name
        => StringMarshal.CreatePooledString(rcl_service_get_service_name(Handle.Object));

    public unsafe bool IsValid
         => rcl_service_is_valid(Handle.Object);

    protected override unsafe void OnWaitCompleted()
    {
        rmw_service_info_t header;

        if (rcl_ret.RCL_RET_OK ==
            (rcl_ret)rcl_take_request_with_info(
                Handle.Object, &header, _requestBuffer.Data.ToPointer()).Value.Value)
        {
            _handler.ProcessRequest(_requestBuffer, _responseBuffer);

            RclException.ThrowIfNonSuccess(
                rcl_send_response(Handle.Object, &header.request_id, _responseBuffer.Data.ToPointer()));
        }
    }

    public override void Dispose()
    {
        if (!IsDisposed)
        {
            _requestBuffer.Dispose();
            _responseBuffer.Dispose();
        }

        base.Dispose();
    }
}