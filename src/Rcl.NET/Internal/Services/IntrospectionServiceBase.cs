﻿using Rcl.Interop;
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
        QosProfile qos)
        : base(node.Context, new(node.Handle, typesupport, serviceName, qos))
    {
        _typesupport = new ServiceIntrospection(typesupport);
    }

    protected virtual RosMessageBuffer CreateRequestBuffer()
        => _typesupport.Request.CreateBuffer();

    protected virtual RosMessageBuffer CreateResponseBuffer()
        => _typesupport.Response.CreateBuffer();

    public unsafe string? Name
        => StringMarshal.CreatePooledString(rcl_service_get_service_name(Handle.Object));

    public unsafe bool IsValid
         => rcl_service_is_valid(Handle.Object);

    protected override unsafe void OnWaitCompleted()
    {
        rmw_service_info_t header;

        var requestBuffer = _typesupport.Request.CreateBuffer();

        if (rcl_ret.RCL_RET_OK ==
            (rcl_ret)rcl_take_request_with_info(
                Handle.Object, &header, requestBuffer.Data.ToPointer()).Value.Value)
        {
            var responseBuffer = CreateRequestBuffer();
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