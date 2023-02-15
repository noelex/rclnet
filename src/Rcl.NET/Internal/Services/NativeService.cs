using Rcl.Interop;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime.Interop;
using Rosidl.Runtime;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Metadata;
using System.Text;
using System.Threading.Tasks;

namespace Rcl.Internal.Services;

/// <summary>
/// Handle requests using native message buffers.
/// </summary>
/// <typeparam name="TService"></typeparam>
/// <typeparam name="TRequest"></typeparam>
/// <typeparam name="TResponse"></typeparam>
internal class NativeService<TService, TRequest, TResponse> : RclWaitObject<SafeServiceHandle>, IRclService where TService : IService<TRequest, TResponse>
    where TRequest : IServiceRequest
    where TResponse : IServiceResponse
{
    private readonly INativeServiceHandler _handler;

    private readonly RosMessageBuffer _requestBuffer = RosMessageBuffer.Create<TRequest>();
    private readonly RosMessageBuffer _responseBuffer = RosMessageBuffer.Create<TResponse>();

    public unsafe NativeService(
        RclNodeImpl node,
        string serviceName,
        INativeServiceHandler handler,
        QosProfile qos)
        : base(node.Context, new(node.Handle, TService.GetTypeSupportHandle(), serviceName, qos))
    {
        _handler = handler;
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