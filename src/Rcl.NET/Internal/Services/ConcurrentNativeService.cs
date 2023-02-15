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
/// This class allows handling multiple concurrent service requests asynchronously.
/// </summary>
/// <typeparam name="TService"></typeparam>
/// <typeparam name="TRequest"></typeparam>
/// <typeparam name="TResponse"></typeparam>
internal class ConcurrentNativeService<TService, TRequest, TResponse> : RclWaitObject<SafeServiceHandle>, IRclService where TService : IService<TRequest, TResponse>
    where TRequest : IServiceRequest
    where TResponse : IServiceResponse
{
    private readonly RclNodeImpl _node;
    private readonly Encoding _textEncoding;
    private readonly IConcurrentNativeServiceHandler _handler;
    private readonly CancellationTokenSource _shutdownSignal = new();

    public unsafe ConcurrentNativeService(
        RclNodeImpl node,
        string serviceName,
        IConcurrentNativeServiceHandler handler,
        QosProfile qos,
        Encoding textEncoding)
        : base(node.Context, new(node.Handle, TService.GetTypeSupportHandle(), serviceName, qos))
    {
        _node = node;
        _handler = handler;
        _textEncoding = textEncoding;
    }

    public unsafe string? Name
        => StringMarshal.CreatePooledString(rcl_service_get_service_name(Handle.Object));

    public unsafe bool IsValid
         => rcl_service_is_valid(Handle.Object);

    protected override unsafe void OnWaitCompleted()
    {
        rmw_service_info_t header;

        var request = RosMessageBuffer.Create<TRequest>();
        var response = RosMessageBuffer.Create<TResponse>();
        if (rcl_ret.RCL_RET_OK ==
            (rcl_ret)rcl_take_request_with_info(
                Handle.Object, &header, request.Data.ToPointer()).Value.Value)
        {
            _ = DispatchAsync(request, response, header.request_id);
        }
    }

    private async Task DispatchAsync(RosMessageBuffer request, RosMessageBuffer response, rmw_request_id_t requestId)
    {
        try
        {
            await _handler.ProcessRequestAsync(request, response, _shutdownSignal.Token);

            // We may resume execution on a background thread,
            // but since calling rcl_send_response is thread-safe,
            // there's no need to yield back to RclContext event loop here.

            rcl_ret_t ret;
            unsafe
            {
                ret = rcl_send_response(Handle.Object, &requestId, response.Data.ToPointer());
            }

            if (ret.Value != (int)rcl_ret.RCL_RET_OK)
            {
                // Yield here since we want to propagate the error to the event loop.
                await _node.Context.Yield();
                RclException.ThrowIfNonSuccess(ret);
            }
        }
        finally
        {
            request.Dispose();
            response.Dispose();
        }
    }

    public override void Dispose()
    {
        if (!_shutdownSignal.IsCancellationRequested)
        {
            _shutdownSignal.Cancel();
            _shutdownSignal.Dispose();
        }

        base.Dispose();
    }
}
