using Rcl.Interop;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System;
using System.Reflection.PortableExecutable;
using System.Text;

namespace Rcl.Internal.Services;

/// <summary>
/// This class allows handling multiple concurrent service requests asynchronously.
/// </summary>
/// <typeparam name="TService"></typeparam>
/// <typeparam name="TRequest"></typeparam>
/// <typeparam name="TResponse"></typeparam>
internal class ConcurrentService<TService, TRequest, TResponse> : RclWaitObject<SafeServiceHandle>, IRclService where TService : IService<TRequest, TResponse>
    where TRequest : IServiceRequest
    where TResponse : IServiceResponse
{
    private readonly RclNodeImpl _node;
    private readonly Encoding _textEncoding;
    private readonly IConcurrentServiceHandler<TRequest, TResponse> _handler;
    private readonly CancellationTokenSource _shutdownSignal = new();

    private readonly RosMessageBuffer _requestBuffer = RosMessageBuffer.Create<TRequest>();

    public unsafe ConcurrentService(
        RclNodeImpl node,
        string serviceName,
        IConcurrentServiceHandler<TRequest, TResponse> handler,
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

        if (rcl_ret.RCL_RET_OK ==
            (rcl_ret)rcl_take_request_with_info(
                Handle.Object, &header, _requestBuffer.Data.ToPointer()).Value.Value)
        {
            var request = (TRequest)TRequest.CreateFrom(_requestBuffer.Data, _textEncoding);
            _ = DispatchAsync(request, header.request_id);
        }
    }

    private async Task DispatchAsync(TRequest request, rmw_request_id_t requestId)
    {
        var response = await _handler.ProcessRequestAsync(request, _shutdownSignal.Token);

        // We may resume execution on a background thread,
        // but since calling rcl_send_response is thread-safe,
        // there's no need to yield back to RclContext event loop here.

        using var responseBuffer = RosMessageBuffer.Create<TResponse>();
        response.WriteTo(responseBuffer.Data, _textEncoding);

        rcl_ret_t ret;
        unsafe
        {
            ret = rcl_send_response(Handle.Object, &requestId, responseBuffer.Data.ToPointer());
        }

        if(ret.Value != (int)rcl_ret.RCL_RET_OK)
        {
            // Yield here since we want to propagate the error to the event loop.
            await _node.Context.Yield();
            RclException.ThrowIfNonSuccess(ret);
        }
    }

    public override void Dispose()
    {
        if (!_shutdownSignal.IsCancellationRequested)
        {
            _requestBuffer.Dispose();
            _shutdownSignal.Cancel();
            _shutdownSignal.Dispose();
        }

        base.Dispose();
    }
}
