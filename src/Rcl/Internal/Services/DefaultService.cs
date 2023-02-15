using Rcl.Interop;
using Rcl.Qos;
using Rcl.SafeHandles;
using Rosidl.Runtime;
using Rosidl.Runtime.Interop;
using System.Text;

namespace Rcl.Internal.Services;

/// <summary>
/// This class handles requests sequentialy on current <see cref="RclContext"/> event loop.
/// </summary>
/// <typeparam name="TService"></typeparam>
/// <typeparam name="TRequest"></typeparam>
/// <typeparam name="TResponse"></typeparam>
internal class DefaultService<TService, TRequest, TResponse> : RclWaitObject<SafeServiceHandle>, IRclService where TService : IService<TRequest, TResponse>
    where TRequest : IServiceRequest
    where TResponse : IServiceResponse
{
    private readonly Encoding _textEncoding;
    private readonly IServiceHandler<TRequest, TResponse> _handler;

    private readonly RosMessageBuffer _requestBuffer = RosMessageBuffer.Create<TRequest>();
    private readonly RosMessageBuffer _responseBuffer = RosMessageBuffer.Create<TResponse>();

    public unsafe DefaultService(
        RclNodeImpl node,
        string serviceName,
        IServiceHandler<TRequest, TResponse> handler,
        QosProfile qos,
        Encoding textEncoding)
        : base(node.Context, new(node.Handle, TService.GetTypeSupportHandle(), serviceName, qos))
    {
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
            var response = _handler.ProcessRequest(request);
            response.WriteTo(_responseBuffer.Data, _textEncoding);

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
